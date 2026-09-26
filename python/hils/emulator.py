"""실물 대체 에뮬레이터: 챔버(부하장치) + 히트펌프(자체 제어기) + PLC 레지스터 로직.

히트펌프는 Simulink 명령을 받지 않는다(native control 보존).
자체 온도조절기(PI + 최소 off 시간 + 히스테리시스)로 챔버 온도를 hp_setpoint 로 유지한다.
PLC 는 Simulink 명령 프레임을 FIFO 로 ack_delay_s 만큼 지연 적용하고, 적용된 프레임의
Sequence 를 AckSequence 로 돌려준다.
"""
import random
from collections import deque

from .registers import RegisterMap

HP_OFF, HP_HEATING, HP_COOLING, HP_DEFROST, HP_FAULT = 0, 1, 2, 3, 9


class HeatPumpNative:
    def __init__(self, e):
        self.e = e
        self.Q = 0.0
        self.integ = 0.0
        self.on = False
        self.off_timer = 1e9
        self.status = HP_OFF
        self.fault = False

    def step(self, dt, T_room, T_out):
        e = self.e
        err = e["hp_setpoint"] - T_room
        if self.fault:
            self.on, demand, self.status = False, 0.0, HP_FAULT
        else:
            if not self.on:
                self.off_timer += dt
                if err > e["hp_hyst"] and self.off_timer >= e["hp_min_off_s"]:
                    self.on, self.integ = True, e["hp_Qmin"]
            demand = 0.0
            if self.on:
                self.integ += e["hp_Kp"] / e["hp_Ti_s"] * err * dt
                self.integ = min(max(self.integ, 0.0), e["hp_Qmax"])
                demand = min(max(e["hp_Kp"] * err + self.integ, e["hp_Qmin"]), e["hp_Qmax"])
                if err < -e["hp_hyst"] and self.integ <= e["hp_Qmin"]:
                    self.on, self.off_timer, demand = False, 0.0, 0.0
            self.status = HP_HEATING if self.on else HP_OFF
        self.Q += dt / e["hp_tau_s"] * (demand - self.Q)   # 압축기/열교환기 동특성
        T_cond, T_evap = T_room + 10.0 + 273.15, T_out - 5.0 + 273.15
        cop = max(1.0, e["hp_carnot_eff"] * T_cond / max(T_cond - T_evap, 5.0))
        P = self.Q / cop + (30.0 if self.on else 5.0)       # 제어기/팬 대기전력 포함
        return self.Q, P, cop


class ChamberPlant:
    """실내측 챔버 공기노드 + 부하장치 + 실외측 챔버."""

    def __init__(self, e, T0=20.0, T_out0=0.0, seed=0):
        self.e = e
        self.T = T0
        self.T_out = T_out0
        self.Q_load = 0.0
        self.hp = HeatPumpNative(e)
        self.rng = random.Random(seed)
        self.hp_Q = self.hp_P = 0.0

    def step(self, dt, Q_load_cmd, load_enable, T_sp, T_out_sp):
        e = self.e
        if load_enable:
            target = e["load_gain_error"] * Q_load_cmd + e["load_bias_W"]
        else:
            # 부하모드 비활성: 챔버 공조가 T_chamber_SP 로 사전조화(P 제어)
            target = -800.0 * (T_sp - self.T)
            target = min(max(target, -4000.0), 4000.0)
        self.Q_load += dt / e["load_tau_s"] * (target - self.Q_load)
        self.T_out += dt / 300.0 * (T_out_sp - self.T_out)
        self.hp_Q, self.hp_P, _ = self.hp.step(dt, self.T, self.T_out)
        dT = (self.hp_Q - self.Q_load + e["UA_chamber_lab"] * (e["T_lab"] - self.T)) / e["C_chamber"]
        self.T += dt * dT

    def measure(self):
        n = self.e["meas_noise_W"]
        return {
            "T_indoor": self.T + self.rng.gauss(0, 0.02) * (n > 0),
            "RH_indoor": 40.0,
            "T_outdoor": self.T_out,
            "P_HP": self.hp_P,
            "Q_HP": self.hp_Q + (self.rng.gauss(0, n) if n > 0 else 0.0),
            "HP_status": self.hp.status,
            "Q_load_meas": self.Q_load + (self.rng.gauss(0, n) if n > 0 else 0.0),
        }


class PLCEmulator:
    """PLC 스캔 로직. registers: 0-based offset -> uint16 인 list 류(공유 메모리 또는 Modbus datastore)."""

    def __init__(self, cfg, bank, seed=0):
        self.cfg = cfg
        self.rm = RegisterMap(cfg)
        self.bank = bank
        e = cfg["emulator"]
        self.plant = ChamberPlant(e, seed=seed)
        self.Ts = cfg["timing"]["Ts_plc"]
        self.dt = cfg["timing"]["Ts_plant_internal"]
        self.delay_steps = int(round(e["ack_delay_s"] / self.Ts))
        self.fifo = deque()
        self.applied = {"Q_load_cmd": 0.0, "T_chamber_SP": e["hp_setpoint"], "Enable": 0,
                        "Sequence": 0, "T_outdoor_SP": 0.0}
        self.hb = 0
        self.estop = False
        self.freeze_heartbeat = False  # 통신두절 시험용
        self.wd_last = None            # PLC 측 watchdog: SIM_heartbeat 감시
        self.wd_age = 0.0
        self.watchdog = False
        self.t = 0.0
        self._write_outputs()

    def _read_commands(self):
        return {n: self.rm.decode(n, self.bank[self.rm.offset(n)]) for n in self.rm.names("SIM2PLC")}

    def _write_outputs(self):
        m = self.plant.measure()
        bits = self.cfg["plc_status_bits"]
        st = 1 << bits["READY"]
        if self.estop:
            st |= 1 << bits["ESTOP"]
        if self.applied["Enable"] and not self.watchdog:
            st |= 1 << bits["LOAD_ACTIVE"]
        if self.watchdog:
            st |= 1 << bits["WATCHDOG"]
        m.update({"AckSequence": self.applied["Sequence"], "PLC_status": st, "PLC_heartbeat": self.hb})
        for n, v in m.items():
            self.bank[self.rm.offset(n)], _ = self.rm.encode(n, v)

    def scan(self):
        """1 PLC 주기(Ts_plc): 명령 읽기 -> 지연 FIFO -> 플랜트 적분 -> 측정 쓰기."""
        cmd = self._read_commands()
        # watchdog: Simulink heartbeat 가 멈추면 PLC 가 스스로 부하장치를 해제 (Simulink 측 안전로직과 이중화)
        if cmd["SIM_heartbeat"] != self.wd_last:
            self.wd_last, self.wd_age = cmd["SIM_heartbeat"], 0.0
        else:
            self.wd_age += self.Ts
        self.watchdog = self.wd_age > self.cfg["emulator"]["plc_watchdog_s"]
        self.fifo.append(cmd)
        if len(self.fifo) > self.delay_steps:
            self.applied = self.fifo.popleft()
        if self.estop:
            self.applied = dict(self.applied, Enable=0)
        a = dict(self.applied, Enable=0) if self.watchdog else self.applied
        for _ in range(int(round(self.Ts / self.dt))):
            self.plant.step(self.dt, a["Q_load_cmd"], bool(a["Enable"]), a["T_chamber_SP"], a["T_outdoor_SP"])
        if not self.freeze_heartbeat:
            self.hb = (self.hb + 1) & 0xFFFF
        self.t += self.Ts
        self._write_outputs()

"""실물 대체 에뮬레이터: PLC 레지스터 로직 + 실내측 챔버(국부 PID) + 히트펌프 실내기 + 실외측 챔버.

- 챔버: PLC 국부 PI 가 T_room_SP / RH_room_SP 를 추종 (가열·냉각 Q_cond, 가습·제습 m_hum).
- 히트펌프: Simulink 명령을 받지 않는다(native control). 리턴공기 온도를 자기 설정온도로 맞추는
  PI + 최소 off 시간 + 히스테리시스. 냉방 코일은 bypass-factor/ADP 모델 (습코일 제습).
- 측정: 토출/리턴 T·RH, 노즐 체적풍량(토출 공기 기준), 대기압. 가우스 노이즈.
- PLC: 명령 프레임을 ack_delay_s 만큼 FIFO 지연 적용, 적용 프레임의 Sequence 를 AckSequence 로 회신,
  SIM_heartbeat watchdog, E-stop. Enable=0 이면 마지막 적용 설정값을 유지(hold).
MATLAB hils_plc_emulator_*.m 과 동일 로직 (노이즈 0 에서 교차검증).
"""
import random
from collections import deque

from .psychro import (cp_moist, enthalpy, rh_from_w, spec_volume, t_from_h_w, t_sat_from_h,
                      w_from_rh)
from .registers import RegisterMap

HP_OFF, HP_HEATING, HP_COOLING, HP_DEFROST, HP_FAULT = 0, 1, 2, 3, 9


def coil_outlet(T_r, W_r, Q_tot, m_da, mode, BF, P):
    """실내기 출구 공기 (T_s, W_s). Q_tot [W] >= 0 은 공급(난방) 또는 제거(냉방) 열량 크기."""
    if m_da <= 0.0 or Q_tot <= 0.0:
        return T_r, W_r
    if mode == "heat":
        return T_r + Q_tot / (1000.0 * m_da * cp_moist(W_r)), W_r
    h_r = enthalpy(T_r, W_r)
    h_s = h_r - Q_tot / (1000.0 * m_da)
    h_adp = (h_s - BF * h_r) / (1.0 - BF)
    T_adp = t_sat_from_h(h_adp, P)
    W_adp = w_from_rh(T_adp, 100.0, P)
    if W_adp >= W_r:                       # 건코일: 현열만
        W_s = W_r
    else:                                  # 습코일: 제습
        W_s = BF * W_r + (1.0 - BF) * W_adp
    return t_from_h_w(h_s, W_s), W_s


class HeatPumpNative:
    """실내기 native 제어기. 리턴공기 온도만 보고 운전 (외부 명령 없음)."""

    def __init__(self, e):
        self.e = e
        self.mode = e["hp_mode"]
        self.set = e["hp_setpoint_heat"] if self.mode == "heat" else e["hp_setpoint_cool"]
        self.Q = 0.0          # 공급(난방) 또는 제거(냉방) 열량 크기 [W]
        self.integ = 0.0
        self.on = False
        self.off_timer = 1e9
        self.status = HP_OFF
        self.P = 0.0
        self.fault = False

    def step(self, dt, T_r, T_out):
        e = self.e
        err = (self.set - T_r) if self.mode == "heat" else (T_r - self.set)
        demand = 0.0
        if self.fault:
            self.on = False
        else:
            if not self.on:
                self.off_timer += dt
                if err > e["hp_hyst"] and self.off_timer >= e["hp_min_off_s"]:
                    self.on, self.integ = True, e["hp_Qmin"]
            if self.on:
                self.integ = min(max(self.integ + e["hp_Kp"] / e["hp_Ti_s"] * err * dt, 0.0), e["hp_Qmax"])
                demand = min(max(e["hp_Kp"] * err + self.integ, e["hp_Qmin"]), e["hp_Qmax"])
                if err < -e["hp_hyst"] and self.integ <= e["hp_Qmin"]:
                    self.on, self.off_timer, demand = False, 0.0, 0.0
        if self.fault:
            self.status = HP_FAULT
        elif self.on:
            self.status = HP_HEATING if self.mode == "heat" else HP_COOLING
        else:
            self.status = HP_OFF
        self.Q += dt / e["hp_tau_s"] * (demand - self.Q)
        if self.mode == "heat":
            T_hi, T_lo = T_r + 10.0 + 273.15, T_out - 5.0 + 273.15
            cop = max(1.0, e["hp_carnot_eff"] * T_hi / max(T_hi - T_lo, 5.0))
        else:
            T_hi, T_lo = T_out + 10.0 + 273.15, T_r - 10.0 + 273.15
            cop = max(1.0, e["hp_carnot_eff"] * T_lo / max(T_hi - T_lo, 5.0))
        self.P = self.Q / cop + (e["hp_fan_W"] if self.on else 5.0)
        return self.Q

    def airflow(self):
        """실내기 팬 풍량 [m3/h] (운전 중 정격, 정지 시 0)."""
        return self.e["hp_airflow_m3h"] if self.on else 0.0


class ChamberPlant:
    def __init__(self, e, seed=0):
        self.e = e
        self.P = e["P_atm"]
        self.T = e["T0"]
        self.W = w_from_rh(e["T0"], e["RH0"], self.P)
        self.T_out, self.RH_out = 0.0, 50.0
        self.Q_cond = 0.0      # 챔버 공조 가열(+)/냉각(-) [W]
        self.m_hum = 0.0       # 가습(+)/제습(-) [kg/s]
        self.iT = 0.0          # PI 적분
        self.iW = 0.0
        self.hp = HeatPumpNative(e)
        self.rng = random.Random(seed)
        self.T_s, self.W_s, self.m_da = self.T, self.W, 0.0
        self.truth = {"Q_sens": 0.0, "Q_lat": 0.0, "Q_tot": 0.0, "m_w": 0.0}

    def step(self, dt, T_sp, RH_sp, T_out_sp, RH_out_sp):
        e, P = self.e, self.P
        W_sp = w_from_rh(T_sp, RH_sp, P)
        # --- PLC 국부 PI (챔버 공조) ---
        eT = T_sp - self.T
        self.iT = min(max(self.iT + e["pid_T_Kp"] / e["pid_T_Ti"] * eT * dt,
                          -e["Q_cond_max"]), e["Q_cond_max"])
        uT = min(max(e["pid_T_Kp"] * eT + self.iT, -e["Q_cond_max"]), e["Q_cond_max"])
        eW = (W_sp - self.W) * 1000.0                  # g/kg
        kW = e["pid_W_Kp"] / 1000.0                    # (kg/s) per (g/kg)
        self.iW = min(max(self.iW + kW / e["pid_W_Ti"] * eW * dt, -e["m_hum_max"]), e["m_hum_max"])
        uW = min(max(kW * eW + self.iW, -e["m_hum_max"]), e["m_hum_max"])
        self.Q_cond += dt / e["act_tau_s"] * (uT - self.Q_cond)
        self.m_hum += dt / e["act_tau_s"] * (uW - self.m_hum)
        # --- 실외측 챔버 ---
        self.T_out += dt / e["outdoor_tau_s"] * (T_out_sp - self.T_out)
        self.RH_out += dt / e["outdoor_tau_s"] * (RH_out_sp - self.RH_out)
        # --- 히트펌프 실내기: 리턴 = 챔버 공기 ---
        Q = self.hp.step(dt, self.T, self.T_out)
        V = self.hp.airflow()
        W_r = self.W
        m_da = V / 3600.0 / spec_volume(self.T, W_r, P) if V > 0 else 0.0   # 흡입측 기준 실제 유량
        T_s, W_s = coil_outlet(self.T, W_r, Q, m_da, self.hp.mode, e["hp_bypass_factor"], P)
        self.T_s, self.W_s, self.m_da = T_s, W_s, m_da
        h_s, h_r = enthalpy(T_s, W_s), enthalpy(self.T, W_r)
        self.truth = {"Q_sens": 1000.0 * m_da * cp_moist(W_r) * (T_s - self.T),
                      "Q_lat": 1000.0 * m_da * ((h_s - h_r) - cp_moist(W_r) * (T_s - self.T)),
                      "Q_tot": 1000.0 * m_da * (h_s - h_r), "m_w": m_da * (W_s - W_r)}
        # --- 챔버 공기 열/수분 수지 ---
        dT = (1000.0 * m_da * cp_moist(W_r) * (T_s - self.T) + self.Q_cond
              + e["UA_chamber_lab"] * (e["T_lab"] - self.T)) / e["C_chamber"]
        dW = (m_da * (W_s - self.W) + self.m_hum) / e["M_chamber_air"]
        self.T += dt * dT
        self.W = max(self.W + dt * dW, 1e-5)
        if e["ideal_chamber"]:                          # 기준해석: 챔버가 설정값을 즉시·완벽 추종
            self.T, self.W = T_sp, W_sp

    def measure(self):
        e, P, g = self.e, self.P, self.rng.gauss
        nT, nRH, nV = e["meas_noise_T"], e["meas_noise_RH"], e["meas_noise_V_pct"]
        n = lambda s: g(0, s) if s > 0 else 0.0          # noqa: E731
        V_nozzle = self.m_da * spec_volume(self.T_s, self.W_s, P) * 3600.0   # 노즐 = 토출측
        rh = lambda T, W: min(rh_from_w(T, W, P), 100.0)  # noqa: E731
        return {
            "T_supply": self.T_s + n(nT), "RH_supply": rh(self.T_s, self.W_s) + n(nRH),
            "T_return": self.T + n(nT), "RH_return": rh(self.T, self.W) + n(nRH),
            "V_air": V_nozzle * (1.0 + n(nV) / 100.0), "P_atm": P,
            "T_chamber": self.T + n(nT), "RH_chamber": rh(self.T, self.W) + n(nRH),
            "T_outdoor": self.T_out, "RH_outdoor": self.RH_out,
            "P_HP": self.hp.P, "HP_status": self.hp.status,
        }


class PLCEmulator:
    """PLC 스캔 로직. bank: 0-based offset -> uint16 (list 또는 RegisterBank)."""

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
        self.applied = {"T_room_SP": e["T0"], "RH_room_SP": e["RH0"], "T_outdoor_SP": 0.0,
                        "RH_outdoor_SP": 50.0, "Enable": 0, "Sequence": 0, "SIM_heartbeat": 0}
        self.hold = dict(self.applied)       # Enable=0 일 때 유지할 설정값
        self.hb = 0
        self.estop = False
        self.freeze_heartbeat = False
        self.wd_last, self.wd_age, self.watchdog = None, 0.0, False
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
            st |= 1 << bits["TRACKING"]
        if self.watchdog:
            st |= 1 << bits["WATCHDOG"]
        m.update({"AckSequence": self.applied["Sequence"], "PLC_status": st, "PLC_heartbeat": self.hb})
        for n, v in m.items():
            self.bank[self.rm.offset(n)], _ = self.rm.encode(n, v)

    def scan(self):
        cmd = self._read_commands()
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
        a = self.applied
        if a["Enable"] and not self.watchdog:
            self.hold = dict(a)              # 추종 중: 최신 설정값을 hold 값으로 갱신
        sp = self.hold
        for _ in range(int(round(self.Ts / self.dt))):
            self.plant.step(self.dt, sp["T_room_SP"], sp["RH_room_SP"], sp["T_outdoor_SP"],
                            sp["RH_outdoor_SP"])
        if not self.freeze_heartbeat:
            self.hb = (self.hb + 1) & 0xFFFF
        self.t += self.Ts
        self._write_outputs()

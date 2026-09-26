"""HILS supervisor (air-enthalpy 결합).

- on_building_step(t, zone_state) : 60 s 층. 가상 존의 다음 상태 -> 챔버 목표 설정값, Sequence 증가
- realization_step(t, meas)        : 1 s 층. 측정 -> air-enthalpy 열량(구간 평균 누적)
                                     -> heartbeat/지연/안전/상태머신 -> 설정값 변화율 제한 -> 명령
"""
from .airside import IntervalAverager, air_enthalpy
from .delay_monitor import DelayMonitor
from .registers import seq_next
from .safety import HeartbeatWatch, TrackingWatch, check_safety
from .setpoint import SetpointShaper
from .state_machine import RUN, StateMachine


class HilsSupervisor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.Ts = cfg["timing"]["Ts_realization"]
        s, z = cfg["safety"], cfg["zone"]
        self.sm = StateMachine(cfg["state_machine"])
        self.dm = DelayMonitor(s["ack_timeout_s"])
        self.hb = HeartbeatWatch(s["heartbeat_timeout_s"])
        self.tw = TrackingWatch(s["track_fault_T_K"], s["track_fault_hold_s"])
        self.shaper = SetpointShaper(cfg["setpoint"], z["T0"], z["RH0"])
        self.avg = IntervalAverager()
        self.seq = 0
        self.T_target, self.RH_target = z["T0"], z["RH0"]
        self.T_out_target, self.RH_out_target = 0.0, 50.0
        self._new_step = False
        self.sim_hb = 0
        self.last_air = None

    # ---- 60 s 층 ------------------------------------------------------------
    def interval_heat(self):
        """직전 건물 구간의 평균 측정 열량을 꺼내고 누적기를 비운다 (가상 존 입력)."""
        q = self.avg.mean()
        self.avg.reset()
        return q

    def on_building_step(self, t, zs):
        sm = self.cfg["state_machine"]
        self._new_step = (abs(zs["T_z"] - self.T_target) > sm["step_threshold_T_K"]
                          or abs(zs["RH_z"] - self.RH_target) > sm["step_threshold_RH_pct"])
        self.T_target, self.RH_target = zs["T_z"], zs["RH_z"]
        self.T_out_target, self.RH_out_target = zs["T_out"], zs["RH_out"]
        self.seq = seq_next(self.seq)
        self.dm.on_send(self.seq, t, (self.T_target, self.RH_target))

    # ---- 1 s 층 -------------------------------------------------------------
    def realization_step(self, t, m):
        c = self.cfg
        air = air_enthalpy(m["T_supply"], m["RH_supply"], m["T_return"], m["RH_return"],
                           m["V_air"], m["P_atm"])
        self.avg.add(air)
        self.last_air = air

        hb_lost = self.hb.step(m["PLC_heartbeat"], self.Ts)
        pending, ack_to = self.dm.step(m["AckSequence"], t)
        ack = self.dm.acked_target()                     # PLC 가 적용 중인 설정값 프레임
        T_ref, RH_ref = ack if ack is not None else (self.shaper.T, self.shaper.RH)
        eT, eRH = m["T_return"] - T_ref, m["RH_return"] - RH_ref
        sp = c["state_machine"]
        tracking_ok = abs(eT) < sp["track_tol_T_K"] and abs(eRH) < sp["track_tol_RH_pct"]
        track_lost = self.tw.step(abs(eT), self.sm.state == RUN, self.Ts)
        fault = check_safety(m, c["safety"], c["plc_status_bits"], c["hp_status_codes"],
                             hb_lost, ack_to, track_lost)
        plc_ready = bool(int(m["PLC_status"]) & (1 << c["plc_status_bits"]["READY"]))
        state = self.sm.step(self.Ts, plc_ready, fault != 0, tracking_ok, self._new_step)
        self._new_step = False

        T_sp, RH_sp = self.shaper.step(self.T_target, self.RH_target, self.Ts)
        self.sim_hb = (self.sim_hb + 1) & 0xFFFF
        cmd = {"T_room_SP": T_sp, "RH_room_SP": RH_sp, "T_outdoor_SP": self.T_out_target,
               "RH_outdoor_SP": self.RH_out_target, "Enable": 1 if self.sm.enable else 0,
               "Sequence": self.seq, "SIM_heartbeat": self.sim_hb}
        status = {"state": state, "fault": fault, "pending": pending, "ack_age": self.dm.age(t),
                  "latency": self.dm.last_latency, "valid": self.sm.valid, "eT": eT, "eRH": eRH,
                  "T_ref": T_ref, "RH_ref": RH_ref, **air}
        return cmd, status

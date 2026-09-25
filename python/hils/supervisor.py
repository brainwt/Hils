"""HILS supervisor: 두 개의 시간층을 한 객체에서 관리.

- on_building_step()  : Ts_supervisor(60 s) 마다 가상건물 목표(Q_target, T_target) 수신,
                         Sequence 증가, step 감지
- realization_step()  : Ts_realization(1 s) 마다 측정 -> 안전/지연/상태머신 -> PI -> 명령
"""

from .control import PIState, delay_aware_gains, pi_step
from .delay_monitor import DelayMonitor
from .registers import seq_next
from .safety import HeartbeatWatch, check_safety
from .state_machine import StateMachine


class HilsSupervisor:
    def __init__(self, cfg):
        self.cfg = cfg
        self.rc = cfg["realization_controller"]
        self.Ts = cfg["timing"]["Ts_realization"]
        self.sm = StateMachine(cfg["state_machine"])
        self.dm = DelayMonitor(cfg["safety"]["ack_timeout_s"])
        self.hb = HeartbeatWatch(cfg["safety"]["heartbeat_timeout_s"])
        self.pi = PIState()
        self.seq = 0
        self.Q_target = 0.0
        self.T_target = cfg["emulator"]["hp_setpoint"]
        self.T_out_target = 0.0
        self._new_step = False
        self.sim_hb = 0

    # ---- 60 s layer -------------------------------------------------------
    def on_building_step(self, t, Q_target, T_target, T_out):
        dQ = abs(Q_target - self.Q_target)
        self._new_step = dQ > self.cfg["state_machine"]["step_threshold_W"]
        self.Q_target, self.T_target, self.T_out_target = Q_target, T_target, T_out
        self.seq = seq_next(self.seq)
        self.dm.on_send(self.seq, t, Q_target)

    # ---- 1 s layer ----------------------------------------------------------
    def realization_step(self, t, meas):
        c = self.cfg
        hb_lost = self.hb.step(meas["PLC_heartbeat"], self.Ts)
        pending, ack_to = self.dm.step(meas["AckSequence"], t)
        fault = check_safety(meas, c["safety"], c["plc_status_bits"], c["hp_status_codes"],
                             hb_lost, ack_to)
        plc_ready = bool(int(meas["PLC_status"]) & (1 << c["plc_status_bits"]["READY"]))
        q_ack = self.dm.acked_target()          # PLC 가 실제 적용 중인 프레임의 목표
        q_ref = self.Q_target if q_ack is None else q_ack
        err_abs = abs(q_ref - meas["Q_load_meas"])
        state = self.sm.step(self.Ts, plc_ready, fault != 0, err_abs, self._new_step)
        self._new_step = False

        gains = delay_aware_gains(self.rc, self.dm.last_latency)
        u, self.pi, info = pi_step(self.pi, self.Q_target, meas["Q_load_meas"], self.rc, self.Ts,
                                   enable=self.sm.enable, hold=q_ack is None, q_ref=q_ref, gains=gains)
        self.sim_hb = (self.sim_hb + 1) & 0xFFFF
        cmd = {
            "Q_load_cmd": u,
            "T_chamber_SP": self.T_target,
            "Enable": 1 if self.sm.enable else 0,
            "Sequence": self.seq,
            "SIM_heartbeat": self.sim_hb,
            "T_outdoor_SP": self.T_out_target,
        }
        status = {
            "state": state, "fault": fault, "pending": pending,
            "ack_age": self.dm.age(t), "latency": self.dm.last_latency,
            "e": info["e"], "limited": info["limited"], "valid": self.sm.valid,
            "integ": self.pi.integ, "q_ref": q_ref, "Kp": gains[0], "Ki": gains[1],
        }
        return cmd, status



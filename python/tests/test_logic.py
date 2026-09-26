from hils.config import load_config
from hils.delay_monitor import DelayMonitor, seq_diff
from hils.safety import (FAULT_ACK_TIMEOUT, FAULT_AIRFLOW, FAULT_HEARTBEAT, FAULT_HP, FAULT_PLC,
                         FAULT_RH_RANGE, FAULT_T_RANGE, FAULT_TRACKING, HeartbeatWatch, TrackingWatch,
                         check_safety, describe)
from hils.setpoint import SetpointShaper
from hils.state_machine import RUN, SAFE_STOP, STABILIZING, STEP_CHANGE, WAIT_PLC, StateMachine

CFG = load_config()
OK = {"T_chamber": 22.0, "RH_chamber": 40.0, "HP_status": 1, "PLC_status": 1, "P_HP": 1500.0,
      "V_air": 890.0}


def _chk(m, **kw):
    return check_safety(m, CFG["safety"], CFG["plc_status_bits"], CFG["hp_status_codes"], **kw)


def test_seq_diff_modular():
    assert seq_diff(1, 65535) == 2 and seq_diff(65535, 1) == -2 and seq_diff(5, 5) == 0


def test_delay_monitor_multiple_outstanding_and_payload():
    dm = DelayMonitor(180)
    dm.on_send(1, 0.0, (20.0, 40.0))
    dm.on_send(2, 60.0, (20.3, 40.5))
    assert dm.step(0, 89.0) == (True, False)
    pending, _ = dm.step(1, 91.0)
    assert pending and dm.last_latency == 91.0 and dm.acked_target() == (20.0, 40.0)
    assert dm.step(2, 152.0) == (False, False) and dm.acked_target() == (20.3, 40.5)


def test_delay_monitor_skip_and_timeout():
    dm = DelayMonitor(120)
    for s, t in [(1, 0), (2, 60), (3, 120)]:
        dm.on_send(s, t, None)
    assert dm.step(0, 121.0) == (True, True)
    assert dm.step(3, 125.0) == (False, False)


def test_safety_bits():
    assert _chk(OK) == 0
    assert _chk(dict(OK, T_chamber=41.0)) == FAULT_T_RANGE
    assert _chk(dict(OK, RH_chamber=97.0)) == FAULT_RH_RANGE
    assert _chk(dict(OK, HP_status=9)) == FAULT_HP
    assert _chk(dict(OK, PLC_status=1 | 16)) == FAULT_PLC          # watchdog 비트
    assert _chk(dict(OK, V_air=20.0)) == FAULT_AIRFLOW               # 운전 중 풍량 없음
    assert _chk(dict(OK, V_air=0.0, HP_status=0)) == 0               # 정지 중 풍량 0 은 정상
    m = _chk(OK, hb_lost=True, ack_timeout=True, track_lost=True)
    assert m == FAULT_HEARTBEAT | FAULT_ACK_TIMEOUT | FAULT_TRACKING and len(describe(m)) == 3


def test_watchers():
    hb = HeartbeatWatch(10)
    assert not any(hb.step(7, 1.0) for _ in range(11)) and hb.step(7, 1.0) and not hb.step(8, 1.0)
    tw = TrackingWatch(3.0, 300)
    assert not any(tw.step(4.0, True, 1.0) for _ in range(300)) and tw.step(4.0, True, 1.0)
    assert not tw.step(4.0, False, 1.0)                              # RUN 이 아니면 감시 안 함


def test_setpoint_shaper_rate_and_limits():
    sh = SetpointShaper(CFG["setpoint"], 20.0, 40.0)
    T, RH = sh.step(25.0, 60.0, 1.0)
    assert T == 20.05 and RH == 40.2
    for _ in range(1000):
        T, RH = sh.step(50.0, 99.0, 1.0)
    assert T == CFG["setpoint"]["T_max"] and RH == CFG["setpoint"]["RH_max"]


def test_state_machine_sequence():
    p = CFG["state_machine"]
    sm = StateMachine(p)
    sm.step(1, True, False, False, False)
    assert sm.state == WAIT_PLC
    sm.step(1, True, False, False, False)
    assert sm.state == STABILIZING and sm.enable
    for _ in range(p["stabilize_hold_s"]):
        sm.step(1, True, False, True, False)
    assert sm.state == RUN and sm.valid
    sm.step(1, True, False, True, True)
    assert sm.state == STEP_CHANGE
    sm.step(1, True, False, True, False)
    assert sm.state == STABILIZING
    sm.step(1, True, True, True, False)
    assert sm.state == SAFE_STOP and not sm.enable
    for _ in range(p["fault_recover_hold_s"]):
        sm.step(1, True, False, True, False)
    assert sm.state == WAIT_PLC


def test_state_machine_timeouts():
    sm = StateMachine(dict(CFG["state_machine"], stabilize_timeout_s=50))
    sm.step(1, True, False, False, False)
    sm.step(1, True, False, False, False)
    for _ in range(60):
        sm.step(1, True, False, False, False)
    assert sm.state == SAFE_STOP and sm.warning == "stabilization timeout"
    sm = StateMachine(CFG["state_machine"])
    for _ in range(CFG["state_machine"]["wait_plc_timeout_s"] + 2):
        sm.step(1, False, False, False, False)
    assert sm.state == SAFE_STOP


def test_state_machine_latch():
    p = dict(CFG["state_machine"], max_auto_restarts=1, fault_recover_hold_s=2)
    sm = StateMachine(p)
    sm.step(1, True, False, False, False)
    sm.step(1, True, False, False, False)            # STABILIZING
    for _ in range(2):                                # 두 번 trip
        sm.step(1, True, True, False, False)
        assert sm.state == SAFE_STOP
        for _ in range(3):
            sm.step(1, True, False, False, False)
        sm.step(1, True, False, False, False)
    assert sm.latched and sm.state == SAFE_STOP
    for _ in range(10):
        sm.step(1, True, False, False, False)
    assert sm.state == SAFE_STOP                     # 잠김 유지
    sm.reset()
    for _ in range(3):
        sm.step(1, True, False, False, False)
    assert sm.state in (WAIT_PLC, STABILIZING)

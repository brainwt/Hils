from hils.config import load_config
from hils.delay_monitor import DelayMonitor, seq_diff
from hils.safety import (FAULT_ACK_TIMEOUT, FAULT_HEARTBEAT, FAULT_HP, FAULT_PLC, FAULT_T_RANGE,
                         HeartbeatWatch, check_safety, describe)
from hils.state_machine import RUN, SAFE_STOP, STABILIZING, STEP_CHANGE, WAIT_PLC, StateMachine

CFG = load_config()
OK_MEAS = {"T_indoor": 22.0, "RH_indoor": 40.0, "HP_status": 1, "PLC_status": 1, "P_HP": 1500.0}


def test_seq_diff_modular():
    assert seq_diff(1, 65535) == 2 and seq_diff(65535, 1) == -2 and seq_diff(5, 5) == 0


def test_delay_monitor_latency_and_target():
    dm = DelayMonitor(120)
    dm.on_send(125, 60.0, 3200.0)
    assert dm.step(124, 61.0) == (True, False)
    assert dm.step(125, 64.0) == (False, False)
    assert dm.last_latency == 4.0 and dm.acked_target() == 3200.0


def test_delay_monitor_multiple_outstanding():
    # 지연(90 s) > Ts_building(60 s): 두 시퀀스가 동시에 대기 -> 둘 다 추적해야 함
    dm = DelayMonitor(120)
    dm.on_send(1, 0.0, 1000.0)
    dm.on_send(2, 60.0, 2000.0)
    assert dm.step(0, 89.0) == (True, False)
    pending, _ = dm.step(1, 91.0)
    assert pending and dm.last_latency == 91.0 and dm.acked_target() == 1000.0
    dm.on_send(3, 120.0, 3000.0)
    pending, _ = dm.step(2, 151.0)
    assert pending and dm.last_latency == 91.0 and dm.acked_target() == 2000.0


def test_delay_monitor_skipped_ack_clears_older():
    dm = DelayMonitor(120)
    for s, t in [(1, 0), (2, 60), (3, 120)]:
        dm.on_send(s, t, s * 100.0)
    assert dm.step(3, 125.0) == (False, False)          # 1, 2 는 건너뛰었지만 모두 확인


def test_delay_monitor_timeout():
    dm = DelayMonitor(120)
    dm.on_send(1, 0.0, 0.0)
    dm.on_send(2, 60.0, 0.0)
    assert dm.step(0, 120.0) == (True, False)
    assert dm.step(0, 121.0) == (True, True)            # 가장 오래된 시퀀스 기준


def test_safety_bits():
    s, b, c = CFG["safety"], CFG["plc_status_bits"], CFG["hp_status_codes"]
    assert check_safety(OK_MEAS, s, b, c) == 0
    assert check_safety(dict(OK_MEAS, T_indoor=36.0), s, b, c) == FAULT_T_RANGE
    assert check_safety(dict(OK_MEAS, HP_status=9), s, b, c) == FAULT_HP
    assert check_safety(dict(OK_MEAS, PLC_status=1 | 4), s, b, c) == FAULT_PLC
    m = check_safety(OK_MEAS, s, b, c, hb_lost=True, ack_timeout=True)
    assert m == FAULT_HEARTBEAT | FAULT_ACK_TIMEOUT and len(describe(m)) == 2


def test_heartbeat_watch():
    hb = HeartbeatWatch(10)
    assert not any(hb.step(7, 1.0) for _ in range(11))
    assert hb.step(7, 1.0)
    assert not hb.step(8, 1.0)


def test_state_machine_sequence():
    p = CFG["state_machine"]
    sm = StateMachine(p)
    sm.step(1, True, False, 1000, False)
    assert sm.state == WAIT_PLC
    sm.step(1, True, False, 1000, False)
    assert sm.state == STABILIZING and sm.enable
    for _ in range(p["stabilize_hold_s"]):
        sm.step(1, True, False, 10, False)
    assert sm.state == RUN and sm.valid
    sm.step(1, True, False, 10, True)
    assert sm.state == STEP_CHANGE
    sm.step(1, True, False, 10, False)
    assert sm.state == STABILIZING and not sm.valid
    sm.step(1, True, True, 10, False)
    assert sm.state == SAFE_STOP and not sm.enable
    for _ in range(p["fault_recover_hold_s"]):
        sm.step(1, True, False, 10, False)
    assert sm.state == WAIT_PLC


def test_state_machine_stabilize_timeout():
    p = dict(CFG["state_machine"], stabilize_timeout_s=50)
    sm = StateMachine(p)
    sm.step(1, True, False, 0, False)
    sm.step(1, True, False, 0, False)
    for _ in range(60):
        sm.step(1, True, False, 5000, False)
    assert sm.state == SAFE_STOP and sm.warning == "stabilization timeout"


def test_state_machine_wait_plc_timeout():
    sm = StateMachine(CFG["state_machine"])
    for _ in range(CFG["state_machine"]["wait_plc_timeout_s"] + 2):
        sm.step(1, False, False, 0, False)
    assert sm.state == SAFE_STOP

"""폐루프 통합시험 (InMemory 전송, 결정적)."""
import numpy as np
import pytest

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import kpis, run_hils
from hils.emulator import PLCEmulator
from hils.safety import FAULT_HEARTBEAT, FAULT_PLC
from hils.state_machine import RUN, SAFE_STOP, WAIT_PLC
from hils.transport import InMemoryTransport

PROFILE = [(0, 2000), (1800, 3500), (3600, 1500), (5400, 3000)]


def _run(duration, overrides=None, building=None, events=None):
    cfg = load_config(overrides=overrides)
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    log, sup, bld = run_hils(cfg, tr, duration, plc=plc, building=building, events=events)
    return cfg, log, sup, bld, plc


def test_24h_virtual_building():
    cfg, log, sup, bld, _ = _run(24 * 3600)
    k = kpis(log, cfg)
    assert k["valid_ratio"] > 0.99
    assert k["rmse_valid_W"] < 50                      # 측정노이즈 30 W 수준
    assert abs(k["energy_error_pct"]) < 1.0
    assert 21.0 < k["T_indoor_mean"] < 23.0            # 히트펌프 native 제어가 설정온도 유지
    assert k["fault_samples"] == 0
    # 가상건물 에너지 수지: 히트펌프 공급열 ~ 재현부하
    assert k["energy_hp_kWh"] == pytest.approx(k["energy_realized_kWh"], rel=0.05)


def test_step_changes_detected():
    cfg, log, sup, *_ = _run(7200, building=StepProfileBuilding(PROFILE))
    st = np.array(log["state"])
    assert (st == 4).sum() == 3                         # 1800/3600/5400 s 세 번의 STEP_CHANGE
    e = np.array(log["Q_ref"]) - np.array(log["Q_load_meas"])
    assert np.abs(e[-300:]).max() < 150


@pytest.mark.parametrize("delay", [3, 30, 60, 90, 120])
def test_delay_robustness_with_compensation(delay):
    cfg, log, sup, *_ = _run(7200, {"emulator": {"ack_delay_s": delay}},
                             building=StepProfileBuilding(PROFILE))
    k = kpis(log, cfg)
    assert k["state_time_s"]["SAFE_STOP"] == 0
    assert k["ack_latency_max_s"] == pytest.approx(delay + 1)   # PLC 지연 + 1 스캔
    assert k["rmse_valid_W"] < 80 and k["valid_ratio"] > 0.75


def test_long_delay_without_compensation_degrades():
    cfg, log, *_ = _run(7200, {"emulator": {"ack_delay_s": 90},
                               "realization_controller": {"delay_compensation": False}},
                        building=StepProfileBuilding(PROFILE))
    assert kpis(log, cfg)["state_time_s"]["SAFE_STOP"] > 0      # 진동 -> 안정화 실패


def test_heartbeat_loss_trips_and_recovers():
    def freeze(plc, tr, sup):
        plc.freeze_heartbeat = True

    def unfreeze(plc, tr, sup):
        plc.freeze_heartbeat = False

    cfg, log, *_ = _run(1800, building=StepProfileBuilding([(0, 2500)]),
                        events={600: freeze, 700: unfreeze})
    st, f = np.array(log["state"]), np.array(log["fault"])
    t = np.array(log["t"])
    assert (f & FAULT_HEARTBEAT).any()
    trip = t[np.argmax(st == SAFE_STOP)]
    assert 610 <= trip <= 613                            # timeout 10 s 후 trip
    assert (st[t > 700] == WAIT_PLC).any() and st[-1] == RUN   # 해제 + hold 후 재기동
    q = np.array(log["Q_load_cmd"])
    assert abs(q[(t > trip + 60) & (t < 700)]).max() < 1e-9   # SAFE_STOP 중 명령 0 으로 ramp-down


def test_estop_trips():
    def estop(plc, tr, sup):
        plc.estop = True

    cfg, log, *_ = _run(900, building=StepProfileBuilding([(0, 2500)]), events={400: estop})
    st, f = np.array(log["state"]), np.array(log["fault"])
    assert st[-1] == SAFE_STOP and (f & FAULT_PLC).any()


def test_command_loss_triggers_ack_timeout():
    def drop(plc, tr, sup):
        tr.drop_writes = True                           # Simulink -> PLC 쓰기 두절

    cfg, log, *_ = _run(1200, building=StepProfileBuilding([(0, 2500)]), events={300: drop})
    st, f, t = np.array(log["state"]), np.array(log["fault"]), np.array(log["t"])
    # PLC watchdog(10 s) 이 먼저 감지 -> WATCHDOG 비트로 즉시 trip, 이후 ack timeout 도 누적
    first = t[np.argmax(f & FAULT_PLC > 0)]
    assert 310 <= first <= 314 and st[t > first][0] == SAFE_STOP
    assert (f & 32).any()                               # FAULT_ACK_TIMEOUT


def test_plc_watchdog_releases_load_on_write_loss():
    def drop(plc, tr, sup):
        tr.drop_writes = True

    cfg, log, sup, bld, plc = _run(600, building=StepProfileBuilding([(0, 2500)]), events={300: drop})
    assert plc.watchdog                                   # PLC 가 스스로 감지
    assert not plc.bank[plc.rm.offset("PLC_status")] & (1 << cfg["plc_status_bits"]["LOAD_ACTIVE"])
    assert abs(plc.plant.Q_load) < 800                    # 부하모드 해제 -> 사전조화 모드로 전환

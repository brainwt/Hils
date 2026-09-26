"""폐루프 통합시험 (InMemory 전송, 결정적)."""
import numpy as np
import pytest

from hils.config import _merge, load_config
from hils.cosim import kpis, make_zone, run_hils, season_overrides
from hils.emulator import PLCEmulator
from hils.safety import FAULT_ACK_TIMEOUT, FAULT_HEARTBEAT, FAULT_PLC, FAULT_TRACKING
from hils.state_machine import RUN, SAFE_STOP, WAIT_PLC
from hils.transport import InMemoryTransport
from hils.zone import ProfileZone

IDEAL = {"ideal_chamber": True, "ack_delay_s": 0, "meas_noise_T": 0, "meas_noise_RH": 0,
         "meas_noise_V_pct": 0}


def _run(season, duration, emu=None, zone=None, events=None, extra=None):
    o = season_overrides(season)
    _merge(o, {"emulator": emu or {}})
    if extra:
        _merge(o, extra)
    cfg = load_config(overrides=o)
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    log, sup, z = run_hils(cfg, tr, duration, plc=plc, zone=zone or make_zone(cfg, season),
                           events=events)
    return cfg, log, sup, plc


def test_air_enthalpy_measurement_matches_plant_truth():
    """센서/레지스터 양자화를 거친 air-enthalpy 산정값 vs 에뮬레이터 내부 참값."""
    cfg, log, sup, plc = _run("summer", 3 * 3600, emu={"meas_noise_T": 0, "meas_noise_RH": 0,
                                                        "meas_noise_V_pct": 0})
    tr, a = plc.plant.truth, sup.last_air
    assert a["Q_tot"] == pytest.approx(tr["Q_tot"], rel=0.02, abs=30)
    assert a["Q_sens"] == pytest.approx(tr["Q_sens"], rel=0.02, abs=30)


@pytest.mark.parametrize("season", ["winter", "summer"])
def test_24h_season(season):
    cfg, log, sup, _ = _run(season, 24 * 3600)
    k = kpis(log, cfg)
    assert k["valid_ratio"] > 0.98 and k["fault_samples"] == 0
    assert k["track_rmse_T_K"] < 0.1 and k["track_rmse_RH_pct"] < 1.0   # 챔버가 가상 존 상태를 재현
    if season == "winter":
        assert 21.5 < k["T_z_mean"] < 22.5 and k["heat_kWh"] > 50 and k["cool_kWh"] == 0
        assert 2.5 < k["COP_or_EER"] < 5
    else:
        assert 24.5 < k["T_z_mean"] < 25.5 and k["cool_kWh"] > 30 and k["heat_kWh"] == 0
        assert 0.6 < k["SHR"] < 0.95                                      # 습코일 제습(잠열) 발생
        assert min(log["Q_lat"]) < -500


@pytest.mark.parametrize("season", ["winter", "summer"])
def test_fidelity_vs_ideal_coupling(season):
    """실제 챔버 동특성·노이즈·3 s 지연을 넣어도 존 궤적이 이상적 결합과 거의 같다."""
    cfg, ref, *_ = _run(season, 4 * 3600, emu=IDEAL)
    cfg, log, *_ = _run(season, 4 * 3600)
    e = np.array(log["T_z"]) - np.array(ref["T_z"])
    assert np.sqrt(np.mean(e ** 2)) < 0.1
    kr, k = kpis(ref, cfg), kpis(log, cfg)
    Er, E = kr["heat_kWh"] + kr["cool_kWh"], k["heat_kWh"] + k["cool_kWh"]
    assert abs(E - Er) / Er < 0.02


@pytest.mark.parametrize("delay", [30, 90, 120])
def test_delay_robustness(delay):
    cfg, log, sup, _ = _run("summer", 3 * 3600, emu={"ack_delay_s": delay})
    k = kpis(log, cfg)
    assert k["state_time_s"]["SAFE_STOP"] == 0
    assert k["ack_latency_max_s"] == pytest.approx(delay + 1)
    assert k["track_rmse_T_K"] < 0.1


def test_setpoint_step_detected():
    z = ProfileZone([(0, 20.0, 40.0), (1800, 23.0, 45.0)])
    cfg, log, *_ = _run("winter", 3600, zone=z)
    st = np.array(log["state"])
    assert (st == 4).sum() == 1 and st[-1] == RUN
    T = np.array(log["T_sp"])
    assert np.max(np.diff(T)) <= cfg["setpoint"]["T_rate_K_per_s"] + 1e-9   # 변화율 제한


def test_heartbeat_loss_trips_and_recovers():
    ev = {600: lambda p, t, s: setattr(p, "freeze_heartbeat", True),
          700: lambda p, t, s: setattr(p, "freeze_heartbeat", False)}
    cfg, log, *_ = _run("winter", 1800, events=ev)
    st, f, t = (np.array(log[k]) for k in ("state", "fault", "t"))
    assert (f & FAULT_HEARTBEAT).any()
    trip = t[np.argmax(st == SAFE_STOP)]
    assert 610 <= trip <= 613
    assert (st[t > 700] == WAIT_PLC).any() and st[-1] == RUN


def test_estop_trips():
    cfg, log, *_ = _run("winter", 900, events={400: lambda p, t, s: setattr(p, "estop", True)})
    assert log["state"][-1] == SAFE_STOP and (np.array(log["fault"]) & FAULT_PLC).any()


def test_write_loss_plc_watchdog_holds_chamber():
    def drop(plc, tr, sup):
        tr.drop_writes = True

    cfg, log, sup, plc = _run("winter", 1200, events={300: drop})
    f, t, st = (np.array(log[k]) for k in ("fault", "t", "state"))
    first = t[np.argmax(f & FAULT_PLC > 0)]
    assert 310 <= first <= 314 and st[t > first][0] == SAFE_STOP
    assert plc.watchdog
    assert not plc.bank[plc.rm.offset("PLC_status")] & (1 << cfg["plc_status_bits"]["TRACKING"])
    assert (f & FAULT_ACK_TIMEOUT).any()


def test_chamber_capacity_shortage_latches_after_restarts():
    """챔버 공조 용량 부족 -> 안정화 실패 반복 -> 자동 재시작 3회 후 SAFE_STOP 잠김."""
    cfg, log, sup, _ = _run("winter", 4 * 3600, emu={"Q_cond_max": 1500.0})
    st = np.array(log["state"])
    assert (st == RUN).sum() == 0
    assert sup.sm.latched and st[-1] == SAFE_STOP
    assert st[0] == WAIT_PLC                                          # t=0 에 이미 대기 상태
    assert int((np.diff(st) != 0)[st[1:] == WAIT_PLC].sum()) == 3     # 자동 재시작 3회
    sup.sm.reset()
    assert not sup.sm.latched


def test_tracking_loss_in_run_trips():
    """RUN 도중 챔버 용량 저하 -> 추종오차 3 K 초과 300 s -> FAULT_TRACKING."""
    def degrade(plc, tr, sup):
        plc.plant.e["Q_cond_max"] = 600.0

    cfg, log, *_ = _run("winter", 5400, events={1200: degrade})
    f, t = np.array(log["fault"]), np.array(log["t"])
    assert (f & FAULT_TRACKING).any() and t[np.argmax(f & FAULT_TRACKING > 0)] > 1500

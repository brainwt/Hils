import pytest

from hils.config import load_config
from hils.cosim import make_zone, season_overrides


def test_zone_is_passive_without_hp():
    """HP 열량 0 이면 존 온도는 외기 쪽으로 움직인다 (겨울: 하강)."""
    cfg = load_config(overrides=season_overrides("winter"))
    z = make_zone(cfg, "winter")
    T0 = z.state()["T_z"]
    for _ in range(60):
        s = z.step(0.0, 0.0)
    assert s["T_z"] < T0 and s["t"] == 3600


def test_zone_heat_input_raises_temperature_and_moisture():
    cfg = load_config(overrides=season_overrides("winter"))
    a, b = make_zone(cfg, "winter"), make_zone(cfg, "winter")
    for _ in range(30):
        sa = a.step(0.0, 0.0)
        sb = b.step(5000.0, 2e-4)
    assert sb["T_z"] > sa["T_z"] + 1.0
    assert sb["W_z"] > sa["W_z"]


def test_zone_steady_state_heat_balance():
    """외기 0 degC, 일사·내부발열 0, 가열 3 kW -> 정상상태에서 입력열 = 창+외피+침기 손실."""
    from hils.psychro import cp_moist
    from hils.zone import VirtualZone
    cfg = load_config()
    zp = dict(cfg["zone"], C_mass=3e5, Q_int_occupied=0.0, Q_int_unoccupied=0.0,
              g_int_occupied=0.0, g_int_unoccupied=0.0)
    z = VirtualZone(zp, lambda t: (0.0, 70.0, 0.0), 60, 10)
    for _ in range(3000):
        s = z.step(3000.0, 0.0)
    loss = (s["T_z"] / zp["R_win"] + s["Tm"] / zp["R_ext"]
            + z.m_inf * 1000 * cp_moist(s["W_z"]) * s["T_z"])
    assert loss == pytest.approx(3000.0, rel=1e-3)

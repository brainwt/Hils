import pytest

from hils.airside import IntervalAverager, air_enthalpy
from hils.psychro import (dew_point, enthalpy, p_ws, rh_from_w, spec_volume, t_from_h_w,
                          t_sat_from_h, w_from_rh)

P = 101.325


@pytest.mark.parametrize("t,expected_kpa", [(-10, 0.25990), (0, 0.61121), (20, 2.3393), (35, 5.6290)])
def test_saturation_pressure_ashrae_table(t, expected_kpa):
    # ASHRAE Fundamentals 2017 Ch.1 Table 3
    assert p_ws(t) == pytest.approx(expected_kpa, rel=2e-3)


def test_moist_air_properties_ashrae_example():
    W = w_from_rh(25.0, 50.0, P)
    assert W == pytest.approx(0.00988, rel=5e-3)
    assert enthalpy(25.0, W) == pytest.approx(50.3, abs=0.2)
    assert spec_volume(25.0, W, P) == pytest.approx(0.858, abs=0.002)
    assert dew_point(W, P) == pytest.approx(13.86, abs=0.05)


def test_inverse_functions():
    W = w_from_rh(18.3, 63.0, P)
    assert rh_from_w(18.3, W, P) == pytest.approx(63.0, rel=1e-9)
    assert t_from_h_w(enthalpy(18.3, W), W) == pytest.approx(18.3, abs=1e-9)
    h = enthalpy(12.0, w_from_rh(12.0, 100.0, P))
    assert t_sat_from_h(h, P) == pytest.approx(12.0, abs=1e-6)


def test_air_enthalpy_heating_sensible_only():
    # 난방: 절대습도 불변 -> 잠열 0, 현열 = m cp dT
    P_ = P
    W = w_from_rh(20.0, 40.0, P_)
    a = air_enthalpy(35.0, rh_from_w(35.0, W, P_), 20.0, 40.0, 900.0, P_)
    assert a["Q_lat"] == pytest.approx(0.0, abs=1e-6)
    m = 900 / 3600 / spec_volume(35.0, W, P_)
    assert a["Q_sens"] == pytest.approx(1000 * m * (1.006 + 1.86 * W) * 15.0, rel=1e-9)
    assert a["Q_tot"] == pytest.approx(a["Q_sens"], rel=1e-9)


def test_air_enthalpy_cooling_signs():
    a = air_enthalpy(13.0, 95.0, 27.0, 50.0, 900.0, P)
    assert a["Q_sens"] < 0 and a["Q_lat"] < 0 and a["m_w"] < 0
    assert a["Q_tot"] == pytest.approx(a["Q_sens"] + a["Q_lat"], rel=1e-12)


def test_interval_averager():
    av = IntervalAverager()
    for q in (100.0, 200.0, 300.0):
        av.add({"Q_sens": q, "Q_lat": -q, "Q_tot": 0.0, "m_w": 1e-4})
    m = av.mean()
    assert m["Q_sens"] == 200.0 and m["Q_lat"] == -200.0 and m["n"] == 3

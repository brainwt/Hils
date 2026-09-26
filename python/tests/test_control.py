import math

import pytest

from hils.config import load_config
from hils.control import PIState, delay_aware_gains, pi_step

P = load_config()["realization_controller"]


def test_rate_limit_applies():
    u, s, _ = pi_step(PIState(), 5000, 0, P, 1.0)
    assert u == pytest.approx(P["rate_limit_W_per_s"])     # 한 스텝에 50 W 만 증가


def test_saturation_and_antiwindup():
    s = PIState()
    for _ in range(2000):
        u, s, info = pi_step(s, 20000, 0, P, 1.0)          # 도달 불가능한 목표
    assert u == P["Q_cmd_max"]
    integ_sat = s.integ
    for _ in range(100):
        u, s, _ = pi_step(s, 20000, 0, P, 1.0)
    assert s.integ == integ_sat                             # 포화 중 적분 정지(와인드업 없음)


def test_hold_freezes_integrator():
    s = PIState(integ=100.0, u_prev=1000.0)
    _, s2, _ = pi_step(s, 1000, 900, P, 1.0, hold=True)
    assert s2.integ == 100.0


def test_q_ref_used_for_error_and_target_for_feedforward():
    s = PIState(u_prev=3000.0)
    _, _, info = pi_step(s, 3000, 1900, P, 1.0, q_ref=2000)
    assert info["e"] == pytest.approx(100)


def test_disable_ramps_to_zero_and_resets():
    s = PIState(integ=500.0, u_prev=120.0)
    u, s, _ = pi_step(s, 3000, 0, P, 1.0, enable=False)
    assert u == pytest.approx(70.0) and s.integ == 0.0


def test_steady_state_error_removed_on_biased_plant():
    # 1차 플랜트 (이득 0.93, 바이어스 -150 W, tau 20 s) -> 정상상태 오차 0
    s, y = PIState(), 0.0
    for _ in range(2000):
        u, s, _ = pi_step(s, 3000, y, P, 1.0)
        y += (0.93 * u - 150 - y) / 20.0
    assert abs(3000 - y) < 1.0


def test_delay_aware_gains():
    assert delay_aware_gains(P, math.nan) == (P["Kp"], P["Ki"])
    assert delay_aware_gains(P, 4.0) == (P["Kp"], P["Ki"])          # 짧은 지연: 설정 게인
    kp, ki = delay_aware_gains(P, 90.0)
    assert kp == pytest.approx(20 / 180) and ki == pytest.approx(kp / 20)
    assert delay_aware_gains(dict(P, delay_compensation=False), 90.0) == (P["Kp"], P["Ki"])

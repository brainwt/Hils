"""Realization load controller: feedforward + PI -> rate limiter -> saturation.

u_raw = ff * Q_target + Kp*e + I
u     = sat(ratelimit(u_raw))
Anti-windup: 조건부 적분 — 출력이 제한되고 오차가 같은 방향이면 적분하지 않는다.
hold=True 이면 적분 정지(아직 어떤 프레임도 ack 되지 않아 기준을 모를 때).
"""
import math
from dataclasses import dataclass


@dataclass
class PIState:
    integ: float = 0.0
    u_prev: float = 0.0


def rate_limit(u, u_prev, max_delta):
    return min(max(u, u_prev - max_delta), u_prev + max_delta)


def saturate(u, lo, hi):
    return min(max(u, lo), hi)


def pi_step(state, q_target, q_meas, p, Ts, enable=True, hold=False, q_ref=None, gains=None):
    """Returns (u, new_state, info). p: realization_controller dict.

    q_target : 피드포워드용 최신 목표 (즉시 송신)
    q_ref    : 오차 계산용 기준 = PLC 가 실제 적용 중인(ack 된) 프레임의 목표. None 이면 q_target
    gains    : (Kp, Ki) 재정의 (지연보상 스케줄링). None 이면 설정값
    """
    if not enable:
        # 비활성: 적분 리셋, 출력은 0 방향으로 rate-limit 하며 복귀
        u = saturate(rate_limit(0.0, state.u_prev, p["rate_limit_W_per_s"] * Ts),
                     p["Q_cmd_min"], p["Q_cmd_max"])
        return u, PIState(0.0, u), {"e": 0.0, "limited": False}

    e = (q_target if q_ref is None else q_ref) - q_meas
    kp, ki = gains if gains else (p["Kp"], p["Ki"])
    integ_new = state.integ if hold else state.integ + ki * e * Ts
    u_raw = p["feedforward_gain"] * q_target + kp * e + integ_new
    u_rl = rate_limit(u_raw, state.u_prev, p["rate_limit_W_per_s"] * Ts)
    u = saturate(u_rl, p["Q_cmd_min"], p["Q_cmd_max"])
    limited = u != u_raw
    if limited and (u_raw - u) * e > 0:
        integ_new = state.integ  # conditional integration (anti-windup)
    return u, PIState(integ_new, u), {"e": e, "limited": limited}


def delay_aware_gains(p, theta):
    """SIMC 튜닝으로 측정 루프지연 theta[s] 에 맞춰 PI 게인을 낮춘다(설정 게인이 상한).

    공칭 모델: G = K e^(-theta s) / (tau s + 1),  tau_c = theta
      Kp = tau / (K (tau_c + theta)),  Ti = min(tau, 4 (tau_c + theta))
    """
    if not p.get("delay_compensation", False) or not (theta > 0) or math.isnan(theta):
        return p["Kp"], p["Ki"]
    tau, K = p["plant_tau_s"], p["plant_gain"]
    kp = min(p["Kp"], tau / (K * 2.0 * theta))
    ti = min(tau, 8.0 * theta)
    return kp, min(p["Ki"], kp / ti)

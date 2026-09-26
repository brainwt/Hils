"""습공기 물성 (ASHRAE Fundamentals 2017, Hyland-Wexler 포화수증기압).

단위: t [degC], RH [%], P [kPa], W [kg/kg_da], h [kJ/kg_da], v [m3/kg_da]
MATLAB matlab/lib/hils_psy_*.m 과 같은 식/같은 반복횟수로 구현 (교차검증 대상).
"""
import math

MW_RATIO = 0.621945          # Mw / Mda
R_DA = 0.287042              # kJ/(kg K)
CP_DA, CP_V, H_FG0 = 1.006, 1.86, 2501.0


def p_ws(t):
    """포화수증기압 [kPa]. t >= 0 물, t < 0 얼음."""
    T = t + 273.15
    if t >= 0.0:
        ln = (-5.8002206e3 / T + 1.3914993 - 4.8640239e-2 * T + 4.1764768e-5 * T ** 2
              - 1.4452093e-8 * T ** 3 + 6.5459673 * math.log(T))
    else:
        ln = (-5.6745359e3 / T + 6.3925247 - 9.677843e-3 * T + 6.2215701e-7 * T ** 2
              + 2.0747825e-9 * T ** 3 - 9.484024e-13 * T ** 4 + 4.1635019 * math.log(T))
    return math.exp(ln) / 1000.0


def w_from_rh(t, rh, P):
    pw = rh / 100.0 * p_ws(t)
    return MW_RATIO * pw / (P - pw)


def rh_from_w(t, W, P):
    pw = P * W / (MW_RATIO + W)
    return 100.0 * pw / p_ws(t)


def enthalpy(t, W):
    return CP_DA * t + W * (H_FG0 + CP_V * t)


def t_from_h_w(h, W):
    return (h - H_FG0 * W) / (CP_DA + CP_V * W)


def spec_volume(t, W, P):
    """습공기 비체적 [m3 / kg 건공기]."""
    return R_DA * (t + 273.15) * (1.0 + 1.607858 * W) / P


def cp_moist(W):
    return CP_DA + CP_V * W


def t_sat_from_h(h, P, lo=-40.0, hi=60.0, n=60):
    """포화 엔탈피 h 를 갖는 온도 (이분법, 고정 반복횟수 -> 구현 간 결과 일치)."""
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if enthalpy(mid, w_from_rh(mid, 100.0, P)) > h:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)


def dew_point(W, P, lo=-40.0, hi=60.0, n=60):
    for _ in range(n):
        mid = 0.5 * (lo + hi)
        if w_from_rh(mid, 100.0, P) > W:
            hi = mid
        else:
            lo = mid
    return 0.5 * (lo + hi)

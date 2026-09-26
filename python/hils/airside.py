"""Air-enthalpy 법 열량 산정 (ASHRAE 37 / KS C 9306 방식).

입력 : 실내기 토출(T_s, RH_s)·리턴(T_r, RH_r), 노즐 체적풍량 V [m3/h], 대기압 P [kPa]
       노즐은 실내기 토출측에 있으므로 토출 공기 비체적으로 건공기 질량유량 환산.
출력 : 존에 공급된 열량 (난방 +, 냉방 -)
  m_da   = V / 3600 / v_s                      [kg_da/s]
  Q_sens = m_da * cp(W_r) * (T_s - T_r) * 1000  [W]    현열
  Q_tot  = m_da * (h_s - h_r) * 1000            [W]    전열
  Q_lat  = Q_tot - Q_sens                       [W]    잠열 (제습 시 -, ASHRAE 37 관례)
  m_w    = m_da * (W_s - W_r)                   [kg/s] 존에 공급된 수분
"""
from .psychro import cp_moist, enthalpy, spec_volume, w_from_rh


def air_enthalpy(T_s, RH_s, T_r, RH_r, V_m3h, P):
    W_s, W_r = w_from_rh(T_s, RH_s, P), w_from_rh(T_r, RH_r, P)
    v_s = spec_volume(T_s, W_s, P)
    m_da = max(V_m3h, 0.0) / 3600.0 / v_s
    h_s, h_r = enthalpy(T_s, W_s), enthalpy(T_r, W_r)
    return {
        "m_da": m_da, "W_s": W_s, "W_r": W_r, "h_s": h_s, "h_r": h_r,
        "Q_sens": 1000.0 * m_da * cp_moist(W_r) * (T_s - T_r),
        "Q_lat": 1000.0 * m_da * ((h_s - h_r) - cp_moist(W_r) * (T_s - T_r)),
        "Q_tot": 1000.0 * m_da * (h_s - h_r),
        "m_w": m_da * (W_s - W_r),
    }


class IntervalAverager:
    """건물 timestep 동안 1 s 측정 열량을 평균 (가상 존 입력)."""

    def __init__(self):
        self.reset()

    def reset(self):
        self.n, self.q_sens, self.q_lat, self.m_w, self.q_tot = 0, 0.0, 0.0, 0.0, 0.0

    def add(self, a):
        self.n += 1
        self.q_sens += a["Q_sens"]
        self.q_lat += a["Q_lat"]
        self.q_tot += a["Q_tot"]
        self.m_w += a["m_w"]

    def mean(self):
        n = max(self.n, 1)
        return {"Q_sens": self.q_sens / n, "Q_lat": self.q_lat / n, "Q_tot": self.q_tot / n,
                "m_w": self.m_w / n, "n": self.n}

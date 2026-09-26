"""가상 존 모델 (EnergyPlus/FMU 자리). 존 공기 상태가 이 모델의 상태변수다.

입력(구간 k 평균): air-enthalpy 로 측정한 히트펌프 공급 현열 Q_sens [W], 수분 m_w [kg/s]
출력(k+1)       : 존 공기 T_z, RH_z (-> 실내측 챔버 설정값)
  C_z dTz/dt = (Tm-Tz)/R_int + (To-Tz)/R_win + m_inf cp (To-Tz) + f I A + Q_int + Q_sens
  C_m dTm/dt = (Tz-Tm)/R_int + (To-Tm)/R_ext + (1-f) I A
  M_z dWz/dt = m_inf (Wo - Wz) + g_int + m_w
Ts_building 안을 Ts_zone_sub 로 나눠 explicit Euler 적분.
"""
from .psychro import cp_moist, rh_from_w, spec_volume, w_from_rh
from .weather import occupied


class VirtualZone:
    def __init__(self, p, weather, Ts, Ts_sub, P=101.325):
        self.p, self.weather, self.Ts, self.Ts_sub, self.P = p, weather, Ts, Ts_sub, P
        self.Tz, self.Tm = p["T0"], p["Tm0"]
        self.Wz = w_from_rh(p["T0"], p["RH0"], P)
        rho = 1.0 / spec_volume(p["T0"], self.Wz, P)
        self.m_air = rho * p["V_zone_m3"]
        self.Cz = self.m_air * 1006.0 * p["C_air_mult"]
        self.Mz = self.m_air * p["moist_mult"]
        self.m_inf = p["ACH_inf"] * self.m_air / 3600.0
        self.t = 0.0

    def state(self):
        T_out, RH_out, I_sol = self.weather(self.t)
        return {"T_z": self.Tz, "RH_z": rh_from_w(self.Tz, self.Wz, self.P), "W_z": self.Wz,
                "Tm": self.Tm, "T_out": T_out, "RH_out": RH_out, "I_sol": I_sol, "t": self.t}

    def step(self, Q_sens, m_w):
        """구간 [t, t+Ts] 를 적분하고 t+Ts 의 존 상태를 반환."""
        p = self.p
        n = int(round(self.Ts / self.Ts_sub))
        for _ in range(n):
            T_out, RH_out, I_sol = self.weather(self.t)
            W_out = w_from_rh(T_out, RH_out, self.P)
            occ = occupied(self.t)
            Q_int = p["Q_int_occupied"] if occ else p["Q_int_unoccupied"]
            g_int = p["g_int_occupied"] if occ else p["g_int_unoccupied"]
            q_mass = (self.Tm - self.Tz) / p["R_int"]
            dTz = (q_mass + (T_out - self.Tz) / p["R_win"]
                   + self.m_inf * 1000.0 * cp_moist(self.Wz) * (T_out - self.Tz)
                   + p["f_sol_air"] * p["A_sol"] * I_sol + Q_int + Q_sens) / self.Cz
            dTm = (-q_mass + (T_out - self.Tm) / p["R_ext"]
                   + (1.0 - p["f_sol_air"]) * p["A_sol"] * I_sol) / p["C_mass"]
            dWz = (self.m_inf * (W_out - self.Wz) + g_int + m_w) / self.Mz
            self.Tz += self.Ts_sub * dTz
            self.Tm += self.Ts_sub * dTm
            self.Wz = max(self.Wz + self.Ts_sub * dWz, 1e-5)
            self.t += self.Ts_sub
        return self.state()


class ProfileZone:
    """시험용: 존 상태를 계단 프로파일로 지정 [(t, T, RH), ...] (측정 열량 무시)."""

    def __init__(self, profile, T_out=0.0, RH_out=70.0):
        self.profile = sorted(profile)
        self.T_out, self.RH_out = T_out, RH_out
        self.t = 0.0

    def state(self):
        T, RH = self.profile[0][1], self.profile[0][2]
        for ts, tt, rr in self.profile:
            if self.t >= ts:
                T, RH = tt, rr
        return {"T_z": T, "RH_z": RH, "W_z": float("nan"), "Tm": float("nan"),
                "T_out": self.T_out, "RH_out": self.RH_out, "I_sol": 0.0, "t": self.t}

    def step(self, Q_sens, m_w, Ts=60.0):
        self.t += Ts
        return self.state()

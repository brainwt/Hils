"""가상건물 에뮬레이터 (EnergyPlus/FMU 자리를 대신하는 2R2C 모델).

존 공기 노드 = 실제 챔버(측정 T_indoor). 가상 부분은 외피/구조체 질량 노드 Tm.
  C_m dTm/dt = (Tz - Tm)/R_int + (T_out - Tm)/R_ext
  Q_target   = (Tz - Tm)/R_int + (Tz - T_out)/R_win - A_sol*I_sol - Q_int
Q_target > 0 : 존에서 열을 빼야 함(난방부하) -> 챔버 부하장치가 냉각으로 재현.
Ts_building 마다 step() 한 번 호출 (EnergyPlus timestep 과 동일한 역할).
"""
from .weather import occupied, winter_day


class VirtualBuilding2R2C:
    def __init__(self, p, Ts, Tm0=18.0, weather=winter_day, T_set=22.0):
        self.p, self.Ts, self.Tm = p, Ts, Tm0
        self.weather = weather
        self.T_set = T_set
        self.k = 0
        self.E_target = 0.0   # 누적 목표부하 [J]
        self.E_hp = 0.0       # 누적 히트펌프 공급열 [J]

    def step(self, t, T_zone_meas, Q_hp_meas):
        p = self.p
        T_out, I_sol = self.weather(t)
        Q_int = p["Q_int_occupied"] if occupied(t) else p["Q_int_unoccupied"]
        Tz = T_zone_meas
        q_mass = (Tz - self.Tm) / p["R_int"]
        Q_target = q_mass + (Tz - T_out) / p["R_win"] - p["A_sol"] * I_sol - Q_int
        # 구조체 노드 적분 (explicit Euler, 시정수 >> Ts 이므로 안정)
        self.Tm += self.Ts * (q_mass + (T_out - self.Tm) / p["R_ext"]) / p["C_mass"]
        self.E_target += Q_target * self.Ts
        self.E_hp += Q_hp_meas * self.Ts
        self.k += 1
        return {"Q_target": Q_target, "T_target": self.T_set, "T_out": T_out,
                "I_sol": I_sol, "Q_int": Q_int, "Tm": self.Tm}


class StepProfileBuilding:
    """시험용 부하 프로파일: [(t_start, Q_target), ...] 계단 입력. 가상건물 대신 사용."""

    def __init__(self, profile, T_set=22.0, T_out=0.0):
        self.profile = sorted(profile)
        self.T_set, self.T_out = T_set, T_out

    def step(self, t, T_zone_meas, Q_hp_meas):
        Q = 0.0
        for ts, q in self.profile:
            if t >= ts:
                Q = q
        return {"Q_target": Q, "T_target": self.T_set, "T_out": self.T_out,
                "I_sol": 0.0, "Q_int": 0.0, "Tm": float("nan")}

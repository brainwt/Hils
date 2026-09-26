"""가상 존 상태 -> 챔버 설정값: 범위 제한 + 1 s 변화율 제한."""


def rate_limit(u, u_prev, max_delta):
    return min(max(u, u_prev - max_delta), u_prev + max_delta)


class SetpointShaper:
    def __init__(self, p, T0, RH0):
        self.p = p
        self.T, self.RH = T0, RH0

    def step(self, T_target, RH_target, Ts):
        p = self.p
        T_target = min(max(T_target, p["T_min"]), p["T_max"])
        RH_target = min(max(RH_target, p["RH_min"]), p["RH_max"])
        self.T = rate_limit(T_target, self.T, p["T_rate_K_per_s"] * Ts)
        self.RH = rate_limit(RH_target, self.RH, p["RH_rate_pct_per_s"] * Ts)
        return self.T, self.RH

"""합성 겨울철 기상(EnergyPlus EPW 대체). t [s] -> (T_out, I_sol[W/m2])."""
import math


def winter_day(t, T_mean=0.0, T_amp=5.0, I_peak=450.0):
    h = (t / 3600.0) % 24.0
    T_out = T_mean + T_amp * math.sin(2 * math.pi * (h - 9.0) / 24.0)  # 15시 최고
    I_sol = I_peak * max(0.0, math.sin(math.pi * (h - 7.0) / 10.0)) if 7.0 <= h <= 17.0 else 0.0
    return T_out, I_sol


def occupied(t):
    h = (t / 3600.0) % 24.0
    return 8.0 <= h < 20.0

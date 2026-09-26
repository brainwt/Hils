"""합성 기상 (EPW/EnergyPlus 대체). t [s] -> (T_out, RH_out, I_sol[W/m2])."""
import math


def make_weather(p):
    """p: {"T_mean","T_amp","RH","I_peak"}"""
    def weather(t):
        h = (t / 3600.0) % 24.0
        T_out = p["T_mean"] + p["T_amp"] * math.sin(2 * math.pi * (h - 9.0) / 24.0)  # 15시 최고
        I_sol = p["I_peak"] * max(0.0, math.sin(math.pi * (h - 7.0) / 10.0)) if 7.0 <= h <= 17.0 else 0.0
        return T_out, p["RH"], I_sol
    return weather


def occupied(t):
    h = (t / 3600.0) % 24.0
    return 8.0 <= h < 20.0

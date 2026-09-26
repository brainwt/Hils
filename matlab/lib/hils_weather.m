function [T_out, RH_out, I_sol, occ] = hils_weather(t, wp)
%HILS_WEATHER  합성 기상 (Python hils.weather 와 동일). wp = P.weather.winter / summer
h = mod(t / 3600, 24);
T_out = wp.T_mean + wp.T_amp * sin(2 * pi * (h - 9) / 24);
if h >= 7 && h <= 17
    I_sol = wp.I_peak * max(0, sin(pi * (h - 7) / 10));
else
    I_sol = 0;
end
RH_out = wp.RH;
occ = h >= 8 && h < 20;
end

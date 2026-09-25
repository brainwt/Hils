function [T_out, I_sol, occ] = hils_weather(t)
%HILS_WEATHER  합성 겨울철 기상 (Python hils.weather 와 동일). EPW/EnergyPlus 대체.
h = mod(t / 3600, 24);
T_out = 0 + 5 * sin(2 * pi * (h - 9) / 24);
if h >= 7 && h <= 17
    I_sol = 450 * max(0, sin(pi * (h - 7) / 10));
else
    I_sol = 0;
end
occ = h >= 8 && h < 20;
end

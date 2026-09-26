function p = hils_psy_pws(t)
%HILS_PSY_PWS  포화수증기압 [kPa] (ASHRAE 2017 Hyland-Wexler). t>=0 물, t<0 얼음.
T = t + 273.15;
if t >= 0
    ln = -5.8002206e3 / T + 1.3914993 - 4.8640239e-2 * T + 4.1764768e-5 * T^2 ...
         - 1.4452093e-8 * T^3 + 6.5459673 * log(T);
else
    ln = -5.6745359e3 / T + 6.3925247 - 9.677843e-3 * T + 6.2215701e-7 * T^2 ...
         + 2.0747825e-9 * T^3 - 9.484024e-13 * T^4 + 4.1635019 * log(T);
end
p = exp(ln) / 1000;
end

function v = hils_psy_v(t, W, P)
%HILS_PSY_V  습공기 비체적 [m3 / kg 건공기].
v = 0.287042 * (t + 273.15) * (1 + 1.607858 * W) / P;
end

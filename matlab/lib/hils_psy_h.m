function h = hils_psy_h(t, W)
%HILS_PSY_H  습공기 엔탈피 [kJ/kg_da].
h = 1.006 * t + W * (2501 + 1.86 * t);
end

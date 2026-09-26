function s = hils_zone_state(Z, wp)
%HILS_ZONE_STATE  현재 존 상태 [T_z RH_z W_z Tm T_out RH_out] 구조체.
[T_out, RH_out, I_sol] = hils_weather(Z.t, wp);
s.T_z = Z.Tz; s.RH_z = hils_psy_rh(Z.Tz, Z.Wz, Z.P); s.W_z = Z.Wz; s.Tm = Z.Tm;
s.T_out = T_out; s.RH_out = RH_out; s.I_sol = I_sol; s.t = Z.t;
end

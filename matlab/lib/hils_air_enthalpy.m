function a = hils_air_enthalpy(T_s, RH_s, T_r, RH_r, V_m3h, P)
%HILS_AIR_ENTHALPY  Air-enthalpy 법 열량 (ASHRAE 37 / KS C 9306 방식).
%   노즐은 실내기 토출측 -> 토출 공기 비체적으로 건공기 질량유량 환산.
%   a.Q_sens = m cp(W_r) (T_s - T_r),  a.Q_tot = m (h_s - h_r),  a.Q_lat = Q_tot - Q_sens  [W]
%   a.m_w    = m (W_s - W_r) [kg/s]    (존에 공급: 난방 +, 냉방/제습 -)
W_s = hils_psy_w(T_s, RH_s, P); W_r = hils_psy_w(T_r, RH_r, P);
v_s = hils_psy_v(T_s, W_s, P);
m = max(V_m3h, 0) / 3600 / v_s;
h_s = hils_psy_h(T_s, W_s); h_r = hils_psy_h(T_r, W_r);
cp = 1.006 + 1.86 * W_r;
a.m_da = m; a.W_s = W_s; a.W_r = W_r; a.h_s = h_s; a.h_r = h_r;
a.Q_sens = 1000 * m * cp * (T_s - T_r);
a.Q_tot = 1000 * m * (h_s - h_r);
a.Q_lat = 1000 * m * ((h_s - h_r) - cp * (T_s - T_r));
a.m_w = m * (W_s - W_r);
end

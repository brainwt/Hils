function Z = hils_zone_init(p, Pa)
%HILS_ZONE_INIT  가상 존 상태 초기화 (존 공기 T,W + 구조체 Tm).
Z.Tz = p.T0; Z.Tm = p.Tm0;
Z.Wz = hils_psy_w(p.T0, p.RH0, Pa);
rho = 1 / hils_psy_v(p.T0, Z.Wz, Pa);
m_air = rho * p.V_zone_m3;
Z.Cz = m_air * 1006 * p.C_air_mult;
Z.Mz = m_air * p.moist_mult;
Z.m_inf = p.ACH_inf * m_air / 3600;
Z.P = Pa;
Z.t = 0;
end

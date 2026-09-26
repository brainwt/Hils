function [Z, s] = hils_zone_step(Z, p, wp, Ts, Ts_sub, Q_sens, m_w)
%HILS_ZONE_STEP  가상 존 [t, t+Ts] 적분 -> t+Ts 의 존 공기 상태 (Python VirtualZone.step 과 동일).
%   입력 Q_sens [W], m_w [kg/s] : 구간 평균 air-enthalpy 실측 (히트펌프가 존에 공급)
n = round(Ts / Ts_sub);
for k = 1:n
    [T_out, RH_out, I_sol, occ] = hils_weather(Z.t, wp);
    W_out = hils_psy_w(T_out, RH_out, Z.P);
    if occ
        Q_int = p.Q_int_occupied; g_int = p.g_int_occupied;
    else
        Q_int = p.Q_int_unoccupied; g_int = p.g_int_unoccupied;
    end
    q_mass = (Z.Tm - Z.Tz) / p.R_int;
    dTz = (q_mass + (T_out - Z.Tz) / p.R_win ...
           + Z.m_inf * 1000 * (1.006 + 1.86 * Z.Wz) * (T_out - Z.Tz) ...
           + p.f_sol_air * p.A_sol * I_sol + Q_int + Q_sens) / Z.Cz;
    dTm = (-q_mass + (T_out - Z.Tm) / p.R_ext + (1 - p.f_sol_air) * p.A_sol * I_sol) / p.C_mass;
    dWz = (Z.m_inf * (W_out - Z.Wz) + g_int + m_w) / Z.Mz;
    Z.Tz = Z.Tz + Ts_sub * dTz;
    Z.Tm = Z.Tm + Ts_sub * dTm;
    Z.Wz = max(Z.Wz + Ts_sub * dWz, 1e-5);
    Z.t = Z.t + Ts_sub;
end
s = hils_zone_state(Z, wp);
end

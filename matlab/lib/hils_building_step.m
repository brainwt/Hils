function [B, out] = hils_building_step(B, p, Ts, t, T_zone_meas)
%HILS_BUILDING_STEP  가상건물 2R2C (EnergyPlus/FMU 대체). Ts_building 마다 1회 호출.
%   존 공기 = 실측 챔버온도, 구조체 질량 노드 Tm 은 가상.
%   Q_target > 0 : 난방부하(챔버에서 열을 제거해야 함)
%   B = struct('Tm', 18)
[T_out, I_sol, occ] = hils_weather(t);
if occ, Q_int = p.Q_int_occupied; else, Q_int = p.Q_int_unoccupied; end
q_mass = (T_zone_meas - B.Tm) / p.R_int;
out.Q_target = q_mass + (T_zone_meas - T_out) / p.R_win - p.A_sol * I_sol - Q_int;
B.Tm = B.Tm + Ts * (q_mass + (T_out - B.Tm) / p.R_ext) / p.C_mass;
out.T_out = T_out; out.I_sol = I_sol; out.Q_int = Q_int; out.Tm = B.Tm;
end

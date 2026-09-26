function [SPs, T, RH] = hils_setpoint_step(SPs, p, T_target, RH_target, Ts)
%HILS_SETPOINT_STEP  가상 존 상태 -> 챔버 설정값: 범위 제한 + 변화율 제한.  SPs = struct('T',..,'RH',..)
T_target = min(max(T_target, p.T_min), p.T_max);
RH_target = min(max(RH_target, p.RH_min), p.RH_max);
dT = p.T_rate_K_per_s * Ts; dR = p.RH_rate_pct_per_s * Ts;
SPs.T = min(max(T_target, SPs.T - dT), SPs.T + dT);
SPs.RH = min(max(RH_target, SPs.RH - dR), SPs.RH + dR);
T = SPs.T; RH = SPs.RH;
end

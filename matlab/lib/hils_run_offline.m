function [L, S, E, Z] = hils_run_offline(P, duration_s, opts)
%HILS_RUN_OFFLINE  MATLAB/Octave 만으로 전체 폐루프 (Python cosim.run_hils + InMemory 와 동일 순서).
%   1 s : 측정 decode -> supervisor(air-enthalpy 누적, 안전, 설정값) -> encode -> PLC 스캔
%   60 s: 구간 평균 열량 -> 가상 존 step -> 존 상태(k+1) -> 챔버 목표, Sequence++
%   opts.profile = [t T RH; ...] 이면 존 대신 설정값 프로파일 (Python ProfileZone)
if nargin < 3, opts = struct(); end
Ts = P.timing.Ts_realization; Tb = P.timing.Ts_building; nb = round(Tb / Ts);
S = hils_supervisor_init(P);
E = hils_plc_emulator_init(P);
Z = hils_zone_init(P.zone, P.emulator.P_atm);
zs = hils_zone_state(Z, P.wx);
n = floor(duration_s / Ts) + 1;
f = {'t','state','fault','seq','ack','latency','valid','T_z','RH_z','T_sp','RH_sp', ...
     'T_supply','RH_supply','T_return','RH_return','V_air','Q_sens','Q_lat','Q_tot','m_w','P_HP','HP_status'};
for i = 1:numel(f), L.(f{i}) = zeros(n, 1); end
for i = 1:n
    t = (i - 1) * Ts;
    m = hils_decode_meas(E.meas_words, P);
    if mod(i - 1, nb) == 0
        if isfield(opts, 'profile')
            r = find(t >= opts.profile(:, 1), 1, 'last');
            zs = struct('T_z', opts.profile(r, 2), 'RH_z', opts.profile(r, 3), 'T_out', 0, 'RH_out', 70);
        elseif i > 1
            [S, q] = hils_supervisor_interval(S);
            [Z, zs] = hils_zone_step(Z, P.zone, P.wx, Tb, P.timing.Ts_zone_sub, q.Q_sens, q.m_w);
        end
        S = hils_supervisor_building(S, P, zs.T_z, zs.RH_z, zs.T_out, zs.RH_out);
    end
    [S, cmd, st] = hils_supervisor_step(S, P, t, m);
    L.t(i) = t; L.state(i) = st.state; L.fault(i) = st.fault; L.seq(i) = S.seq;
    L.ack(i) = m.AckSequence; L.latency(i) = st.latency; L.valid(i) = st.valid;
    L.T_z(i) = zs.T_z; L.RH_z(i) = zs.RH_z; L.T_sp(i) = cmd.T_room_SP; L.RH_sp(i) = cmd.RH_room_SP;
    L.T_supply(i) = m.T_supply; L.RH_supply(i) = m.RH_supply; L.T_return(i) = m.T_return;
    L.RH_return(i) = m.RH_return; L.V_air(i) = m.V_air; L.Q_sens(i) = st.Q_sens;
    L.Q_lat(i) = st.Q_lat; L.Q_tot(i) = st.Q_tot; L.m_w(i) = st.m_w; L.P_HP(i) = m.P_HP;
    L.HP_status(i) = m.HP_status;
    [E, ~] = hils_plc_emulator_step(E, P, hils_encode_cmd(cmd, P));
end
end

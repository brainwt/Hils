function [L, S] = hils_run_realtime(P, duration_s, io, opts)
%HILS_RUN_REALTIME  Simulink 없이 MATLAB 스크립트로 HILS 를 실시간 운전하는 master.
%   [L, S] = hils_run_realtime(P, 3600, hils_io_modbus(P))       % 실제/가짜 PLC
%   [L, S] = hils_run_realtime(P, 600,  hils_io_emulator(P), struct('pacing', 0))
%   opts.pacing : 1 이면 wall-clock 에 맞춰 1 s 주기 실행 (기본 1)
%   opts.ZoneFcn: @(k, t, q) -> zs (구조체 T_z RH_z T_out RH_out). EnergyPlus/FMU 연동 지점.
%                 q = 직전 구간 평균 air-enthalpy 열량 (Q_sens, Q_lat, m_w). 기본: 내장 가상 존
%   opts.events : {t, @(io) ...} 셀 배열 (장애 주입)
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'pacing'), opts.pacing = 1; end
if ~isfield(opts, 'events'), opts.events = {}; end
Ts = P.timing.Ts_realization; Tb = P.timing.Ts_building; nb = round(Tb / Ts);
S = hils_supervisor_init(P);
Z = hils_zone_init(P.zone, P.emulator.P_atm);
zs = hils_zone_state(Z, P.wx);
n = floor(duration_s / Ts) + 1;
f = {'t','state','fault','seq','ack','latency','valid','T_z','RH_z','T_sp','RH_sp', ...
     'T_supply','RH_supply','T_return','RH_return','V_air','Q_sens','Q_lat','Q_tot','P_HP','HP_status','PLC_status'};
for i = 1:numel(f), L.(f{i}) = zeros(n, 1); end
t0 = tic;
for i = 1:n
    t = (i - 1) * Ts;
    for j = 1:2:numel(opts.events)
        if opts.events{j} == t, opts.events{j + 1}(io); end
    end
    m = hils_decode_meas(io.read(), P);
    if mod(i - 1, nb) == 0
        if i > 1
            [S, q] = hils_supervisor_interval(S);
            if isfield(opts, 'ZoneFcn')
                zs = opts.ZoneFcn((i - 1) / nb, t, q);
            else
                [Z, zs] = hils_zone_step(Z, P.zone, P.wx, Tb, P.timing.Ts_zone_sub, q.Q_sens, q.m_w);
            end
        end
        S = hils_supervisor_building(S, P, zs.T_z, zs.RH_z, zs.T_out, zs.RH_out);
    end
    [S, cmd, st] = hils_supervisor_step(S, P, t, m);
    io.write(hils_encode_cmd(cmd, P));
    L.t(i) = t; L.state(i) = st.state; L.fault(i) = st.fault; L.seq(i) = S.seq;
    L.ack(i) = m.AckSequence; L.latency(i) = st.latency; L.valid(i) = st.valid;
    L.T_z(i) = zs.T_z; L.RH_z(i) = zs.RH_z; L.T_sp(i) = cmd.T_room_SP; L.RH_sp(i) = cmd.RH_room_SP;
    L.T_supply(i) = m.T_supply; L.RH_supply(i) = m.RH_supply; L.T_return(i) = m.T_return;
    L.RH_return(i) = m.RH_return; L.V_air(i) = m.V_air; L.Q_sens(i) = st.Q_sens;
    L.Q_lat(i) = st.Q_lat; L.Q_tot(i) = st.Q_tot; L.P_HP(i) = m.P_HP; L.HP_status(i) = m.HP_status;
    L.PLC_status(i) = m.PLC_status;
    if opts.pacing
        dt = t + Ts - toc(t0);
        if dt > 0, pause(dt); end
    end
end
end

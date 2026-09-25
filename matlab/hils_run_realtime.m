function [L, S] = hils_run_realtime(P, duration_s, io, opts)
%HILS_RUN_REALTIME  Simulink 없이 MATLAB 스크립트로 HILS 를 실시간 운전하는 master.
%   [L, S] = hils_run_realtime(P, 3600, hils_io_modbus(P))       % 실제/가짜 PLC
%   [L, S] = hils_run_realtime(P, 600,  hils_io_emulator(P), struct('pacing', 0))
%   opts.pacing     : 1 이면 wall-clock 에 맞춰 Ts_realization 주기로 실행 (기본 1)
%   opts.BuildingFcn: @(t, T_indoor) -> [Q_target, T_target, T_out] (기본 2R2C)
%   opts.events     : {t, @(io) ...} 셀 배열. 장애 주입 등
%   Simulink 모델과 동일한 lib 함수를 쓰므로, Simulink 라이선스 없이도 PLC 루프 시험 가능.
if nargin < 4, opts = struct(); end
if ~isfield(opts, 'pacing'), opts.pacing = 1; end
if ~isfield(opts, 'events'), opts.events = {}; end
Ts = P.timing.Ts_realization; Tb = P.timing.Ts_building;
B = struct('Tm', 18);
S = hils_supervisor_init(P);
n = floor(duration_s / Ts) + 1;
f = {'t','state','fault','seq','ack','latency','valid','Q_target','Q_ref','Q_load_meas', ...
     'Q_load_cmd','Q_HP','P_HP','T_indoor','HP_status','PLC_status'};
for i = 1:numel(f), L.(f{i}) = zeros(n, 1); end
t0 = tic;
for i = 1:n
    t = (i - 1) * Ts;
    for j = 1:2:numel(opts.events)
        if opts.events{j} == t, opts.events{j + 1}(io); end
    end
    m = hils_decode_meas(io.read(), P);
    if mod(i - 1, round(Tb / Ts)) == 0
        if isfield(opts, 'BuildingFcn')
            [Q, T, To] = opts.BuildingFcn(t, m.T_indoor);
        else
            [B, bo] = hils_building_step(B, P.virtual_building, Tb, t, m.T_indoor);
            Q = bo.Q_target; T = P.emulator.hp_setpoint; To = bo.T_out;
        end
        S = hils_supervisor_building(S, P, Q, T, To);
    end
    [S, cmd, st] = hils_supervisor_step(S, P, t, m);
    io.write(hils_encode_cmd(cmd, P));
    L.t(i) = t; L.state(i) = st.state; L.fault(i) = st.fault; L.seq(i) = S.seq;
    L.ack(i) = m.AckSequence; L.latency(i) = st.latency; L.valid(i) = st.valid;
    L.Q_target(i) = S.Q_target; L.Q_ref(i) = st.q_ref; L.Q_load_meas(i) = m.Q_load_meas;
    L.Q_load_cmd(i) = cmd.Q_load_cmd; L.Q_HP(i) = m.Q_HP; L.P_HP(i) = m.P_HP;
    L.T_indoor(i) = m.T_indoor; L.HP_status(i) = m.HP_status; L.PLC_status(i) = m.PLC_status;
    if opts.pacing
        dt = t + Ts - toc(t0);
        if dt > 0, pause(dt); end
    end
end
end

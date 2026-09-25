function [L, S, E, B] = hils_run_offline(P, duration_s, opts)
%HILS_RUN_OFFLINE  MATLAB/Octave 만으로 전체 폐루프 실행 (Simulink 없이 로직 검증).
%   가상건물(60 s) -> supervisor(1 s) -> 레지스터 encode -> PLC 에뮬레이터 -> decode
%   Python hils.cosim.run_hils(InMemoryTransport) 와 동일한 실행 순서.
%   opts.profile = [t_start Q; ...] 이면 계단 부하 프로파일 사용 (StepProfileBuilding)
if nargin < 3, opts = struct(); end
Ts = P.timing.Ts_realization; Tb = P.timing.Ts_building;
S = hils_supervisor_init(P);
E = hils_plc_emulator_init(P);
B = struct('Tm', 18);
n = floor(duration_s / Ts) + 1;
f = {'t','state','fault','seq','ack','pending','latency','valid','Q_target','Q_ref', ...
     'Q_load_meas','Q_load_cmd','Q_HP','P_HP','T_indoor','T_outdoor','HP_status','Kp'};
for i = 1:numel(f), L.(f{i}) = zeros(n, 1); end
cmd_words = hils_encode_cmd(struct('Q_load_cmd',0,'T_chamber_SP',0,'Enable',0,'Sequence',0, ...
                                   'SIM_heartbeat',0,'T_outdoor_SP',0), P);
for i = 1:n
    t = (i - 1) * Ts;
    m = hils_decode_meas(E.meas_words, P);
    if mod(i - 1, round(Tb / Ts)) == 0
        if isfield(opts, 'profile')
            q = 0;
            for r = 1:size(opts.profile, 1)
                if t >= opts.profile(r, 1), q = opts.profile(r, 2); end
            end
            bo = struct('Q_target', q, 'T_out', 0);
        else
            [B, bo] = hils_building_step(B, P.virtual_building, Tb, t, m.T_indoor);
        end
        S = hils_supervisor_building(S, P, bo.Q_target, P.emulator.hp_setpoint, bo.T_out);
    end
    [S, cmd, st] = hils_supervisor_step(S, P, t, m);
    cmd_words = hils_encode_cmd(cmd, P);
    L.t(i) = t; L.state(i) = st.state; L.fault(i) = st.fault; L.seq(i) = S.seq;
    L.ack(i) = m.AckSequence; L.pending(i) = st.pending; L.latency(i) = st.latency;
    L.valid(i) = st.valid; L.Q_target(i) = S.Q_target; L.Q_ref(i) = st.q_ref;
    L.Q_load_meas(i) = m.Q_load_meas; L.Q_load_cmd(i) = cmd.Q_load_cmd; L.Q_HP(i) = m.Q_HP;
    L.P_HP(i) = m.P_HP; L.T_indoor(i) = m.T_indoor; L.T_outdoor(i) = m.T_outdoor;
    L.HP_status(i) = m.HP_status; L.Kp(i) = st.kp;
    [E, ~] = hils_plc_emulator_step(E, P, cmd_words);
end
end

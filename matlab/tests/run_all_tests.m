function run_all_tests()
%RUN_ALL_TESTS  MATLAB/Octave 단위시험 + 폐루프 시험. 실패 시 error.
%   cd matlab; addpath tests; run_all_tests
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
addpath(fullfile(root, 'lib')); addpath(root);
tests = {@t_codec, @t_frames, @t_pi, @t_delay_gains, @t_delay_monitor, @t_safety, ...
         @t_state_machine, @t_params_const_in_sync, @t_closed_loop_step, ...
         @t_closed_loop_delay90, @t_io_emulator_equivalence, @t_estop};
nfail = 0;
for i = 1:numel(tests)
    name = func2str(tests{i});
    try
        tests{i}();
        fprintf('  PASS  %s\n', name);
    catch err
        nfail = nfail + 1;
        fprintf('  FAIL  %s : %s\n', name, err.message);
    end
end
fprintf('%d / %d passed\n', numel(tests) - nfail, numel(tests));
if nfail > 0
    error('HILS:tests', '%d test(s) failed', nfail);
end
end

function check(cond, msg)
if ~cond, error(msg); end
end

function t_codec()
check(abs(hils_decode(2357, 100, true) - 23.57) < 1e-12, 'plan example 2357');
[w, o] = hils_encode(23.57, 100, true); check(w == 2357 && ~o, 'encode 23.57');
[w, o] = hils_encode(-1, 1, true); check(w == 65535 && ~o, 'negative');
[w, o] = hils_encode(40000, 1, true); check(w == 32767 && o, 'overflow');
[w, o] = hils_encode(-5, 1, false); check(w == 0 && o, 'unsigned underflow');
check(hils_encode(2.5, 1, true) == 3 && hils_encode(-2.5, 1, true) == 65536 - 3, 'half away');
check(hils_decode(hils_encode(-12.34, 100, true), 100, true) == -12.34, 'roundtrip');
end

function t_frames()
P = hils_params();
c = struct('Q_load_cmd', -3200, 'T_chamber_SP', 22.5, 'Enable', 1, 'Sequence', 125, ...
           'SIM_heartbeat', 9, 'T_outdoor_SP', -7.25);
w = hils_encode_cmd(c, P);
check(isequal(w(:)', [65536-3200 2250 1 125 9 65536-725]), 'cmd words');
c2 = hils_decode_cmd(w, P);
check(isequal(struct2cell(c2), struct2cell(c)), 'cmd roundtrip');
m = struct('T_indoor', 23.57, 'RH_indoor', 45.5, 'T_outdoor', -3.2, 'P_HP', 1500, 'Q_HP', 4200, ...
           'HP_status', 1, 'Q_load_meas', -300, 'AckSequence', 125, 'PLC_status', 9, 'PLC_heartbeat', 77);
m2 = hils_decode_meas(hils_encode_meas(m, P), P);
check(max(abs(cell2mat(struct2cell(m2)) - cell2mat(struct2cell(m)))) < 1e-9, 'meas roundtrip');
end

function t_pi()
p = hils_params().realization_controller;
S = struct('integ', 0, 'u_prev', 0);
[u, S] = hils_pi_step(S, 5000, 0, 5000, p, 1, true, false, p.Kp, p.Ki);
check(abs(u - p.rate_limit_W_per_s) < 1e-12, 'rate limit');
S = struct('integ', 0, 'u_prev', 0);
for k = 1:2000, [u, S] = hils_pi_step(S, 20000, 0, 20000, p, 1, true, false, p.Kp, p.Ki); end
I = S.integ;
for k = 1:100, [u, S] = hils_pi_step(S, 20000, 0, 20000, p, 1, true, false, p.Kp, p.Ki); end
check(u == p.Q_cmd_max && S.integ == I, 'anti-windup');
S = struct('integ', 0, 'u_prev', 0); y = 0;
for k = 1:2000
    [u, S] = hils_pi_step(S, 3000, y, 3000, p, 1, true, false, p.Kp, p.Ki);
    y = y + (0.93 * u - 150 - y) / 20;
end
check(abs(3000 - y) < 1, 'steady state error');
[u, S] = hils_pi_step(struct('integ', 500, 'u_prev', 120), 3000, 0, 3000, p, 1, false, false, p.Kp, p.Ki);
check(abs(u - 70) < 1e-12 && S.integ == 0, 'disable ramp');
end

function t_delay_gains()
p = hils_params().realization_controller;
[kp, ki] = hils_delay_gains(p, NaN); check(kp == p.Kp && ki == p.Ki, 'nan');
[kp, ki] = hils_delay_gains(p, 4);   check(kp == p.Kp && ki == p.Ki, 'short');
[kp, ki] = hils_delay_gains(p, 90);  check(abs(kp - 20/180) < 1e-12 && abs(ki - kp/20) < 1e-12, '90 s');
end

function t_delay_monitor()
D = hils_delay_monitor_init(64);
[D, pend] = hils_delay_monitor_step(D, true, 1, 1000, 0, 0, 120);   check(pend, 'pending');
[D, pend] = hils_delay_monitor_step(D, true, 2, 2000, 0, 60, 120);  check(pend, 'pending 2');
[D, pend] = hils_delay_monitor_step(D, false, 2, 2000, 1, 91, 120);
check(pend && D.last_latency == 91 && D.q_ack == 1000, 'first ack');
[D, pend, to] = hils_delay_monitor_step(D, false, 2, 2000, 1, 170, 120);
check(pend && ~to, 'no timeout yet (age 110)');
[D, pend, to] = hils_delay_monitor_step(D, false, 2, 2000, 1, 181, 120); check(to, 'timeout');
[D, pend] = hils_delay_monitor_step(D, false, 2, 2000, 2, 182, 120);
check(~pend && D.q_ack == 2000, 'cleared');
% 65535 -> 1 wrap
D = hils_delay_monitor_init(8);
D = hils_delay_monitor_step(D, true, 65535, 1, 0, 0, 120);
D = hils_delay_monitor_step(D, true, 1, 2, 0, 60, 120);
[D, pend] = hils_delay_monitor_step(D, false, 1, 2, 1, 65, 120);
check(~pend, 'wrap clears older');
end

function t_safety()
P = hils_params();
m = struct('T_indoor', 22, 'RH_indoor', 40, 'HP_status', 1, 'PLC_status', 1, 'P_HP', 1500);
f = @(mm, hb, ak) hils_safety_check(mm, P.safety, P.plc_status_bits, P.hp_status_codes, hb, ak);
check(f(m, false, false) == 0, 'ok');
m2 = m; m2.T_indoor = 36; check(f(m2, false, false) == 1, 'T');
m2 = m; m2.HP_status = 9; check(f(m2, false, false) == 4, 'HP');
m2 = m; m2.PLC_status = 5; check(f(m2, false, false) == 8, 'estop');
check(f(m, true, true) == 48, 'hb+ack');
end

function t_state_machine()
p = hils_params().state_machine;
SM = struct('state', 0, 'timer', 0, 'ok_timer', 0, 'warning', 0);
SM = hils_state_machine_step(SM, p, 1, true, false, 1000, false); check(SM.state == 1, 'WAIT');
SM = hils_state_machine_step(SM, p, 1, true, false, 1000, false); check(SM.state == 2, 'STAB');
for k = 1:p.stabilize_hold_s, SM = hils_state_machine_step(SM, p, 1, true, false, 10, false); end
check(SM.state == 3, 'RUN');
SM = hils_state_machine_step(SM, p, 1, true, false, 10, true);  check(SM.state == 4, 'STEP');
SM = hils_state_machine_step(SM, p, 1, true, false, 10, false); check(SM.state == 2, 'STAB2');
SM = hils_state_machine_step(SM, p, 1, true, true, 10, false);  check(SM.state == 5, 'SAFE');
for k = 1:p.fault_recover_hold_s, SM = hils_state_machine_step(SM, p, 1, true, false, 10, false); end
check(SM.state == 1, 'recover');
end

function t_params_const_in_sync()
% hils_params_const.m (codegen 용) 가 JSON 과 일치하는지 - JSON 수정 후 재생성 누락 검출
P = hils_params(); C = hils_params_const();
compare(P, C, 'P');
end

function compare(a, b, name)
if isstruct(a)
    f = fieldnames(a);
    for i = 1:numel(f)
        if ischar(a.(f{i})), continue; end
        check(isfield(b, f{i}), [name '.' f{i} ' missing in hils_params_const']);
        compare(a.(f{i}), b.(f{i}), [name '.' f{i}]);
    end
else
    check(isequal(double(a), double(b)), [name ' differs: rerun hils_write_params_const']);
end
end

function t_closed_loop_step()
P = hils_params();
L = hils_run_offline(P, 7200, struct('profile', [0 2000; 1800 3500; 3600 1500; 5400 3000]));
v = L.valid > 0; e = L.Q_ref - L.Q_load_meas;
check(mean(v) > 0.85, sprintf('valid ratio %.2f', mean(v)));
check(sqrt(mean(e(v).^2)) < 50, 'rmse');
check(sum(L.state == 4) == 3, 'three STEP_CHANGE');
check(~any(L.state == 5), 'no SAFE_STOP');
end

function t_closed_loop_delay90()
P = hils_params(); P.emulator.ack_delay_s = 90;
L = hils_run_offline(P, 7200, struct('profile', [0 2000; 1800 3500; 3600 1500; 5400 3000]));
check(~any(L.state == 5), 'no SAFE_STOP with delay compensation');
check(max(L.latency) == 91, 'latency 91 s');
check(min(L.Kp(L.t > 200)) < 0.2, 'gains detuned');
end

function t_io_emulator_equivalence()
P = hils_params();
pr = [0 2000; 1800 3500];
L1 = hils_run_offline(P, 1200, struct('profile', pr));
bf = @(t, Ti) deal(2000 * (t < 1800) + 3500 * (t >= 1800), P.emulator.hp_setpoint, 0);
L2 = hils_run_realtime(P, 1200, hils_io_emulator(P), struct('pacing', 0, 'BuildingFcn', bf));
check(isequal(L1.Q_load_cmd, L2.Q_load_cmd) && isequal(L1.T_indoor, L2.T_indoor), 'io path equal');
end

function t_estop()
P = hils_params();
io = hils_io_emulator(P);
ev = {300, @(io) io.set('estop', true)};
bf = @(t, Ti) deal(2500, P.emulator.hp_setpoint, 0);
L = hils_run_realtime(P, 600, io, struct('pacing', 0, 'BuildingFcn', bf, 'events', {ev}));
check(L.state(end) == 5 && any(bitand(L.fault, 8)), 'estop -> SAFE_STOP');
end

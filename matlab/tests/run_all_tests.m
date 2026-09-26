function run_all_tests()
%RUN_ALL_TESTS  MATLAB/Octave 단위시험 + 폐루프 시험. 실패 시 error.
%   cd matlab; addpath tests; run_all_tests
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
addpath(fullfile(root, 'lib')); addpath(root);
tests = {@t_codec, @t_frames, @t_psychro, @t_air_enthalpy, @t_coil, @t_zone_balance, ...
         @t_delay_monitor, @t_safety, @t_state_machine, @t_state_machine_latch, @t_setpoint, ...
         @t_params_const_in_sync, @t_closed_loop_winter, @t_closed_loop_summer, ...
         @t_io_emulator_equivalence, @t_estop, @t_simulink_block_scripts};
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
if nfail > 0, error('HILS:tests', '%d test(s) failed', nfail); end
end

function check(cond, msg)
if ~cond, error(msg); end
end

function t_codec()
check(abs(hils_decode(2357, 100, true) - 23.57) < 1e-12, '2357');
[w, o] = hils_encode(-1, 1, true); check(w == 65535 && ~o, 'negative');
[w, o] = hils_encode(40000, 1, true); check(w == 32767 && o, 'overflow');
check(hils_encode(2.5, 1, true) == 3 && hils_encode(-2.5, 1, true) == 65536 - 3, 'half away');
check(hils_decode(hils_encode(101.33, 100, false), 100, false) == 101.33, 'kPa');
end

function t_frames()
P = hils_params();
c = struct('T_room_SP', 22.47, 'RH_room_SP', 41.5, 'T_outdoor_SP', -3.25, 'RH_outdoor_SP', 70, ...
           'Enable', 1, 'Sequence', 125, 'SIM_heartbeat', 9);
w = hils_encode_cmd(c, P);
check(isequal(w(:)', [2247 4150 65536-325 7000 1 125 9]), 'cmd words');
check(isequal(struct2cell(hils_decode_cmd(w, P)), struct2cell(c)), 'cmd roundtrip');
m = struct('T_supply', 35.2, 'RH_supply', 15.3, 'T_return', 20.1, 'RH_return', 40.2, 'V_air', 887, ...
           'P_atm', 101.33, 'T_chamber', 20.1, 'RH_chamber', 40.2, 'T_outdoor', -2.5, 'RH_outdoor', 70, ...
           'P_HP', 1500, 'HP_status', 1, 'AckSequence', 125, 'PLC_status', 9, 'PLC_heartbeat', 77);
m2 = hils_decode_meas(hils_encode_meas(m, P), P);
check(max(abs(cell2mat(struct2cell(m2)) - cell2mat(struct2cell(m)))) < 1e-9, 'meas roundtrip');
end

function t_psychro()
Pa = 101.325;
check(abs(hils_psy_pws(20) - 2.3393) / 2.3393 < 2e-3, 'pws 20');
check(abs(hils_psy_pws(-10) - 0.2599) / 0.2599 < 2e-3, 'pws -10');
W = hils_psy_w(25, 50, Pa);
check(abs(W - 0.00988) / 0.00988 < 5e-3, 'W');
check(abs(hils_psy_h(25, W) - 50.3) < 0.2, 'h');
check(abs(hils_psy_v(25, W, Pa) - 0.858) < 0.002, 'v');
check(abs(hils_psy_tdp(W, Pa) - 13.86) < 0.05, 'dew point');
check(abs(hils_psy_rh(18.3, hils_psy_w(18.3, 63, Pa), Pa) - 63) < 1e-9, 'rh inverse');
check(abs(hils_psy_tsat_h(hils_psy_h(12, hils_psy_w(12, 100, Pa)), Pa) - 12) < 1e-6, 'tsat');
end

function t_air_enthalpy()
Pa = 101.325;
W = hils_psy_w(20, 40, Pa);
a = hils_air_enthalpy(35, hils_psy_rh(35, W, Pa), 20, 40, 900, Pa);
check(abs(a.Q_lat) < 1e-6, 'heating latent 0');
m = 900 / 3600 / hils_psy_v(35, W, Pa);
check(abs(a.Q_sens - 1000 * m * (1.006 + 1.86 * W) * 15) < 1e-6, 'heating sensible');
a = hils_air_enthalpy(13, 95, 27, 50, 900, Pa);
check(a.Q_sens < 0 && a.Q_lat < 0 && a.m_w < 0, 'cooling signs');
check(abs(a.Q_tot - a.Q_sens - a.Q_lat) < 1e-9, 'split');
end

function t_coil()
Pa = 101.325;
W = hils_psy_w(27, 50, Pa); m = 0.28;
[Ts, Ws] = hils_coil_outlet(27, W, 5000, m, true, 0.15, Pa);
check(Ws < W && Ts < 27, 'wet coil dehumidifies');
check(abs(1000 * m * (hils_psy_h(27, W) - hils_psy_h(Ts, Ws)) - 5000) < 1e-6, 'energy');
Wd = hils_psy_w(27, 20, Pa);
[~, Ws] = hils_coil_outlet(27, Wd, 1000, m, true, 0.15, Pa);
check(Ws == Wd, 'dry coil');
[Ts, Ws] = hils_coil_outlet(20, W, 3000, m, false, 0.15, Pa);
check(Ws == W && Ts > 20, 'heating');
end

function t_zone_balance()
P = hils_params();
p = P.zone; p.C_mass = 3e5; p.Q_int_occupied = 0; p.Q_int_unoccupied = 0;
p.g_int_occupied = 0; p.g_int_unoccupied = 0;
wp = struct('T_mean', 0, 'T_amp', 0, 'RH', 70, 'I_peak', 0);
Z = hils_zone_init(p, 101.325);
for k = 1:3000, [Z, s] = hils_zone_step(Z, p, wp, 60, 10, 3000, 0); end
loss = s.T_z / p.R_win + s.Tm / p.R_ext + Z.m_inf * 1000 * (1.006 + 1.86 * s.W_z) * s.T_z;
check(abs(loss - 3000) / 3000 < 1e-3, sprintf('steady balance %.1f', loss));
end

function t_delay_monitor()
D = hils_delay_monitor_init(64);
[D, pend] = hils_delay_monitor_step(D, true, 1, 20, 40, 0, 0, 180);   check(pend, 'pending');
D = hils_delay_monitor_step(D, true, 2, 20.3, 40.5, 0, 60, 180);
[D, pend] = hils_delay_monitor_step(D, false, 2, 0, 0, 1, 91, 180);
check(pend && D.last_latency == 91 && D.T_ack == 20 && D.RH_ack == 40, 'first ack');
[D, pend, to] = hils_delay_monitor_step(D, false, 2, 0, 0, 1, 240, 180); check(~to, 'age 180');
[D, pend, to] = hils_delay_monitor_step(D, false, 2, 0, 0, 1, 240.5, 180); check(to, 'timeout');
[D, pend] = hils_delay_monitor_step(D, false, 2, 0, 0, 2, 242, 180);
check(~pend && D.T_ack == 20.3, 'cleared');
end

function t_safety()
P = hils_params();
m = struct('T_chamber', 22, 'RH_chamber', 40, 'HP_status', 1, 'PLC_status', 1, 'P_HP', 1500, 'V_air', 890);
f = @(mm, a, b, c) hils_safety_check(mm, P.safety, P.plc_status_bits, P.hp_status_codes, a, b, c);
check(f(m, false, false, false) == 0, 'ok');
m2 = m; m2.T_chamber = 41; check(f(m2, false, false, false) == 1, 'T');
m2 = m; m2.RH_chamber = 97; check(f(m2, false, false, false) == 2, 'RH');
m2 = m; m2.PLC_status = 17; check(f(m2, false, false, false) == 8, 'watchdog');
m2 = m; m2.V_air = 20; check(f(m2, false, false, false) == 128, 'airflow');
m2 = m; m2.V_air = 0; m2.HP_status = 0; check(f(m2, false, false, false) == 0, 'off no flow ok');
check(f(m, true, true, true) == 16 + 32 + 256, 'hb+ack+track');
end

function t_state_machine()
p = hils_params().state_machine;
SM = struct('state', 0, 'timer', 0, 'ok_timer', 0, 'warning', 0, 'restarts', 0, 'latched', false);
SM = hils_state_machine_step(SM, p, 1, true, false, false, false); check(SM.state == 1, 'WAIT');
SM = hils_state_machine_step(SM, p, 1, true, false, false, false); check(SM.state == 2, 'STAB');
for k = 1:p.stabilize_hold_s, SM = hils_state_machine_step(SM, p, 1, true, false, true, false); end
check(SM.state == 3, 'RUN');
SM = hils_state_machine_step(SM, p, 1, true, false, true, true);  check(SM.state == 4, 'STEP');
SM = hils_state_machine_step(SM, p, 1, true, false, true, false); check(SM.state == 2, 'STAB2');
SM = hils_state_machine_step(SM, p, 1, true, true, true, false);  check(SM.state == 5, 'SAFE');
for k = 1:p.fault_recover_hold_s, SM = hils_state_machine_step(SM, p, 1, true, false, true, false); end
check(SM.state == 1, 'recover');
end

function t_state_machine_latch()
p = hils_params().state_machine; p.max_auto_restarts = 1; p.fault_recover_hold_s = 2;
SM = struct('state', 0, 'timer', 0, 'ok_timer', 0, 'warning', 0, 'restarts', 0, 'latched', false);
SM = hils_state_machine_step(SM, p, 1, true, false, false, false);
SM = hils_state_machine_step(SM, p, 1, true, false, false, false);
for r = 1:2
    SM = hils_state_machine_step(SM, p, 1, true, true, false, false); check(SM.state == 5, 'trip');
    for k = 1:4, SM = hils_state_machine_step(SM, p, 1, true, false, false, false); end
end
check(SM.latched && SM.state == 5 && SM.warning == 3, 'latched');
end

function t_setpoint()
p = hils_params().setpoint;
S = struct('T', 20, 'RH', 40);
[S, T, RH] = hils_setpoint_step(S, p, 25, 60, 1);
check(abs(T - 20.05) < 1e-12 && abs(RH - 40.2) < 1e-12, 'rate');
for k = 1:1000, [S, T, RH] = hils_setpoint_step(S, p, 50, 99, 1); end
check(T == p.T_max && RH == p.RH_max, 'limits');
end

function t_params_const_in_sync()
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

function t_closed_loop_winter()
P = hils_params([], 'winter');
L = hils_run_offline(P, 3600);
v = L.valid > 0;
check(mean(v) > 0.85 && ~any(L.state == 5), 'reaches RUN, no SAFE_STOP');
check(any(L.HP_status == 1) && all(L.Q_lat(L.HP_status == 1) < 1e-6 + 50), 'heating, sensible only');
check(sqrt(mean((L.T_return(v) - L.T_sp(v)).^2)) < 0.1, 'chamber tracks zone');
end

function t_closed_loop_summer()
P = hils_params([], 'summer');
L = hils_run_offline(P, 2400);
check(any(L.HP_status == 2), 'cooling');
check(~any(L.state == 5), 'no SAFE_STOP');
check(mean(L.T_z(end-300:end)) < 27, 'zone cooled');
end

function t_io_emulator_equivalence()
P = hils_params();
L1 = hils_run_offline(P, 900);
L2 = hils_run_realtime(P, 900, hils_io_emulator(P), struct('pacing', 0));
check(isequal(L1.T_sp, L2.T_sp) && isequal(L1.T_return, L2.T_return) && isequal(L1.T_z, L2.T_z), ...
      'io path equal');
end

function t_estop()
P = hils_params();
io = hils_io_emulator(P);
ev = {300, @(io) io.set('estop', true)};
L = hils_run_realtime(P, 600, io, struct('pacing', 0, 'events', {ev}));
check(L.state(end) == 5 && any(bitand(L.fault, 8)), 'estop -> SAFE_STOP');
end

function t_simulink_block_scripts()
% build_HILS_Controller 가 MATLAB Function 블록에 넣는 스크립트를 내보내 Simulink 와 같은 배선
% (Core -> Encode -> PLC_Emulator -> Unit Delay -> Core) 으로 실행, 검증된 hils_run_offline 과 비교.
d = tempname; mkdir(d);
here = fileparts(mfilename('fullpath'));
build_HILS_Controller('ExportScripts', d);
addpath(d);
clear HILS_Core_1s PLC_Emulator
P = hils_params();
E0 = hils_plc_emulator_init(P);
meas = E0.meas_words; n = 1201; Tsp = zeros(n, 1); Tz = zeros(n, 1);
for i = 1:n
    t = i - 1;
    [cmd, st, air, zone] = HILS_Core_1s(meas, t, [0; 0; 0; 0], 0);
    w = Encode_Cmd(cmd);
    meas = PLC_Emulator(w);                      % Unit Delay: 다음 스텝 입력
    y = Decode_Meas(meas);
    Tsp(i) = cmd(1); Tz(i) = zone(1);
end
rmpath(d); clear HILS_Core_1s PLC_Emulator
L = hils_run_offline(P, 1200);
check(max(abs(Tsp - L.T_sp)) < 1e-12 && max(abs(Tz - L.T_z)) < 1e-12, 'block scripts == offline');
check(numel(y) == 15 && numel(st) == 10 && numel(air) == 4, 'port sizes');
end

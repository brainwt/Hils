function [E, meas_words] = hils_plc_emulator_step(E, P, cmd_words)
%HILS_PLC_EMULATOR_STEP  PLC 1 스캔(Ts_plc): 명령 읽기 -> 지연 FIFO -> 플랜트 적분 -> 측정 쓰기.
%   Python hils.emulator.PLCEmulator.scan 과 동일 (측정 노이즈 제외 -> meas_noise_W=0 과 비교).
e = P.emulator;
% watchdog: Simulink heartbeat 가 멈추면 PLC 가 스스로 부하장치 해제 (Simulink 안전로직과 이중화)
if cmd_words(5) ~= E.wd_last
    E.wd_last = cmd_words(5); E.wd_age = 0;
else
    E.wd_age = E.wd_age + P.timing.Ts_plc;
end
E.watchdog = E.wd_age > e.plc_watchdog_s;
% FIFO(ring buffer): 현재 프레임 push, 길이가 delay_steps 를 넘으면 가장 오래된 프레임 적용
N = size(E.fifo, 2);
E.fifo(:, mod(E.fifo_head + E.fifo_n, N) + 1) = cmd_words(:);
E.fifo_n = E.fifo_n + 1;
if E.fifo_n > E.delay_steps
    E.applied = hils_decode_cmd(E.fifo(:, E.fifo_head + 1), P);
    E.fifo_head = mod(E.fifo_head + 1, N);
    E.fifo_n = E.fifo_n - 1;
end
if E.estop, E.applied.Enable = 0; end
a = E.applied;
if E.watchdog, a.Enable = 0; end
dt = P.timing.Ts_plant_internal;
for k = 1:round(P.timing.Ts_plc / dt)
    E = plant_step(E, e, dt, a);
end
if ~E.freeze_heartbeat
    E.hb = mod(E.hb + 1, 65536);
end
bits = P.plc_status_bits;
st = 2^bits.READY;
if E.estop, st = st + 2^bits.ESTOP; end
if E.applied.Enable && ~E.watchdog, st = st + 2^bits.LOAD_ACTIVE; end
if E.watchdog, st = st + 2^bits.WATCHDOG; end
m = struct('T_indoor', E.T, 'RH_indoor', 40, 'T_outdoor', E.T_out, 'P_HP', E.hp.P, ...
           'Q_HP', E.hp.Q, 'HP_status', E.hp.status, 'Q_load_meas', E.Q_load, ...
           'AckSequence', E.applied.Sequence, 'PLC_status', st, 'PLC_heartbeat', E.hb);
meas_words = hils_encode_meas(m, P);
E.meas_words = meas_words;
end

function E = plant_step(E, e, dt, a)
% 부하장치 (1차 지연 + 이득/바이어스 오차)
if a.Enable
    target = e.load_gain_error * a.Q_load_cmd + e.load_bias_W;
else
    target = min(max(-800 * (a.T_chamber_SP - E.T), -4000), 4000);   % 사전조화
end
E.Q_load = E.Q_load + dt / e.load_tau_s * (target - E.Q_load);
E.T_out = E.T_out + dt / 300 * (a.T_outdoor_SP - E.T_out);
E.hp = heat_pump(E.hp, e, dt, E.T, E.T_out);
dT = (E.hp.Q - E.Q_load + e.UA_chamber_lab * (e.T_lab - E.T)) / e.C_chamber;
E.T = E.T + dt * dT;
end

function hp = heat_pump(hp, e, dt, T_room, T_out)
% 히트펌프 native 제어기: PI + 최소 off 시간 + 히스테리시스 (Simulink 는 개입하지 않음)
err = e.hp_setpoint - T_room;
if ~hp.on
    hp.off_timer = hp.off_timer + dt;
    if err > e.hp_hyst && hp.off_timer >= e.hp_min_off_s
        hp.on = true; hp.integ = e.hp_Qmin;
    end
end
demand = 0;
if hp.on
    hp.integ = min(max(hp.integ + e.hp_Kp / e.hp_Ti_s * err * dt, 0), e.hp_Qmax);
    demand = min(max(e.hp_Kp * err + hp.integ, e.hp_Qmin), e.hp_Qmax);
    if err < -e.hp_hyst && hp.integ <= e.hp_Qmin
        hp.on = false; hp.off_timer = 0; demand = 0;
    end
end
if hp.on, hp.status = 1; else, hp.status = 0; end
hp.Q = hp.Q + dt / e.hp_tau_s * (demand - hp.Q);
T_cond = T_room + 10 + 273.15; T_evap = T_out - 5 + 273.15;
cop = max(1, e.hp_carnot_eff * T_cond / max(T_cond - T_evap, 5));
if hp.on, aux = 30; else, aux = 5; end
hp.P = hp.Q / cop + aux;
end

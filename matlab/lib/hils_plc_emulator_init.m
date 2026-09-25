function E = hils_plc_emulator_init(P)
%HILS_PLC_EMULATOR_INIT  가짜 PLC + 챔버 + 히트펌프(자체제어) 상태. 실물이 없을 때 사용.
e = P.emulator;
E.T = 20; E.T_out = 0; E.Q_load = 0;
E.hp = struct('Q', 0, 'integ', 0, 'on', false, 'off_timer', 1e9, 'status', 0, 'P', 0);
E.fifo = zeros(6, P.emulator.fifo_max);  % 명령 프레임 FIFO (열 = 프레임)
E.fifo_n = 0;                            % 저장된 프레임 수
E.fifo_head = 0;                         % 가장 오래된 프레임 위치(0-based)
E.delay_steps = min(round(e.ack_delay_s / P.timing.Ts_plc), P.emulator.fifo_max - 1);
E.applied = struct('Q_load_cmd', 0, 'T_chamber_SP', e.hp_setpoint, 'Enable', 0, ...
                   'Sequence', 0, 'SIM_heartbeat', 0, 'T_outdoor_SP', 0);
E.hb = 0;
E.estop = false;
E.freeze_heartbeat = false;
E.wd_last = -1; E.wd_age = 0; E.watchdog = false;   % PLC 측 SIM_heartbeat watchdog
E.meas_words = hils_encode_meas(plc_measure(E, P), P);
end

function m = plc_measure(E, P)
bits = P.plc_status_bits;
st = 2^bits.READY;
if E.estop, st = st + 2^bits.ESTOP; end
if E.applied.Enable && ~E.watchdog, st = st + 2^bits.LOAD_ACTIVE; end
if E.watchdog, st = st + 2^bits.WATCHDOG; end
m = struct('T_indoor', E.T, 'RH_indoor', 40, 'T_outdoor', E.T_out, 'P_HP', E.hp.P, ...
           'Q_HP', E.hp.Q, 'HP_status', E.hp.status, 'Q_load_meas', E.Q_load, ...
           'AckSequence', E.applied.Sequence, 'PLC_status', st, 'PLC_heartbeat', E.hb);
end

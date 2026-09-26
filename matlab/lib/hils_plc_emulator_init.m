function E = hils_plc_emulator_init(P)
%HILS_PLC_EMULATOR_INIT  가짜 PLC + 실내측 챔버(국부 PI) + 히트펌프 실내기 + 실외측 챔버.
e = P.emulator;
E.P = e.P_atm;
E.T = e.T0; E.W = hils_psy_w(e.T0, e.RH0, E.P);
E.T_out = 0; E.RH_out = 50;
E.Q_cond = 0; E.m_hum = 0; E.iT = 0; E.iW = 0;
if e.hp_cool, hset = e.hp_setpoint_cool; else, hset = e.hp_setpoint_heat; end
E.hp = struct('set', hset, 'Q', 0, 'integ', 0, 'on', false, 'off_timer', 1e9, 'status', 0, ...
              'P', 0, 'fault', false);
E.T_s = E.T; E.W_s = E.W; E.m_da = 0;
E.fifo = zeros(7, e.fifo_max); E.fifo_n = 0; E.fifo_head = 0;
E.delay_steps = min(round(e.ack_delay_s / P.timing.Ts_plc), e.fifo_max - 1);
E.applied = struct('T_room_SP', e.T0, 'RH_room_SP', e.RH0, 'T_outdoor_SP', 0, ...
                   'RH_outdoor_SP', 50, 'Enable', 0, 'Sequence', 0, 'SIM_heartbeat', 0);
E.hold = E.applied;
E.hb = 0; E.estop = false; E.freeze_heartbeat = false;
E.wd_last = -1; E.wd_age = 0; E.watchdog = false;
E.meas_words = hils_encode_meas(hils_plc_measure(E, P), P);
end

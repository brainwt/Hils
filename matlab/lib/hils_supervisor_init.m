function S = hils_supervisor_init(P)
%HILS_SUPERVISOR_INIT  supervisor 전체 상태 초기화.
S.sm = struct('state', 0, 'timer', 0, 'ok_timer', 0, 'warning', 0);
S.dm = hils_delay_monitor_init(P.delay_history);
S.hb = struct('last', -1, 'age', 0);
S.pi = struct('integ', 0, 'u_prev', 0);
S.seq = 0;
S.Q_target = 0;
S.T_target = P.emulator.hp_setpoint;
S.T_out_target = 0;
S.new_step = false;
S.sim_hb = 0;
S.send = false;
end

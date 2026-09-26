function S = hils_supervisor_init(P)
%HILS_SUPERVISOR_INIT  supervisor 전체 상태.
S.sm = struct('state', 0, 'timer', 0, 'ok_timer', 0, 'warning', 0, 'restarts', 0, 'latched', false);
S.dm = hils_delay_monitor_init(P.delay_history);
S.hb = struct('last', -1, 'age', 0);
S.tw_age = 0;
S.sp = struct('T', P.zone.T0, 'RH', P.zone.RH0);
S.acc = struct('n', 0, 'Q_sens', 0, 'Q_lat', 0, 'Q_tot', 0, 'm_w', 0);
S.seq = 0;
S.T_target = P.zone.T0; S.RH_target = P.zone.RH0;
S.T_out_target = 0; S.RH_out_target = 50;
S.new_step = false; S.sim_hb = 0; S.send = false;
end

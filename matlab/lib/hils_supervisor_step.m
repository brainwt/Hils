function [S, cmd, st] = hils_supervisor_step(S, P, t, m)
%HILS_SUPERVISOR_STEP  1 s 층: 측정 m -> heartbeat/지연/안전/상태머신 -> PI -> 명령 cmd.
%   Python hils.supervisor.HilsSupervisor.realization_step 과 동일.
Ts = P.timing.Ts_realization;
[S.hb, hb_lost] = hils_heartbeat_step(S.hb, m.PLC_heartbeat, Ts, P.safety.heartbeat_timeout_s);
[S.dm, pending, ack_to, age] = hils_delay_monitor_step(S.dm, S.send, S.seq, S.Q_target, ...
    m.AckSequence, t, P.safety.ack_timeout_s);
S.send = false;
fault = hils_safety_check(m, P.safety, P.plc_status_bits, P.hp_status_codes, hb_lost, ack_to);
plc_ready = bitand(round(m.PLC_status), 2^P.plc_status_bits.READY) > 0;
if S.dm.has_ack
    q_ref = S.dm.q_ack;
else
    q_ref = S.Q_target;
end
err_abs = abs(q_ref - m.Q_load_meas);
S.sm = hils_state_machine_step(S.sm, P.state_machine, Ts, plc_ready, fault ~= 0, err_abs, S.new_step);
S.new_step = false;
enable = S.sm.state == 2 || S.sm.state == 3 || S.sm.state == 4;
[kp, ki] = hils_delay_gains(P.realization_controller, S.dm.last_latency);
[u, S.pi, e, limited] = hils_pi_step(S.pi, S.Q_target, m.Q_load_meas, q_ref, ...
    P.realization_controller, Ts, enable, ~S.dm.has_ack, kp, ki);
S.sim_hb = mod(S.sim_hb + 1, 65536);
cmd.Q_load_cmd = u;
cmd.T_chamber_SP = S.T_target;
cmd.Enable = double(enable);
cmd.Sequence = S.seq;
cmd.SIM_heartbeat = S.sim_hb;
cmd.T_outdoor_SP = S.T_out_target;
st.state = S.sm.state; st.fault = fault; st.pending = pending; st.ack_age = age;
st.latency = S.dm.last_latency; st.e = e; st.limited = limited;
st.valid = S.sm.state == 3; st.q_ref = q_ref; st.kp = kp; st.ki = ki; st.integ = S.pi.integ;
end

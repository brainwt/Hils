function [S, cmd, st] = hils_supervisor_step(S, P, t, m)
%HILS_SUPERVISOR_STEP  1 s 층 (Python HilsSupervisor.realization_step 과 동일).
%   측정 -> air-enthalpy 열량 누적 -> heartbeat/지연/추종/안전 -> 상태머신 -> 설정값 변화율 제한
Ts = P.timing.Ts_realization;
a = hils_air_enthalpy(m.T_supply, m.RH_supply, m.T_return, m.RH_return, m.V_air, m.P_atm);
S.acc.n = S.acc.n + 1; S.acc.Q_sens = S.acc.Q_sens + a.Q_sens; S.acc.Q_lat = S.acc.Q_lat + a.Q_lat;
S.acc.Q_tot = S.acc.Q_tot + a.Q_tot; S.acc.m_w = S.acc.m_w + a.m_w;

[S.hb, hb_lost] = hils_heartbeat_step(S.hb, m.PLC_heartbeat, Ts, P.safety.heartbeat_timeout_s);
[S.dm, pending, ack_to, age] = hils_delay_monitor_step(S.dm, S.send, S.seq, S.T_target, ...
    S.RH_target, m.AckSequence, t, P.safety.ack_timeout_s);
S.send = false;
if S.dm.has_ack
    T_ref = S.dm.T_ack; RH_ref = S.dm.RH_ack;
else
    T_ref = S.sp.T; RH_ref = S.sp.RH;
end
eT = m.T_return - T_ref; eRH = m.RH_return - RH_ref;
tracking_ok = abs(eT) < P.state_machine.track_tol_T_K && abs(eRH) < P.state_machine.track_tol_RH_pct;
if S.sm.state == 3 && abs(eT) > P.safety.track_fault_T_K
    S.tw_age = S.tw_age + Ts;
else
    S.tw_age = 0;
end
track_lost = S.tw_age > P.safety.track_fault_hold_s;
fault = hils_safety_check(m, P.safety, P.plc_status_bits, P.hp_status_codes, hb_lost, ack_to, track_lost);
plc_ready = bitand(round(m.PLC_status), 2^P.plc_status_bits.READY) > 0;
S.sm = hils_state_machine_step(S.sm, P.state_machine, Ts, plc_ready, fault ~= 0, tracking_ok, S.new_step);
S.new_step = false;
enable = S.sm.state == 2 || S.sm.state == 3 || S.sm.state == 4;
[S.sp, T_sp, RH_sp] = hils_setpoint_step(S.sp, P.setpoint, S.T_target, S.RH_target, Ts);
S.sim_hb = mod(S.sim_hb + 1, 65536);
cmd.T_room_SP = T_sp; cmd.RH_room_SP = RH_sp;
cmd.T_outdoor_SP = S.T_out_target; cmd.RH_outdoor_SP = S.RH_out_target;
cmd.Enable = double(enable); cmd.Sequence = S.seq; cmd.SIM_heartbeat = S.sim_hb;
st.state = S.sm.state; st.fault = fault; st.pending = pending; st.ack_age = age;
st.latency = S.dm.last_latency; st.valid = S.sm.state == 3; st.eT = eT; st.eRH = eRH;
st.T_ref = T_ref; st.RH_ref = RH_ref;
st.Q_sens = a.Q_sens; st.Q_lat = a.Q_lat; st.Q_tot = a.Q_tot; st.m_w = a.m_w;
end

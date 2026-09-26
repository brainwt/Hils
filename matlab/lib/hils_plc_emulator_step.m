function [E, meas_words] = hils_plc_emulator_step(E, P, cmd_words)
%HILS_PLC_EMULATOR_STEP  PLC 1 스캔 (Python PLCEmulator.scan 과 동일, 노이즈 0 기준).
%   명령 읽기 -> watchdog -> 지연 FIFO -> (Enable 이면 hold 갱신) -> 플랜트 적분 -> 측정 쓰기
e = P.emulator;
if cmd_words(7) ~= E.wd_last
    E.wd_last = cmd_words(7); E.wd_age = 0;
else
    E.wd_age = E.wd_age + P.timing.Ts_plc;
end
E.watchdog = E.wd_age > e.plc_watchdog_s;
N = size(E.fifo, 2);
E.fifo(:, mod(E.fifo_head + E.fifo_n, N) + 1) = cmd_words(:);
E.fifo_n = E.fifo_n + 1;
if E.fifo_n > E.delay_steps
    E.applied = hils_decode_cmd(E.fifo(:, E.fifo_head + 1), P);
    E.fifo_head = mod(E.fifo_head + 1, N);
    E.fifo_n = E.fifo_n - 1;
end
if E.estop, E.applied.Enable = 0; end
if E.applied.Enable && ~E.watchdog
    E.hold = E.applied;
end
dt = P.timing.Ts_plant_internal;
for k = 1:round(P.timing.Ts_plc / dt)
    E = plant_step(E, e, dt, E.hold);
end
if ~E.freeze_heartbeat
    E.hb = mod(E.hb + 1, 65536);
end
meas_words = hils_encode_meas(hils_plc_measure(E, P), P);
E.meas_words = meas_words;
end

function E = plant_step(E, e, dt, sp)
Pa = E.P;
W_sp = hils_psy_w(sp.T_room_SP, sp.RH_room_SP, Pa);
% PLC 국부 PI (챔버 공조: 가열/냉각, 가습/제습)
eT = sp.T_room_SP - E.T;
E.iT = min(max(E.iT + e.pid_T_Kp / e.pid_T_Ti * eT * dt, -e.Q_cond_max), e.Q_cond_max);
uT = min(max(e.pid_T_Kp * eT + E.iT, -e.Q_cond_max), e.Q_cond_max);
eW = (W_sp - E.W) * 1000;
kW = e.pid_W_Kp / 1000;
E.iW = min(max(E.iW + kW / e.pid_W_Ti * eW * dt, -e.m_hum_max), e.m_hum_max);
uW = min(max(kW * eW + E.iW, -e.m_hum_max), e.m_hum_max);
E.Q_cond = E.Q_cond + dt / e.act_tau_s * (uT - E.Q_cond);
E.m_hum = E.m_hum + dt / e.act_tau_s * (uW - E.m_hum);
% 실외측 챔버
E.T_out = E.T_out + dt / e.outdoor_tau_s * (sp.T_outdoor_SP - E.T_out);
E.RH_out = E.RH_out + dt / e.outdoor_tau_s * (sp.RH_outdoor_SP - E.RH_out);
% 히트펌프 실내기 (native 제어, 리턴 = 챔버 공기)
E.hp = heat_pump(E.hp, e, dt, E.T, E.T_out);
if E.hp.on, V = e.hp_airflow_m3h; else, V = 0; end
W_r = E.W;
if V > 0, m_da = V / 3600 / hils_psy_v(E.T, W_r, Pa); else, m_da = 0; end
[T_s, W_s] = hils_coil_outlet(E.T, W_r, E.hp.Q, m_da, e.hp_cool, e.hp_bypass_factor, Pa);
E.T_s = T_s; E.W_s = W_s; E.m_da = m_da;
dT = (1000 * m_da * (1.006 + 1.86 * W_r) * (T_s - E.T) + E.Q_cond ...
      + e.UA_chamber_lab * (e.T_lab - E.T)) / e.C_chamber;
dW = (m_da * (W_s - E.W) + E.m_hum) / e.M_chamber_air;
E.T = E.T + dt * dT;
E.W = max(E.W + dt * dW, 1e-5);
if e.ideal_chamber
    E.T = sp.T_room_SP; E.W = W_sp;
end
end

function hp = heat_pump(hp, e, dt, T_r, T_out)
if e.hp_cool, err = T_r - hp.set; else, err = hp.set - T_r; end
demand = 0;
if hp.fault
    hp.on = false;
else
    if ~hp.on
        hp.off_timer = hp.off_timer + dt;
        if err > e.hp_hyst && hp.off_timer >= e.hp_min_off_s
            hp.on = true; hp.integ = e.hp_Qmin;
        end
    end
    if hp.on
        hp.integ = min(max(hp.integ + e.hp_Kp / e.hp_Ti_s * err * dt, 0), e.hp_Qmax);
        demand = min(max(e.hp_Kp * err + hp.integ, e.hp_Qmin), e.hp_Qmax);
        if err < -e.hp_hyst && hp.integ <= e.hp_Qmin
            hp.on = false; hp.off_timer = 0; demand = 0;
        end
    end
end
if hp.fault
    hp.status = 9;
elseif hp.on
    if e.hp_cool, hp.status = 2; else, hp.status = 1; end
else
    hp.status = 0;
end
hp.Q = hp.Q + dt / e.hp_tau_s * (demand - hp.Q);
if e.hp_cool
    T_hi = T_out + 10 + 273.15; T_lo = T_r - 10 + 273.15;
    cop = max(1, e.hp_carnot_eff * T_lo / max(T_hi - T_lo, 5));
else
    T_hi = T_r + 10 + 273.15; T_lo = T_out - 5 + 273.15;
    cop = max(1, e.hp_carnot_eff * T_hi / max(T_hi - T_lo, 5));
end
if hp.on, aux = e.hp_fan_W; else, aux = 5; end
hp.P = hp.Q / cop + aux;
end

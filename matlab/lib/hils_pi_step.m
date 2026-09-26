function [u, S, e, limited] = hils_pi_step(S, q_target, q_meas, q_ref, p, Ts, enable, hold, kp, ki)
%HILS_PI_STEP  부하재현 제어기: feedforward + PI -> rate limiter -> saturation.
%   S       : struct(integ, u_prev)
%   q_target: 피드포워드용 최신 목표,  q_ref: 오차 기준(ack 된 프레임의 목표)
%   hold    : true 이면 적분 정지,     kp, ki: 지연보상 스케줄 게인
%   Anti-windup: 출력이 제한되고 오차가 같은 방향이면 적분하지 않음(조건부 적분).
dmax = p.rate_limit_W_per_s * Ts;
if ~enable
    u = min(max(0, S.u_prev - dmax), S.u_prev + dmax);
    u = min(max(u, p.Q_cmd_min), p.Q_cmd_max);
    S.integ = 0; S.u_prev = u; e = 0; limited = false;
    return;
end
e = q_ref - q_meas;
if hold
    integ_new = S.integ;
else
    integ_new = S.integ + ki * e * Ts;
end
u_raw = p.feedforward_gain * q_target + kp * e + integ_new;
u = min(max(u_raw, S.u_prev - dmax), S.u_prev + dmax);
u = min(max(u, p.Q_cmd_min), p.Q_cmd_max);
limited = (u ~= u_raw);
if limited && (u_raw - u) * e > 0
    integ_new = S.integ;
end
S.integ = integ_new;
S.u_prev = u;
end

function [kp, ki] = hils_delay_gains(p, theta)
%HILS_DELAY_GAINS  측정 루프지연 theta[s] 에 따른 SIMC PI 게인 (설정 게인이 상한).
%   G = K e^(-theta s)/(tau s+1), tau_c = theta:
%   Kp = tau/(K*2*theta),  Ti = min(tau, 8*theta),  Ki = Kp/Ti
kp = p.Kp; ki = p.Ki;
if ~p.delay_compensation || isnan(theta) || theta <= 0
    return;
end
kp = min(p.Kp, p.plant_tau_s / (p.plant_gain * 2 * theta));
ti = min(p.plant_tau_s, 8 * theta);
ki = min(p.Ki, kp / ti);
end

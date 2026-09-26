function [S, q] = hils_supervisor_interval(S)
%HILS_SUPERVISOR_INTERVAL  직전 건물 구간의 평균 air-enthalpy 열량을 꺼내고 누적기 초기화.
n = max(S.acc.n, 1);
q.Q_sens = S.acc.Q_sens / n; q.Q_lat = S.acc.Q_lat / n; q.Q_tot = S.acc.Q_tot / n;
q.m_w = S.acc.m_w / n; q.n = S.acc.n;
S.acc = struct('n', 0, 'Q_sens', 0, 'Q_lat', 0, 'Q_tot', 0, 'm_w', 0);
end

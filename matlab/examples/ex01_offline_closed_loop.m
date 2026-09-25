%EX01_OFFLINE_CLOSED_LOOP  MATLAB/Octave 전용 폐루프 (Simulink 불필요).
%   가상건물(2R2C) -> supervisor -> 레지스터 encode -> 가짜 PLC -> decode, 24 h.
%   cd matlab/examples; ex01_offline_closed_loop
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
P = hils_params();
tic; L = hils_run_offline(P, 24 * 3600); fprintf('elapsed %.1f s\n', toc);
v = L.valid > 0; e = L.Q_ref - L.Q_load_meas;
fprintf('valid ratio      %.3f\n', mean(v));
fprintf('RMSE (RUN)       %.1f W\n', sqrt(mean(e(v).^2)));
fprintf('energy target    %.2f kWh\n', sum(L.Q_target) / 3.6e6);
fprintf('energy realized  %.2f kWh\n', sum(L.Q_load_meas) / 3.6e6);
fprintf('HP heat / elec   %.2f / %.2f kWh\n', sum(L.Q_HP) / 3.6e6, sum(L.P_HP) / 3.6e6);
fprintf('T_indoor mean    %.2f degC\n', mean(L.T_indoor));
try   % 헤드리스 Octave(그래픽 툴킷 없음)에서는 그림 생략
    h = L.t / 3600;
    figure('Visible', 'off');
    subplot(3, 1, 1); plot(h, L.Q_target / 1e3, h, L.Q_load_meas / 1e3, h, L.Q_HP / 1e3);
    ylabel('kW'); legend('Q\_target', 'Q\_load\_meas', 'Q\_HP'); grid on;
    subplot(3, 1, 2); plot(h, L.T_indoor); ylabel('T\_indoor [degC]'); grid on;
    subplot(3, 1, 3); stairs(h, L.state); ylabel('state'); xlabel('time [h]'); grid on;
    print(fullfile(here, '..', '..', 'results', 'matlab_ex01_24h.png'), '-dpng', '-r90');
catch err
    fprintf('plot skipped: %s\n', err.message);
end

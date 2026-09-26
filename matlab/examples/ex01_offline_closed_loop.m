%EX01_OFFLINE_CLOSED_LOOP  MATLAB/Octave 전용 폐루프 (Simulink 불필요). 겨울 2 h + 여름 2 h.
%   air-enthalpy 측정 -> 가상 존 -> 존 상태 -> 챔버 설정값 -> 가짜 PLC/챔버/히트펌프
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
for season = {'winter', 'summer'}
    P = hils_params([], season{1});
    tic; L = hils_run_offline(P, 7200); el = toc;
    v = L.valid > 0; e = L.T_return - L.T_sp;
    fprintf('[%s] %.0f s  RUN %.3f  track RMSE %.3f K  T_z %.2f degC  RH_z %.1f %%  Q_sens %.0f W  Q_lat %.0f W\n', ...
            season{1}, el, mean(v), sqrt(mean(e(v).^2)), mean(L.T_z(end-600:end)), ...
            mean(L.RH_z(end-600:end)), mean(L.Q_sens(end-600:end)), mean(L.Q_lat(end-600:end)));
end

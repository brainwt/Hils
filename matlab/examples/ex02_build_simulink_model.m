%EX02_BUILD_SIMULINK_MODEL  HILS_Controller.slx 자동 생성 + 에뮬레이터 모드 2 h 실행.  [MATLAB+Simulink]
%   Simulink 결과를 hils_run_offline (Octave·Python 교차검증 완료) 과 비교해 모델 배선을 확인.
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
addpath(fullfile(root, 'lib')); addpath(root);
season = 'winter';
P = hils_params([], season);
mdl = build_HILS_Controller('Season', season, 'PlcIo', 'emulator', 'StopTime', '7200');
open_system(mdl);
out = sim(mdl);                                   % 검증용 1 회 실행 (본운전은 hils_master)
L = hils_run_offline(P, 7200);
cmd = out.hils_cmd.signals.values;                % [T_sp RH_sp Tout_sp RHout_sp Enable Seq hb]
zone = out.hils_zone.signals.values;              % [T_z RH_z T_out RH_out Q_sens_int m_w_int]
n = min(size(cmd, 1), numel(L.T_sp));
fprintf('max |T_sp(Simulink) - T_sp(script)| = %.3g K\n', max(abs(cmd(1:n, 1) - L.T_sp(1:n))));
fprintf('max |T_z (Simulink) - T_z (script)| = %.3g K\n', max(abs(zone(1:n, 1) - L.T_z(1:n))));

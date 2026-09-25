%EX02_BUILD_SIMULINK_MODEL  HILS_Controller.slx 자동 생성 + 오프라인(에뮬레이터) 2 h 실행.  [MATLAB+Simulink]
%   Octave 에서는 실행 불가. 결과를 hils_run_offline (Octave 검증 완료) 과 비교해 모델 배선 확인.
here = fileparts(mfilename('fullpath'));
root = fullfile(here, '..');
addpath(fullfile(root, 'lib')); addpath(root);
P = hils_params();

mdl = build_HILS_Controller('PlcIo', 'emulator', 'StopTime', '7200');
open_system(mdl);
out = sim(mdl);                                   % 1 회성 검증 실행 (본운전은 hils_master)

% Simulink 결과 vs 스크립트 결과: 같은 lib 함수 -> 동일해야 한다
L = hils_run_offline(P, 7200);
cmd = out.hils_cmd.signals.values;                % [Q_cmd T_sp Enable Seq SIM_hb Tout_sp]
st  = out.hils_status.signals.values;             % [state fault pending ack_age latency e valid q_ref kp integ]
n = min(size(cmd, 1), numel(L.Q_load_cmd));
fprintf('max |Q_cmd(Simulink) - Q_cmd(script)| = %.3g W\n', max(abs(cmd(1:n, 1) - L.Q_load_cmd(1:n))));
fprintf('state mismatch samples = %d\n', sum(st(1:n, 1) ~= L.state(1:n)));

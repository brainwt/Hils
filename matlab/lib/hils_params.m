function P = hils_params(cfgFile)
%HILS_PARAMS  config/hils_config.json 을 읽어 파라미터 구조체로 반환.
%   P = hils_params()            기본 설정 파일
%   P = hils_params(cfgFile)     지정 설정 파일
%   Python(hils.config) 과 같은 파일을 쓰므로 두 구현의 파라미터가 항상 일치한다.
if nargin < 1 || isempty(cfgFile)
    here = fileparts(mfilename('fullpath'));
    cfgFile = fullfile(here, '..', '..', 'config', 'hils_config.json');
end
txt = fileread(cfgFile);
P = jsondecode(txt);
% PLC 에뮬레이터 FIFO 길이 (codegen 용 고정 상한)
P.emulator.fifo_max = 600;
P.delay_history = 64;
end

function P = hils_params(cfgFile, season)
%HILS_PARAMS  config/hils_config.json -> 파라미터 구조체 (+ 계절 설정).
%   P = hils_params()                 기본 파일, 겨울(난방)
%   P = hils_params([], 'summer')     여름(냉방) 초기조건·히트펌프 모드
%   계절 설정은 Python hils.cosim.season_overrides 와 같다.
if nargin < 1 || isempty(cfgFile)
    here = fileparts(mfilename('fullpath'));
    cfgFile = fullfile(here, '..', '..', 'config', 'hils_config.json');
end
if nargin < 2 || isempty(season), season = 'winter'; end
P = jsondecode(fileread(cfgFile));
switch season
    case 'winter'
        P.zone.T0 = 20; P.zone.RH0 = 40; P.zone.Tm0 = 18; P.emulator.hp_mode = 'heat';
        P.wx = P.weather.winter;
    case 'summer'
        P.zone.T0 = 27; P.zone.RH0 = 60; P.zone.Tm0 = 28; P.emulator.hp_mode = 'cool';
        P.wx = P.weather.summer;
    otherwise
        error('HILS:season', 'season must be winter or summer');
end
P.emulator.T0 = P.zone.T0; P.emulator.RH0 = P.zone.RH0;
P.emulator.hp_cool = strcmp(P.emulator.hp_mode, 'cool');   % codegen 용 숫자 플래그
P.season_cool = double(strcmp(season, 'summer'));
P.emulator.fifo_max = 600;
P.delay_history = 64;
end

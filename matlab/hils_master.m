function out = hils_master(varargin)
%HILS_MASTER  MATLAB 이 co-simulation master 로 Simulink HILS 모델을 운전 (Simulation 객체).
%
%   out = hils_master('Mode', 'internal')   % 내장 가상 존 사용, 모델을 끝까지 연속 실행
%   out = hils_master('Mode', 'cosim', 'ZoneFcn', @my_energyplus_step)
%         % 60 s 마다 step(sm,'PauseTime',t) 로 정지 -> 외부 존 모델 계산 -> setVariable -> 재개
%
%   ZoneFcn : @(k, t, q) -> zs   (EnergyPlus/FMU 연동 지점)
%             입력 q  : 직전 60 s 구간 평균 air-enthalpy 측정 (q.Q_sens [W], q.Q_lat [W], q.m_w [kg/s])
%             출력 zs : 다음 시각 존 공기 상태 (zs.T_z, zs.RH_z, zs.T_out, zs.RH_out)
%             기본값: 내장 2R2C 가상 존 (hils_zone_step)
%   QFcn    : @(sm) -> q  정지 중 직전 구간 열량을 얻는 방법.
%             기본값: 모델 로그 hils_zone(5:6) 의 마지막 값 (릴리스별 접근 방법 차이로 fallback 포함)
%   Season, Duration, Model 옵션.
%
%   필요: MATLAB R2024a+ (Simulation 객체). 이 저장소 환경에는 MATLAB 이 없어 실행 검증되지 않았다.
%   같은 순서의 로직은 hils_run_offline / hils_run_realtime 로 Octave 에서 검증됨.
%
%   매 60 s 마다 sim() 을 새로 부르지 않는다: 가상 존/지연 감시/상태머신 상태가 끊기지 않도록
%   하나의 simulation 을 이어서 진행하며 존 상태만 바꾼다.

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
o = struct('Mode', 'internal', 'Model', 'HILS_Controller', 'Season', 'winter', ...
           'Duration', 3600, 'ZoneFcn', [], 'QFcn', []);
for k = 1:2:numel(varargin)
    o.(varargin{k}) = varargin{k + 1};
end
P = hils_params([], o.Season);
Tb = P.timing.Ts_building;
mdl = o.Model;
if ~exist(fullfile(here, [mdl '.slx']), 'file')
    build_HILS_Controller('ModelName', mdl, 'Season', o.Season);
end
load_system(fullfile(here, [mdl '.slx']));

Z = hils_zone_init(P.zone, P.emulator.P_atm);
if isempty(o.ZoneFcn), o.ZoneFcn = @default_zone; end
if isempty(o.QFcn), o.QFcn = @default_q; end

sm = simulation(mdl);
sm = setModelParameter(sm, 'StopTime', num2str(o.Duration));
initialize(sm);
hist = zeros(0, 6);
switch lower(o.Mode)
    case 'internal'
        setVariable(sm, 'USE_EXTERNAL_ZONE', 0);
        start(sm);
        while ~strcmpi(get_status(sm), 'stopped') && ~strcmpi(get_status(sm), 'inactive')
            pause(1);
        end
        out = finish(sm);
    case 'cosim'
        setVariable(sm, 'USE_EXTERNAL_ZONE', 1);
        zs = hils_zone_state(Z, P.wx);
        push(zs);
        for k = 1:floor(o.Duration / Tb)
            tNext = k * Tb;
            step(sm, 'PauseTime', tNext - P.timing.Ts_realization);  % t=60k 직전에서 정지
            q = o.QFcn(sm);
            zs = o.ZoneFcn(k, tNext, q);
            push(zs);                                                 % t=60k 스텝에서 반영
            hist(end + 1, :) = [tNext, q.Q_sens, q.m_w, zs.T_z, zs.RH_z, zs.T_out]; %#ok<AGROW>
            fprintf('[%6.0f s] Q_sens=%7.1f W  -> T_z=%5.2f degC RH_z=%4.1f %%\n', ...
                    tNext, q.Q_sens, zs.T_z, zs.RH_z);
        end
        out = finish(sm);
    otherwise
        error('HILS:mode', 'Mode must be ''internal'' or ''cosim''');
end
assignin('base', 'hils_master_history', hist);   % [t Q_sens m_w T_z RH_z T_out]

    function push(zs)
        setVariable(sm, 'T_zone_ext', zs.T_z);
        setVariable(sm, 'RH_zone_ext', zs.RH_z);
        setVariable(sm, 'T_out_ext', zs.T_out);
        setVariable(sm, 'RH_out_ext', zs.RH_out);
    end

    function zs = default_zone(~, ~, q)
        [Z, zs] = hils_zone_step(Z, P.zone, P.wx, Tb, P.timing.Ts_zone_sub, q.Q_sens, q.m_w);
    end
end

function q = default_q(sm)
% 정지 중 로그된 air-enthalpy 1 s 값(hils_air)의 직전 60개 평균. 릴리스별 접근 경로가 달라 차례로 시도.
q = struct('Q_sens', 0, 'Q_lat', 0, 'm_w', 0);
v = [];
try
    so = sm.SimulationOutput;  v = so.hils_air.signals.values;
catch
    try
        v = evalin('base', 'hils_air.signals.values');
    catch
        warning('HILS:q', '로그된 hils_air 를 읽지 못함: QFcn 으로 열량 획득 방법을 지정하세요.');
    end
end
if ~isempty(v)
    n = min(60, size(v, 1));
    w = v(end - n + 1:end, :);
    q.Q_sens = mean(w(:, 1)); q.Q_lat = mean(w(:, 2)); q.m_w = mean(w(:, 4));
end
end

function s = get_status(sm)
try
    s = char(sm.Status);
catch
    s = 'stopped';
end
end

function o2 = finish(sm)
try
    o2 = stop(sm);
catch
    stop(sm); o2 = [];
end
if isempty(o2)
    try, o2 = sm.SimulationOutput; catch, o2 = []; end
end
end

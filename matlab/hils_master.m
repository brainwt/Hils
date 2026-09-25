function out = hils_master(varargin)
%HILS_MASTER  MATLAB 에서 Simulink HILS 모델 전체를 제어하는 master (계획서 7, 8 절).
%
%   out = hils_master('Mode', 'continuous')   % 모델이 계속 실행, 60 s 마다 목표 setVariable
%   out = hils_master('Mode', 'cosim')        % step(sm, PauseTime=...) 로 건물 timestep 마다 정지
%
%   옵션
%     'Model'        (기본 'HILS_Controller', 없으면 build_HILS_Controller 로 생성)
%     'Duration'     실험 시간 [s] (기본 3600)
%     'BuildingFcn'  @(k, t, T_indoor) -> [Q_target, T_target, T_out]
%                    EnergyPlus/FMU 연동 지점. 기본은 hils_building_step (2R2C)
%     'TindoorFcn'   @() -> T_indoor  (cosim 모드에서 정지 중 실측 존온도 획득)
%                    기본: Modbus 로 40001 직접 읽기(modbus 모드) / NaN 이면 직전 목표 유지
%
%   필요: MATLAB R2024a+ (Simulation 객체: initialize/start/step/pause/resume/stop/setVariable)
%
%   주의: 매 60 s 마다 sim() 을 새로 호출하지 않는다. PI 적분기/필터/상태머신/지연 상태가
%   끊기지 않도록 하나의 simulation 을 계속 진행시키며 목표만 바꾼다 (계획서 10 절).

here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'lib'));
P = hils_params();
o = struct('Mode', 'cosim', 'Model', 'HILS_Controller', 'Duration', 3600, ...
           'BuildingFcn', [], 'TindoorFcn', []);
for k = 1:2:numel(varargin)
    o.(varargin{k}) = varargin{k + 1};
end
Tb = P.timing.Ts_building;
mdl = o.Model;
if ~exist(fullfile(here, [mdl '.slx']), 'file')
    build_HILS_Controller('ModelName', mdl);
end
load_system(fullfile(here, [mdl '.slx']));

% 기본 가상건물: 2R2C (EnergyPlus 자리)
B = struct('Tm', 18);
if isempty(o.BuildingFcn)
    o.BuildingFcn = @default_building;
end
if isempty(o.TindoorFcn)
    o.TindoorFcn = @() NaN;
end

sm = simulation(mdl);
sm = setModelParameter(sm, 'StopTime', num2str(o.Duration));
initialize(sm);
setVariable(sm, 'USE_EXTERNAL_TARGET', 1);      % 목표는 MATLAB master 가 공급
[Q, T, To] = o.BuildingFcn(0, 0, P.emulator.hp_setpoint);
setVariable(sm, 'Q_zone_target', Q);
setVariable(sm, 'T_zone_target', T);
setVariable(sm, 'T_out_target', To);
hist = zeros(0, 4);
T_last = P.emulator.hp_setpoint;

switch lower(o.Mode)
    case 'cosim'
        % 60 s 마다 정지 -> 건물 계산 -> 목표 갱신 -> 재개 (MATLAB 가 co-simulation master)
        nSteps = floor(o.Duration / Tb);
        for k = 1:nSteps
            tNext = k * Tb;
            step(sm, 'PauseTime', tNext);
            Ti = o.TindoorFcn();
            if isnan(Ti), Ti = T_last; end
            T_last = Ti;
            [Q, T, To] = o.BuildingFcn(k, tNext, Ti);
            setVariable(sm, 'Q_zone_target', Q);
            setVariable(sm, 'T_zone_target', T);
            setVariable(sm, 'T_out_target', To);
            hist(end + 1, :) = [tNext, Q, T, Ti]; %#ok<AGROW>
            fprintf('[%6.0f s] Q_target = %7.1f W  T_indoor = %5.2f degC\n', tNext, Q, Ti);
        end
        out = finish(sm);
    case 'continuous'
        % 모델은 (pacing 된) 실시간으로 계속 실행. master 는 wall-clock 60 s 마다 목표만 변경
        start(sm);
        t0 = tic; k = 0;
        while toc(t0) < o.Duration
            pause(Tb);
            k = k + 1;
            Ti = o.TindoorFcn();
            if isnan(Ti), Ti = T_last; end
            T_last = Ti;
            [Q, T, To] = o.BuildingFcn(k, k * Tb, Ti);
            setVariable(sm, 'Q_zone_target', Q);
            setVariable(sm, 'T_zone_target', T);
            setVariable(sm, 'T_out_target', To);
            hist(end + 1, :) = [k * Tb, Q, T, Ti]; %#ok<AGROW>
        end
        out = finish(sm);
    otherwise
        error('HILS:mode', 'Mode must be ''cosim'' or ''continuous''');
end
assignin('base', 'hils_master_targets', hist);

    function o2 = finish(sm_)
        % 릴리스에 따라 stop 이 출력을 돌려주거나 SimulationOutput 속성으로 제공
        try
            o2 = stop(sm_);
        catch
            stop(sm_); o2 = [];
        end
        if isempty(o2)
            try, o2 = sm_.SimulationOutput; catch, o2 = []; end
        end
    end

    function [Q, T, To] = default_building(~, t, T_indoor)
        [B, bo] = hils_building_step(B, P.virtual_building, Tb, t, T_indoor);
        Q = bo.Q_target; T = P.emulator.hp_setpoint; To = bo.T_out;
    end
end

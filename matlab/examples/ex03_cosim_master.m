%EX03_COSIM_MASTER  Simulation 객체로 건물 timestep(60 s) 마다 정지하는 co-simulation.  [MATLAB R2024a+]
%   계획서 8절: step(sm, PauseTime=tNext) -> 건물 계산 -> setVariable -> 재개.
%   EnergyPlus 를 쓰려면 BuildingFcn 을 EnergyPlus/FMU step 함수로 바꾼다.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
P = hils_params();

% 예: 가짜 PLC 가 Modbus 로 동작 중이면 실측 T_indoor 를 직접 읽는다
%   (터미널) cd python && python -m hils.plc_server --port 5020
useModbus = false;
if useModbus
    build_HILS_Controller('PlcIo', 'modbus', 'Host', '127.0.0.1', 'Port', 5020);
    io = hils_io_modbus(P, '127.0.0.1', 5020);
    tin = io.tindoor;
else
    build_HILS_Controller('PlcIo', 'emulator');
    tin = @() NaN;               % 에뮬레이터 모드: 직전 값 유지 (모델 내부 루프는 그대로 폐루프)
end

% EnergyPlus 자리: 여기서는 2R2C. 사용자 함수로 교체 가능
%   bf = @(k, t, Tin) my_energyplus_step(k, t, Tin);
out = hils_master('Mode', 'cosim', 'Duration', 3600, 'TindoorFcn', tin);
disp(hils_master_targets(1:5, :));

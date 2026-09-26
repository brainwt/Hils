%EX03_COSIM_MASTER  Simulation 객체 co-simulation: 60 s 마다 정지 -> 외부 존 모델 -> 재개.  [MATLAB R2024a+]
%   EnergyPlus/FMU 를 쓰려면 ZoneFcn 을 교체한다:
%     zs = my_energyplus_step(k, t, q)   % q.Q_sens, q.Q_lat, q.m_w -> zs.T_z, RH_z, T_out, RH_out
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
build_HILS_Controller('Season', 'summer', 'PlcIo', 'emulator');
out = hils_master('Mode', 'cosim', 'Season', 'summer', 'Duration', 3600);
disp(hils_master_history(1:5, :));   % [t Q_sens m_w T_z RH_z T_out]

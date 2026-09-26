%EX04_SCRIPT_MASTER_MODBUS  Simulink 없이 MATLAB 스크립트 master 로 PLC 와 실시간 폐루프.  [MATLAB + ICT]
%   Phase 1 시험: 가짜 PLC(Python) 에 Modbus TCP 로 접속해 레지스터 맵/시퀀스/지연을 확인.
%     (터미널) cd python && python -m hils.plc_server --port 5020
%   실물 PLC 는 host/port 만 바꾼다.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
P = hils_params();
io = hils_io_modbus(P, '127.0.0.1', 5020);
w = io.read(); m = hils_decode_meas(w, P);
fprintf('PLC: T_indoor=%.2f degC  status=%d  heartbeat=%d\n', m.T_indoor, m.PLC_status, m.PLC_heartbeat);
bf = @(t, Tin) deal(2000 + 1500 * (t >= 900), P.emulator.hp_setpoint, 0);  % 계단 부하
[L, S] = hils_run_realtime(P, 1800, io, struct('pacing', 1, 'BuildingFcn', bf));
fprintf('last ack latency %.0f s, faults %d samples\n', S.dm.last_latency, sum(L.fault > 0));

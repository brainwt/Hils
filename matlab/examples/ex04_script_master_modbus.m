%EX04_SCRIPT_MASTER_MODBUS  Simulink 없이 MATLAB 스크립트 master 로 PLC 와 실시간 폐루프.  [MATLAB + ICT]
%   (터미널) cd python && python -m hils.plc_server --port 5020 --season summer
%   실물 PLC 는 host/port 만 바꾼다.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'lib')); addpath(fullfile(here, '..'));
P = hils_params([], 'summer');
io = hils_io_modbus(P, '127.0.0.1', 5020);
m = hils_decode_meas(io.read(), P);
fprintf('PLC: T_return=%.2f degC RH=%.1f %% V=%.0f m3/h status=%d\n', m.T_return, m.RH_return, m.V_air, m.PLC_status);
[L, S] = hils_run_realtime(P, 1800, io, struct('pacing', 1));
fprintf('last ack latency %.0f s, faults %d samples, T_z end %.2f degC\n', ...
        S.dm.last_latency, sum(L.fault > 0), L.T_z(end));

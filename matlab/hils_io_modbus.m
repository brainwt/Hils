function io = hils_io_modbus(P, host, port)
%HILS_IO_MODBUS  Industrial Communication Toolbox modbus 객체로 PLC I/O 함수 핸들 생성.
%   io.read()        -> 15 words (40001~40015)
%   io.write(words)  -> 7 words  (40100~40106)
%   MATLAB 전용 (Octave 미지원). 가짜 PLC: python -m hils.plc_server --port 5020
if nargin < 2, host = P.modbus.host; end
if nargin < 3, port = P.modbus.port; end
m = modbus('tcpip', host, port);
m.Timeout = P.modbus.timeout_s;
id = P.modbus.unit_id;
base = P.modbus.base_address;
% modbus() 의 holdingregs 주소는 1-based (40001 -> 1)
a_in  = P.registers.T_supply.addr  - base + 1;
a_out = P.registers.T_room_SP.addr - base + 1;
io.read    = @() double(read(m, 'holdingregs', a_in, 15, id, 'uint16')).';
io.write   = @(w) write(m, 'holdingregs', a_out, uint16(w(:).'), id, 'uint16');
io.close   = @() clear('m');
end

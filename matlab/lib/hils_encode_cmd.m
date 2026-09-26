function w = hils_encode_cmd(c, P)
%HILS_ENCODE_CMD  명령 구조체 -> Simulink->PLC 블록 (40100~40106, 7 words).
R = P.registers;
w = zeros(7, 1);
w(1) = hils_encode(c.T_room_SP,     R.T_room_SP.scale,     R.T_room_SP.signed);
w(2) = hils_encode(c.RH_room_SP,    R.RH_room_SP.scale,    R.RH_room_SP.signed);
w(3) = hils_encode(c.T_outdoor_SP,  R.T_outdoor_SP.scale,  R.T_outdoor_SP.signed);
w(4) = hils_encode(c.RH_outdoor_SP, R.RH_outdoor_SP.scale, R.RH_outdoor_SP.signed);
w(5) = hils_encode(c.Enable,        R.Enable.scale,        R.Enable.signed);
w(6) = hils_encode(c.Sequence,      R.Sequence.scale,      R.Sequence.signed);
w(7) = hils_encode(c.SIM_heartbeat, R.SIM_heartbeat.scale, R.SIM_heartbeat.signed);
end

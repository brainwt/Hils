function c = hils_decode_cmd(w, P)
%HILS_DECODE_CMD  PLC 측: 7 words -> 명령 구조체.
R = P.registers;
c.T_room_SP     = hils_decode(w(1), R.T_room_SP.scale,     R.T_room_SP.signed);
c.RH_room_SP    = hils_decode(w(2), R.RH_room_SP.scale,    R.RH_room_SP.signed);
c.T_outdoor_SP  = hils_decode(w(3), R.T_outdoor_SP.scale,  R.T_outdoor_SP.signed);
c.RH_outdoor_SP = hils_decode(w(4), R.RH_outdoor_SP.scale, R.RH_outdoor_SP.signed);
c.Enable        = hils_decode(w(5), R.Enable.scale,        R.Enable.signed);
c.Sequence      = hils_decode(w(6), R.Sequence.scale,      R.Sequence.signed);
c.SIM_heartbeat = hils_decode(w(7), R.SIM_heartbeat.scale, R.SIM_heartbeat.signed);
end

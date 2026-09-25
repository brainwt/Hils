function c = hils_decode_cmd(w, P)
%HILS_DECODE_CMD  PLC 측: 명령 레지스터 블록(6 words) -> 명령 구조체.
R = P.registers;
c.Q_load_cmd    = hils_decode(w(1), R.Q_load_cmd.scale,    R.Q_load_cmd.signed);
c.T_chamber_SP  = hils_decode(w(2), R.T_chamber_SP.scale,  R.T_chamber_SP.signed);
c.Enable        = hils_decode(w(3), R.Enable.scale,        R.Enable.signed);
c.Sequence      = hils_decode(w(4), R.Sequence.scale,      R.Sequence.signed);
c.SIM_heartbeat = hils_decode(w(5), R.SIM_heartbeat.scale, R.SIM_heartbeat.signed);
c.T_outdoor_SP  = hils_decode(w(6), R.T_outdoor_SP.scale,  R.T_outdoor_SP.signed);
end

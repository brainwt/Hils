function w = hils_encode_cmd(c, P)
%HILS_ENCODE_CMD  명령 구조체 -> Simulink->PLC 레지스터 블록(40100~40105, 6 words).
R = P.registers;
w = zeros(6, 1);
w(1) = hils_encode(c.Q_load_cmd,    R.Q_load_cmd.scale,    R.Q_load_cmd.signed);
w(2) = hils_encode(c.T_chamber_SP,  R.T_chamber_SP.scale,  R.T_chamber_SP.signed);
w(3) = hils_encode(c.Enable,        R.Enable.scale,        R.Enable.signed);
w(4) = hils_encode(c.Sequence,      R.Sequence.scale,      R.Sequence.signed);
w(5) = hils_encode(c.SIM_heartbeat, R.SIM_heartbeat.scale, R.SIM_heartbeat.signed);
w(6) = hils_encode(c.T_outdoor_SP,  R.T_outdoor_SP.scale,  R.T_outdoor_SP.signed);
end

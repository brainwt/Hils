function w = hils_encode_meas(m, P)
%HILS_ENCODE_MEAS  PLC 측: 측정 구조체 -> 측정 레지스터 블록(10 words).
R = P.registers;
w = zeros(10, 1);
w(1)  = hils_encode(m.T_indoor,      R.T_indoor.scale,      R.T_indoor.signed);
w(2)  = hils_encode(m.RH_indoor,     R.RH_indoor.scale,     R.RH_indoor.signed);
w(3)  = hils_encode(m.T_outdoor,     R.T_outdoor.scale,     R.T_outdoor.signed);
w(4)  = hils_encode(m.P_HP,          R.P_HP.scale,          R.P_HP.signed);
w(5)  = hils_encode(m.Q_HP,          R.Q_HP.scale,          R.Q_HP.signed);
w(6)  = hils_encode(m.HP_status,     R.HP_status.scale,     R.HP_status.signed);
w(7)  = hils_encode(m.Q_load_meas,   R.Q_load_meas.scale,   R.Q_load_meas.signed);
w(8)  = hils_encode(m.AckSequence,   R.AckSequence.scale,   R.AckSequence.signed);
w(9)  = hils_encode(m.PLC_status,    R.PLC_status.scale,    R.PLC_status.signed);
w(10) = hils_encode(m.PLC_heartbeat, R.PLC_heartbeat.scale, R.PLC_heartbeat.signed);
end

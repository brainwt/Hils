function m = hils_decode_meas(w, P)
%HILS_DECODE_MEAS  PLC->Simulink 레지스터 블록(40001~40010, 10 words) 을 측정 구조체로 변환.
%   w(k) 는 주소 40000+k 의 word. codegen 호환을 위해 신호별로 명시적으로 작성.
R = P.registers;
m.T_indoor      = hils_decode(w(1),  R.T_indoor.scale,      R.T_indoor.signed);
m.RH_indoor     = hils_decode(w(2),  R.RH_indoor.scale,     R.RH_indoor.signed);
m.T_outdoor     = hils_decode(w(3),  R.T_outdoor.scale,     R.T_outdoor.signed);
m.P_HP          = hils_decode(w(4),  R.P_HP.scale,          R.P_HP.signed);
m.Q_HP          = hils_decode(w(5),  R.Q_HP.scale,          R.Q_HP.signed);
m.HP_status     = hils_decode(w(6),  R.HP_status.scale,     R.HP_status.signed);
m.Q_load_meas   = hils_decode(w(7),  R.Q_load_meas.scale,   R.Q_load_meas.signed);
m.AckSequence   = hils_decode(w(8),  R.AckSequence.scale,   R.AckSequence.signed);
m.PLC_status    = hils_decode(w(9),  R.PLC_status.scale,    R.PLC_status.signed);
m.PLC_heartbeat = hils_decode(w(10), R.PLC_heartbeat.scale, R.PLC_heartbeat.signed);
end

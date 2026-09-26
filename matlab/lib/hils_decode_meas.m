function m = hils_decode_meas(w, P)
%HILS_DECODE_MEAS  PLC->Simulink 블록 (40001~40015, 15 words) -> 측정 구조체.
R = P.registers;
m.T_supply      = hils_decode(w(1),  R.T_supply.scale,      R.T_supply.signed);
m.RH_supply     = hils_decode(w(2),  R.RH_supply.scale,     R.RH_supply.signed);
m.T_return      = hils_decode(w(3),  R.T_return.scale,      R.T_return.signed);
m.RH_return     = hils_decode(w(4),  R.RH_return.scale,     R.RH_return.signed);
m.V_air         = hils_decode(w(5),  R.V_air.scale,         R.V_air.signed);
m.P_atm         = hils_decode(w(6),  R.P_atm.scale,         R.P_atm.signed);
m.T_chamber     = hils_decode(w(7),  R.T_chamber.scale,     R.T_chamber.signed);
m.RH_chamber    = hils_decode(w(8),  R.RH_chamber.scale,    R.RH_chamber.signed);
m.T_outdoor     = hils_decode(w(9),  R.T_outdoor.scale,     R.T_outdoor.signed);
m.RH_outdoor    = hils_decode(w(10), R.RH_outdoor.scale,    R.RH_outdoor.signed);
m.P_HP          = hils_decode(w(11), R.P_HP.scale,          R.P_HP.signed);
m.HP_status     = hils_decode(w(12), R.HP_status.scale,     R.HP_status.signed);
m.AckSequence   = hils_decode(w(13), R.AckSequence.scale,   R.AckSequence.signed);
m.PLC_status    = hils_decode(w(14), R.PLC_status.scale,    R.PLC_status.signed);
m.PLC_heartbeat = hils_decode(w(15), R.PLC_heartbeat.scale, R.PLC_heartbeat.signed);
end

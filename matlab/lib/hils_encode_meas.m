function w = hils_encode_meas(m, P)
%HILS_ENCODE_MEAS  PLC 측: 측정 구조체 -> 15 words.
R = P.registers;
w = zeros(15, 1);
w(1)  = hils_encode(m.T_supply,      R.T_supply.scale,      R.T_supply.signed);
w(2)  = hils_encode(m.RH_supply,     R.RH_supply.scale,     R.RH_supply.signed);
w(3)  = hils_encode(m.T_return,      R.T_return.scale,      R.T_return.signed);
w(4)  = hils_encode(m.RH_return,     R.RH_return.scale,     R.RH_return.signed);
w(5)  = hils_encode(m.V_air,         R.V_air.scale,         R.V_air.signed);
w(6)  = hils_encode(m.P_atm,         R.P_atm.scale,         R.P_atm.signed);
w(7)  = hils_encode(m.T_chamber,     R.T_chamber.scale,     R.T_chamber.signed);
w(8)  = hils_encode(m.RH_chamber,    R.RH_chamber.scale,    R.RH_chamber.signed);
w(9)  = hils_encode(m.T_outdoor,     R.T_outdoor.scale,     R.T_outdoor.signed);
w(10) = hils_encode(m.RH_outdoor,    R.RH_outdoor.scale,    R.RH_outdoor.signed);
w(11) = hils_encode(m.P_HP,          R.P_HP.scale,          R.P_HP.signed);
w(12) = hils_encode(m.HP_status,     R.HP_status.scale,     R.HP_status.signed);
w(13) = hils_encode(m.AckSequence,   R.AckSequence.scale,   R.AckSequence.signed);
w(14) = hils_encode(m.PLC_status,    R.PLC_status.scale,    R.PLC_status.signed);
w(15) = hils_encode(m.PLC_heartbeat, R.PLC_heartbeat.scale, R.PLC_heartbeat.signed);
end

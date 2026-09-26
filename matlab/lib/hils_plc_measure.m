function m = hils_plc_measure(E, P)
%HILS_PLC_MEASURE  PLC 측정값 구조체 (노이즈 없음; 노이즈는 Python 에뮬레이터에서만).
bits = P.plc_status_bits;
st = 2^bits.READY;
if E.estop, st = st + 2^bits.ESTOP; end
if E.applied.Enable && ~E.watchdog, st = st + 2^bits.TRACKING; end
if E.watchdog, st = st + 2^bits.WATCHDOG; end
V = E.m_da * hils_psy_v(E.T_s, E.W_s, E.P) * 3600;       % 노즐 = 토출측
m.T_supply = E.T_s;  m.RH_supply = min(hils_psy_rh(E.T_s, E.W_s, E.P), 100);
m.T_return = E.T;    m.RH_return = min(hils_psy_rh(E.T, E.W, E.P), 100);
m.V_air = V; m.P_atm = E.P;
m.T_chamber = E.T;   m.RH_chamber = min(hils_psy_rh(E.T, E.W, E.P), 100);
m.T_outdoor = E.T_out; m.RH_outdoor = E.RH_out;
m.P_HP = E.hp.P; m.HP_status = E.hp.status;
m.AckSequence = E.applied.Sequence; m.PLC_status = st; m.PLC_heartbeat = E.hb;
end

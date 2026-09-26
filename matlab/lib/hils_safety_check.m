function fault = hils_safety_check(m, s, bits, codes, hb_lost, ack_timeout)
%HILS_SAFETY_CHECK  Safety interlock. fault bitmask (0 = 정상)
%   bit0 T_indoor 범위, bit1 RH, bit2 HP fault, bit3 PLC fault/E-stop/watchdog,
%   bit4 heartbeat 두절, bit5 ack timeout, bit6 P_HP 과대
fault = 0;
if m.T_indoor < s.T_indoor_min || m.T_indoor > s.T_indoor_max, fault = fault + 1;  end
if m.RH_indoor > s.RH_max,                                     fault = fault + 2;  end
if round(m.HP_status) == codes.FAULT,                          fault = fault + 4;  end
st = round(m.PLC_status);
if bitand(st, 2^bits.FAULT) || bitand(st, 2^bits.ESTOP) || bitand(st, 2^bits.WATCHDOG)
    fault = fault + 8;   % WATCHDOG: PLC 가 명령 두절 감지 -> ack timeout 전에 즉시 trip
end
if hb_lost,                                                    fault = fault + 16; end
if ack_timeout,                                                fault = fault + 32; end
if m.P_HP > s.P_HP_max,                                        fault = fault + 64; end
end

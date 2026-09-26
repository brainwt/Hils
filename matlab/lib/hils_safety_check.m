function fault = hils_safety_check(m, s, bits, codes, hb_lost, ack_timeout, track_lost)
%HILS_SAFETY_CHECK  fault bitmask (Python hils.safety 와 동일)
%   1 T 범위, 2 RH 범위, 4 HP fault, 8 PLC fault/E-stop/watchdog, 16 heartbeat,
%   32 ack timeout, 64 P_HP, 128 풍량 이상, 256 챔버 추종 실패
fault = 0;
if m.T_chamber < s.T_chamber_min || m.T_chamber > s.T_chamber_max, fault = fault + 1; end
if m.RH_chamber < s.RH_chamber_min || m.RH_chamber > s.RH_chamber_max, fault = fault + 2; end
hs = round(m.HP_status);
if hs == codes.FAULT, fault = fault + 4; end
st = round(m.PLC_status);
if bitand(st, 2^bits.FAULT) || bitand(st, 2^bits.ESTOP) || bitand(st, 2^bits.WATCHDOG)
    fault = fault + 8;
end
if hb_lost, fault = fault + 16; end
if ack_timeout, fault = fault + 32; end
if m.P_HP > s.P_HP_max, fault = fault + 64; end
running = hs == codes.HEATING || hs == codes.COOLING;
if m.V_air > s.V_air_max || (running && m.V_air < s.V_air_min_running), fault = fault + 128; end
if track_lost, fault = fault + 256; end
end

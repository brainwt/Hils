function [word, overflow] = hils_encode(value, scale, signed)
%HILS_ENCODE  공학단위 값 -> 16-bit Modbus 레지스터 (uint16 값을 double 로 반환).
%   raw = round(value*scale), signed 이면 int16 2의 보수. 범위 밖은 포화 + overflow.
raw = round(value * scale);   % MATLAB round = half away from zero (Python 측과 동일)
if signed
    lo = -32768; hi = 32767;
else
    lo = 0; hi = 65535;
end
overflow = raw < lo || raw > hi;
raw = min(max(raw, lo), hi);
if raw < 0
    word = raw + 65536;
else
    word = raw;
end
end

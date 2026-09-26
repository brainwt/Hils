function value = hils_decode(word, scale, signed)
%HILS_DECODE  16-bit 레지스터 -> 공학단위 값.  예) 2357, scale 100 -> 23.57
word = mod(double(word), 65536);
if signed && word >= 32768
    word = word - 65536;
end
value = word / scale;
end

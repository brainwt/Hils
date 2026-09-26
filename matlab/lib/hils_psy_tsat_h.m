function t = hils_psy_tsat_h(h, P)
%HILS_PSY_TSAT_H  포화 엔탈피가 h 인 온도 (이분법 60회, Python 과 동일).
lo = -40; hi = 60;
for k = 1:60
    mid = 0.5 * (lo + hi);
    if hils_psy_h(mid, hils_psy_w(mid, 100, P)) > h
        hi = mid;
    else
        lo = mid;
    end
end
t = 0.5 * (lo + hi);
end

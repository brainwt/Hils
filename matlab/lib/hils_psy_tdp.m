function t = hils_psy_tdp(W, P)
%HILS_PSY_TDP  이슬점 온도 (이분법 60회).
lo = -40; hi = 60;
for k = 1:60
    mid = 0.5 * (lo + hi);
    if hils_psy_w(mid, 100, P) > W
        hi = mid;
    else
        lo = mid;
    end
end
t = 0.5 * (lo + hi);
end

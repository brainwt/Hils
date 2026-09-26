function t = hils_psy_t_hw(h, W)
%HILS_PSY_T_HW  엔탈피·절대습도 -> 건구온도.
t = (h - 2501 * W) / (1.006 + 1.86 * W);
end

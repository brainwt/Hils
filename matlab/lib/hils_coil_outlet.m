function [T_s, W_s] = hils_coil_outlet(T_r, W_r, Q, m_da, cool, BF, P)
%HILS_COIL_OUTLET  실내기 출구 공기. 난방: 현열만. 냉방: bypass-factor/ADP 습코일 모델.
%   Q [W] >= 0 : 공급(난방) 또는 제거(냉방) 열량 크기
if m_da <= 0 || Q <= 0
    T_s = T_r; W_s = W_r; return;
end
if ~cool
    T_s = T_r + Q / (1000 * m_da * (1.006 + 1.86 * W_r)); W_s = W_r; return;
end
h_r = hils_psy_h(T_r, W_r);
h_s = h_r - Q / (1000 * m_da);
h_adp = (h_s - BF * h_r) / (1 - BF);
T_adp = hils_psy_tsat_h(h_adp, P);
W_adp = hils_psy_w(T_adp, 100, P);
if W_adp >= W_r
    W_s = W_r;                           % 건코일
else
    W_s = BF * W_r + (1 - BF) * W_adp;   % 습코일(제습)
end
T_s = hils_psy_t_hw(h_s, W_s);
end

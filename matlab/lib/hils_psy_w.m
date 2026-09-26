function W = hils_psy_w(t, rh, P)
%HILS_PSY_W  절대습도 [kg/kg_da] from 건구온도 t [degC], RH [%], 대기압 P [kPa].
pw = rh / 100 * hils_psy_pws(t);
W = 0.621945 * pw / (P - pw);
end

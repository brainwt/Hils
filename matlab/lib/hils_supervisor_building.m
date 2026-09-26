function S = hils_supervisor_building(S, P, T_z, RH_z, T_out, RH_out)
%HILS_SUPERVISOR_BUILDING  60 s 층: 가상 존의 다음 상태 -> 챔버 목표, Sequence 증가.
p = P.state_machine;
S.new_step = abs(T_z - S.T_target) > p.step_threshold_T_K || ...
             abs(RH_z - S.RH_target) > p.step_threshold_RH_pct;
S.T_target = T_z; S.RH_target = RH_z; S.T_out_target = T_out; S.RH_out_target = RH_out;
S.seq = mod(S.seq, 65535) + 1;
S.send = true;
end

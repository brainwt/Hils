function S = hils_supervisor_building(S, P, Q_target, T_target, T_out)
%HILS_SUPERVISOR_BUILDING  60 s 층: 가상건물 새 목표 수신 -> Sequence 증가, step 감지.
S.new_step = abs(Q_target - S.Q_target) > P.state_machine.step_threshold_W;
S.Q_target = Q_target; S.T_target = T_target; S.T_out_target = T_out;
S.seq = mod(S.seq, 65535) + 1;          % 1..65535 (0 = 명령없음 예약)
S.send = true;                          % 다음 realization step 에서 delay monitor 등록
end

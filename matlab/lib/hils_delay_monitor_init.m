function D = hils_delay_monitor_init(N)
%HILS_DELAY_MONITOR_INIT  고정 크기(N) 시퀀스 이력 버퍼 (설정값 T, RH 저장). codegen 호환.
D.seq = zeros(N, 1); D.tsent = zeros(N, 1);
D.tT = zeros(N, 1); D.tRH = zeros(N, 1);       % 그 시퀀스의 설정값
D.used = false(N, 1); D.out = false(N, 1);
D.head = 0; D.last_ack = 0; D.last_latency = NaN;
D.T_ack = 0; D.RH_ack = 0; D.has_ack = false;
end

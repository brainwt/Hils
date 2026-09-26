function D = hils_delay_monitor_init(N)
%HILS_DELAY_MONITOR_INIT  고정 크기(N) 시퀀스 이력 버퍼. codegen 호환.
D.seq      = zeros(N, 1);    % 송신한 시퀀스
D.tsent    = zeros(N, 1);    % 송신 시각
D.target   = zeros(N, 1);    % 그 시퀀스의 목표 부하
D.used     = false(N, 1);    % 이력 슬롯 사용 여부
D.out      = false(N, 1);    % 미확인(outstanding) 여부
D.head     = 0;              % 마지막으로 쓴 슬롯
D.last_ack = 0;
D.last_latency = NaN;
D.q_ack    = 0;              % ack 된 프레임의 목표
D.has_ack  = false;
end

function [D, pending, timeout, age] = hils_delay_monitor_step(D, send, seq, target, ack, t, ack_timeout)
%HILS_DELAY_MONITOR_STEP  Sequence/AckSequence 지연 감시 (Python DelayMonitor 와 동일 로직).
%   send=true 이면 (seq, target) 을 t 에 송신한 것으로 등록.
%   ack 가 바뀌어 미확인 시퀀스와 일치하면, 그 이전(모듈러 순서) 시퀀스를 모두 확인 처리.
N = numel(D.seq);
if send
    D.head = mod(D.head, N) + 1;
    D.seq(D.head) = seq; D.tsent(D.head) = t; D.target(D.head) = target;
    D.used(D.head) = true; D.out(D.head) = true;
end
if ack ~= D.last_ack
    k = find(D.out & D.seq == ack, 1);
    if ~isempty(k)
        for i = 1:N
            if D.out(i) && seq_diff(D.seq(i), ack) <= 0
                D.out(i) = false;
            end
        end
        D.last_latency = t - D.tsent(k(1));   % k(1): codegen 스칼라 보장
    end
    j = find(D.used & D.seq == ack, 1);
    if ~isempty(j)
        D.q_ack = D.target(j(1)); D.has_ack = true;
    else
        D.has_ack = false;
    end
end
D.last_ack = ack;
pending = any(D.out);
if pending
    age = t - min(D.tsent(D.out));
else
    age = 0;
end
timeout = age > ack_timeout;
end

function d = seq_diff(a, b)
d = mod(a - b, 65536);
if d >= 32768
    d = d - 65536;
end
end

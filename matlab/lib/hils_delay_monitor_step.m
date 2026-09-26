function [D, pending, timeout, age] = hils_delay_monitor_step(D, send, seq, T, RH, ack, t, ack_timeout)
%HILS_DELAY_MONITOR_STEP  Sequence/Ack 지연 감시 (Python DelayMonitor 와 동일).
%   send=true 이면 (seq, T, RH) 를 t 에 송신 등록. 미확인 시퀀스를 모두 추적.
N = numel(D.seq);
if send
    D.head = mod(D.head, N) + 1;
    D.seq(D.head) = seq; D.tsent(D.head) = t; D.tT(D.head) = T; D.tRH(D.head) = RH;
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
        D.last_latency = t - D.tsent(k(1));
    end
    j = find(D.used & D.seq == ack, 1);
    if ~isempty(j)
        D.T_ack = D.tT(j(1)); D.RH_ack = D.tRH(j(1)); D.has_ack = true;
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
if d >= 32768, d = d - 65536; end
end

function [H, lost] = hils_heartbeat_step(H, value, Ts, timeout)
%HILS_HEARTBEAT_STEP  PLC heartbeat 가 timeout 동안 변하지 않으면 통신두절.
%   H = struct('last', -1, 'age', 0)
if value ~= H.last
    H.last = value; H.age = 0;
else
    H.age = H.age + Ts;
end
lost = H.age > timeout;
end

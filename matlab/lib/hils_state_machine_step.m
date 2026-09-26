function SM = hils_state_machine_step(SM, p, Ts, plc_ready, fault, tracking_ok, new_step)
%HILS_STATE_MACHINE_STEP  0 INIT 1 WAIT_PLC 2 STABILIZING 3 RUN 4 STEP_CHANGE 5 SAFE_STOP
%   SM = struct('state',0,'timer',0,'ok_timer',0,'warning',0,'restarts',0,'latched',false)
%   warning: 1 PLC not ready, 2 stabilization timeout, 3 latched (재시작 초과)
s = SM.state;
SM.timer = SM.timer + Ts;
if fault && s ~= 0 && s ~= 5
    SM = go(SM, 5);
elseif s == 0
    SM = go(SM, 1);
elseif s == 1
    if plc_ready && ~fault
        SM = go(SM, 2);
    elseif SM.timer > p.wait_plc_timeout_s
        SM.warning = 1; SM = go(SM, 5);
    end
elseif s == 2
    if tracking_ok, SM.ok_timer = SM.ok_timer + Ts; else, SM.ok_timer = 0; end
    if new_step, SM.ok_timer = 0; end
    if SM.ok_timer >= p.stabilize_hold_s
        SM = go(SM, 3);
    elseif SM.timer > p.stabilize_timeout_s
        SM.warning = 2; SM = go(SM, 5);
    end
elseif s == 3
    SM.restarts = 0;
    if new_step, SM = go(SM, 4); end
elseif s == 4
    SM = go(SM, 2);
elseif s == 5
    if fault || SM.latched, SM.ok_timer = 0; else, SM.ok_timer = SM.ok_timer + Ts; end
    if SM.ok_timer >= p.fault_recover_hold_s
        if SM.restarts >= p.max_auto_restarts
            SM.latched = true; SM.warning = 3; SM.ok_timer = 0;
        else
            SM.restarts = SM.restarts + 1;
            SM = go(SM, 1);
        end
    end
end
end

function SM = go(SM, s)
SM.state = s; SM.timer = 0; SM.ok_timer = 0;
end

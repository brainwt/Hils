"""HILS 운전 상태머신.

INIT -> WAIT_PLC -> STABILIZING -> RUN -> STEP_CHANGE -> STABILIZING -> RUN ...
STABILIZING -> RUN : 챔버 리턴공기가 (ack 된) 설정값을 T/RH 허용오차 안에서 hold 시간 유지
어느 상태든 fault -> SAFE_STOP. fault 해제 후 hold 시간 -> WAIT_PLC (자동 재시작).
자동 재시작이 max_auto_restarts 회를 넘으면 SAFE_STOP 에 잠김(latched) -> reset() 으로만 해제.
"""
INIT, WAIT_PLC, STABILIZING, RUN, STEP_CHANGE, SAFE_STOP = range(6)
STATE_NAMES = ["INIT", "WAIT_PLC", "STABILIZING", "RUN", "STEP_CHANGE", "SAFE_STOP"]


class StateMachine:
    def __init__(self, p):
        self.p = p
        self.state = INIT
        self.timer = 0.0
        self.ok_timer = 0.0
        self.warning = ""
        self.restarts = 0
        self.latched = False

    def reset(self):
        """운전자 해제: 잠김 해제 + 재시작 횟수 초기화."""
        self.latched, self.restarts = False, 0

    def _go(self, new):
        self.state, self.timer, self.ok_timer = new, 0.0, 0.0

    def step(self, Ts, plc_ready, fault, tracking_ok, new_step):
        p, s = self.p, self.state
        self.timer += Ts
        if fault and s not in (INIT, SAFE_STOP):
            self._go(SAFE_STOP)
        elif s == INIT:
            self._go(WAIT_PLC)
        elif s == WAIT_PLC:
            if plc_ready and not fault:
                self._go(STABILIZING)
            elif self.timer > p["wait_plc_timeout_s"]:
                self.warning = "PLC not ready"
                self._go(SAFE_STOP)
        elif s == STABILIZING:
            self.ok_timer = self.ok_timer + Ts if tracking_ok else 0.0
            if new_step:
                self.ok_timer = 0.0
            if self.ok_timer >= p["stabilize_hold_s"]:
                self._go(RUN)
            elif self.timer > p["stabilize_timeout_s"]:
                self.warning = "stabilization timeout"
                self._go(SAFE_STOP)
        elif s == RUN:
            self.restarts = 0                 # 정상 운전 도달 -> 재시작 횟수 초기화
            if new_step:
                self._go(STEP_CHANGE)
        elif s == STEP_CHANGE:
            self._go(STABILIZING)
        elif s == SAFE_STOP:
            self.ok_timer = 0.0 if (fault or self.latched) else self.ok_timer + Ts
            if self.ok_timer >= p["fault_recover_hold_s"]:
                if self.restarts >= p["max_auto_restarts"]:
                    self.latched = True
                    self.warning = "latched: too many restarts"
                    self.ok_timer = 0.0
                else:
                    self.restarts += 1
                    self._go(WAIT_PLC)
        return self.state

    @property
    def enable(self):
        return self.state in (STABILIZING, RUN, STEP_CHANGE)

    @property
    def valid(self):
        return self.state == RUN

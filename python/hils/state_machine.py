"""HILS 운전 상태머신.

INIT -> WAIT_PLC -> STABILIZING -> RUN -> STEP_CHANGE -> STABILIZING -> RUN ...
어느 상태에서든 fault 발생 시 SAFE_STOP. fault 해제 후 hold 시간이 지나면 WAIT_PLC.
"""
INIT, WAIT_PLC, STABILIZING, RUN, STEP_CHANGE, SAFE_STOP = range(6)
STATE_NAMES = ["INIT", "WAIT_PLC", "STABILIZING", "RUN", "STEP_CHANGE", "SAFE_STOP"]


class StateMachine:
    def __init__(self, p):
        self.p = p
        self.state = INIT
        self.timer = 0.0      # 현재 상태 체류시간
        self.ok_timer = 0.0   # STABILIZING: 오차 허용범위 연속 유지시간 / SAFE_STOP: fault 해제 유지시간
        self.warning = ""

    def _go(self, new):
        self.state, self.timer, self.ok_timer = new, 0.0, 0.0

    def step(self, Ts, plc_ready, fault, err_abs, new_step):
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
            self.ok_timer = self.ok_timer + Ts if err_abs < p["stabilize_tol_W"] else 0.0
            if new_step:
                self.ok_timer = 0.0
            if self.ok_timer >= p["stabilize_hold_s"]:
                self._go(RUN)
            elif self.timer > p["stabilize_timeout_s"]:
                self.warning = "stabilization timeout"
                self._go(SAFE_STOP)
        elif s == RUN:
            if new_step:
                self._go(STEP_CHANGE)
        elif s == STEP_CHANGE:
            self._go(STABILIZING)
        elif s == SAFE_STOP:
            self.ok_timer = 0.0 if fault else self.ok_timer + Ts
            if self.ok_timer >= p["fault_recover_hold_s"]:
                self._go(WAIT_PLC)
        return self.state

    @property
    def enable(self):
        return self.state in (STABILIZING, RUN, STEP_CHANGE)

    @property
    def valid(self):
        """연구 데이터로 사용 가능한 구간 (정상상태 재현 중)."""
        return self.state == RUN

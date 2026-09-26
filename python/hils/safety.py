"""Safety interlock. fault bitmask (0 = 정상)."""

FAULT_T_RANGE = 1 << 0
FAULT_RH_RANGE = 1 << 1
FAULT_HP = 1 << 2
FAULT_PLC = 1 << 3
FAULT_HEARTBEAT = 1 << 4
FAULT_ACK_TIMEOUT = 1 << 5
FAULT_P_HP = 1 << 6
FAULT_AIRFLOW = 1 << 7
FAULT_TRACKING = 1 << 8

FAULT_NAMES = {
    FAULT_T_RANGE: "chamber temperature out of range",
    FAULT_RH_RANGE: "chamber humidity out of range",
    FAULT_HP: "heat pump fault status",
    FAULT_PLC: "PLC fault / E-stop / watchdog",
    FAULT_HEARTBEAT: "PLC heartbeat lost",
    FAULT_ACK_TIMEOUT: "sequence ack timeout",
    FAULT_P_HP: "heat pump power over limit",
    FAULT_AIRFLOW: "airflow implausible",
    FAULT_TRACKING: "chamber cannot follow setpoint",
}


def describe(mask):
    return [name for bit, name in FAULT_NAMES.items() if mask & bit]


class HeartbeatWatch:
    """값이 timeout 동안 변하지 않으면 통신두절."""

    def __init__(self, timeout_s):
        self.timeout = timeout_s
        self.last_value = None
        self.age = 0.0

    def step(self, value, Ts):
        if value != self.last_value:
            self.last_value = value
            self.age = 0.0
        else:
            self.age += Ts
        return self.age > self.timeout


class TrackingWatch:
    """RUN 중 |T_return - T_sp(ack)| 가 한계를 hold 시간 넘게 초과하면 fault."""

    def __init__(self, tol, hold):
        self.tol, self.hold, self.age = tol, hold, 0.0

    def step(self, err_abs, active, Ts):
        self.age = self.age + Ts if (active and err_abs > self.tol) else 0.0
        return self.age > self.hold


def check_safety(m, s, bits, hp_codes, hb_lost=False, ack_timeout=False, track_lost=False):
    """m: decoded PLC2SIM dict."""
    f = 0
    if not (s["T_chamber_min"] <= m["T_chamber"] <= s["T_chamber_max"]):
        f |= FAULT_T_RANGE
    if not (s["RH_chamber_min"] <= m["RH_chamber"] <= s["RH_chamber_max"]):
        f |= FAULT_RH_RANGE
    if int(m["HP_status"]) == hp_codes["FAULT"]:
        f |= FAULT_HP
    st = int(m["PLC_status"])
    if st & (1 << bits["FAULT"]) or st & (1 << bits["ESTOP"]) or st & (1 << bits["WATCHDOG"]):
        f |= FAULT_PLC
    if hb_lost:
        f |= FAULT_HEARTBEAT
    if ack_timeout:
        f |= FAULT_ACK_TIMEOUT
    if m["P_HP"] > s["P_HP_max"]:
        f |= FAULT_P_HP
    running = int(m["HP_status"]) in (hp_codes["HEATING"], hp_codes["COOLING"])
    if m["V_air"] > s["V_air_max"] or (running and m["V_air"] < s["V_air_min_running"]):
        f |= FAULT_AIRFLOW
    if track_lost:
        f |= FAULT_TRACKING
    return f

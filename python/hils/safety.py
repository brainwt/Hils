"""Safety interlock. 반환값은 fault bitmask (0 = 정상)."""

FAULT_T_RANGE = 1 << 0
FAULT_RH_HIGH = 1 << 1
FAULT_HP = 1 << 2
FAULT_PLC = 1 << 3
FAULT_HEARTBEAT = 1 << 4
FAULT_ACK_TIMEOUT = 1 << 5
FAULT_P_HP = 1 << 6

FAULT_NAMES = {
    FAULT_T_RANGE: "T_indoor out of range",
    FAULT_RH_HIGH: "RH too high",
    FAULT_HP: "heat pump fault status",
    FAULT_PLC: "PLC fault / E-stop / watchdog",
    FAULT_HEARTBEAT: "PLC heartbeat lost",
    FAULT_ACK_TIMEOUT: "sequence ack timeout",
    FAULT_P_HP: "heat pump power over limit",
}


def describe(mask):
    return [name for bit, name in FAULT_NAMES.items() if mask & bit]


class HeartbeatWatch:
    """PLC heartbeat 레지스터가 timeout 동안 변하지 않으면 통신두절로 판단."""

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


def check_safety(meas, s, bits, hp_codes, hb_lost=False, ack_timeout=False):
    """meas: decoded PLC2SIM dict, s: safety cfg, bits: plc_status_bits."""
    m = 0
    if not (s["T_indoor_min"] <= meas["T_indoor"] <= s["T_indoor_max"]):
        m |= FAULT_T_RANGE
    if meas["RH_indoor"] > s["RH_max"]:
        m |= FAULT_RH_HIGH
    if int(meas["HP_status"]) == hp_codes["FAULT"]:
        m |= FAULT_HP
    st = int(meas["PLC_status"])
    # WATCHDOG: PLC 가 Simulink 명령 두절을 감지해 부하를 해제한 상태 -> ack timeout 을 기다리지 않고 즉시 trip
    if st & (1 << bits["FAULT"]) or st & (1 << bits["ESTOP"]) or st & (1 << bits["WATCHDOG"]):
        m |= FAULT_PLC
    if hb_lost:
        m |= FAULT_HEARTBEAT
    if ack_timeout:
        m |= FAULT_ACK_TIMEOUT
    if meas["P_HP"] > s["P_HP_max"]:
        m |= FAULT_P_HP
    return m

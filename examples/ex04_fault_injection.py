"""예제 4: 안전 인터록 시험 (겨울). heartbeat 두절 / E-stop / 명령 쓰기 두절 / 챔버 용량 저하.

    python examples/ex04_fault_injection.py
"""
import os

import numpy as np
from _common import RESULTS, dump

from hils.config import load_config
from hils.cosim import make_zone, run_hils, season_overrides
from hils.emulator import PLCEmulator
from hils.plotting import plot_timeseries
from hils.safety import describe
from hils.state_machine import STATE_NAMES
from hils.transport import InMemoryTransport


def setp(attr, value, on="plc"):
    def f(plc, tr, sup):
        if on == "plc":
            setattr(plc, attr, value)
        elif on == "tr":
            setattr(tr, attr, value)
        else:
            plc.plant.e[attr] = value
    return f


EVENTS = {
    900: setp("freeze_heartbeat", True), 1000: setp("freeze_heartbeat", False),     # PLC 통신두절 100 s
    2100: setp("estop", True), 2200: setp("estop", False),                          # 비상정지 100 s
    3300: setp("drop_writes", True, "tr"), 3600: setp("drop_writes", False, "tr"),  # 명령 쓰기 두절 300 s
    4800: setp("Q_cond_max", 600.0, "e"),                                           # 챔버 공조 용량 저하
}
cfg = load_config(overrides=season_overrides("winter"))
tr = InMemoryTransport(cfg)
plc = PLCEmulator(cfg, tr.bank)
log, sup, _ = run_hils(cfg, tr, 7200, plc=plc, zone=make_zone(cfg, "winter"), events=dict(EVENTS))
t, st, f = (np.array(log[k]) for k in ("t", "state", "fault"))
trans = [{"t": float(t[i]), "from": STATE_NAMES[st[i - 1]], "to": STATE_NAMES[st[i]],
          "faults": describe(int(f[i]))} for i in range(1, len(t)) if st[i] != st[i - 1]]
dump(trans, "ex04_transitions.json")
plot_timeseries(log, os.path.join(RESULTS, "ex04_faults.png"),
                "Ex.4  faults: heartbeat 900 s, E-stop 2100 s, write loss 3300 s, chamber capacity 4800 s",
                xunit="min")
for r in trans:
    print(f"{r['t']:6.0f} s  {r['from']:>11} -> {r['to']:<11} {', '.join(r['faults'])}")
print("latched:", sup.sm.latched, "| restarts:", sup.sm.restarts)

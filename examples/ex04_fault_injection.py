"""예제 4: 안전 인터록 시험. heartbeat 두절 / E-stop / 명령 쓰기 두절 -> SAFE_STOP -> 복귀.

    python examples/ex04_fault_injection.py
"""
import os

import numpy as np
from _common import RESULTS, dump

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import run_hils
from hils.emulator import PLCEmulator
from hils.plotting import plot_timeseries
from hils.safety import describe
from hils.state_machine import STATE_NAMES
from hils.transport import InMemoryTransport


def ev(attr, value, on_transport=False):
    def f(plc, tr, sup):
        setattr(tr if on_transport else plc, attr, value)
    return f


EVENTS = {
    900: ev("freeze_heartbeat", True), 1000: ev("freeze_heartbeat", False),     # PLC 통신두절 100 s
    2100: ev("estop", True), 2200: ev("estop", False),                          # 비상정지 100 s
    3300: ev("drop_writes", True, True), 3600: ev("drop_writes", False, True),  # 명령 쓰기 두절 300 s
}
cfg = load_config()
tr = InMemoryTransport(cfg)
plc = PLCEmulator(cfg, tr.bank)
log, sup, _ = run_hils(cfg, tr, 4800, plc=plc, building=StepProfileBuilding([(0, 2500)]),
                       events=dict(EVENTS))
t, st, f = (np.array(log[k]) for k in ("t", "state", "fault"))
trans = [{"t": float(t[i]), "from": STATE_NAMES[st[i - 1]], "to": STATE_NAMES[st[i]],
          "faults": describe(int(f[i]))} for i in range(1, len(t)) if st[i] != st[i - 1]]
dump(trans, "ex04_transitions.json")
plot_timeseries(log, os.path.join(RESULTS, "ex04_faults.png"),
                "Ex.4  fault injection: heartbeat loss 900 s, E-stop 2100 s, write loss 3300 s",
                xunit="min")
for r in trans:
    print(f"{r['t']:6.0f} s  {r['from']:>11} -> {r['to']:<11} {', '.join(r['faults'])}")

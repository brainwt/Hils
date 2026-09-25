"""예제 1: 가상건물(2R2C, 겨울 1일) - supervisor - 가짜 PLC/챔버/히트펌프 24 h 폐루프 (오프라인, 결정적).

    python examples/ex01_offline_24h.py
"""
import os
import time

from _common import RESULTS, dump

from hils.config import load_config
from hils.cosim import kpis, run_hils, save_csv
from hils.emulator import PLCEmulator
from hils.plotting import plot_timeseries
from hils.transport import InMemoryTransport

cfg = load_config()
tr = InMemoryTransport(cfg)
plc = PLCEmulator(cfg, tr.bank)
t0 = time.time()
log, sup, bld = run_hils(cfg, tr, 24 * 3600, plc=plc)
k = kpis(log, cfg)
k["wall_time_s"] = time.time() - t0
save_csv(log, os.path.join(RESULTS, "ex01_24h.csv"))
dump(k, "ex01_24h_kpis.json")
plot_timeseries(log, os.path.join(RESULTS, "ex01_24h.png"),
                "Ex.1  24 h winter day - virtual building load realized in chamber")
for key, v in k.items():
    print(f"{key:22s} {v}")

"""예제 1: 겨울(난방)·여름(냉방) 24 h air-enthalpy HILS 폐루프 + 이상적 결합 기준해석 비교.

    python examples/ex01_offline_24h.py
"""
import os
import time

import numpy as np
from _common import RESULTS, dump

from hils.config import _merge, load_config
from hils.cosim import kpis, make_zone, run_hils, save_csv, season_overrides
from hils.emulator import PLCEmulator
from hils.plotting import plot_timeseries
from hils.transport import InMemoryTransport

IDEAL = {"ideal_chamber": True, "ack_delay_s": 0, "meas_noise_T": 0, "meas_noise_RH": 0,
         "meas_noise_V_pct": 0}


def run(season, emu=None, hours=24):
    o = season_overrides(season)
    _merge(o, {"emulator": emu or {}})
    cfg = load_config(overrides=o)
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    log, sup, z = run_hils(cfg, tr, hours * 3600, plc=plc, zone=make_zone(cfg, season))
    return cfg, log


out = {}
for season in ("winter", "summer"):
    t0 = time.time()
    cfg, log = run(season)
    _, ref = run(season, IDEAL)
    k = kpis(log, cfg)
    kr = kpis(ref, cfg)
    e = np.array(log["T_z"]) - np.array(ref["T_z"])
    k["rmse_Tz_vs_ideal_K"] = float(np.sqrt(np.mean(e ** 2)))
    Er, E = kr["heat_kWh"] + kr["cool_kWh"], k["heat_kWh"] + k["cool_kWh"]
    k["energy_vs_ideal_pct"] = 100 * (E - Er) / Er
    k["wall_time_s"] = time.time() - t0
    out[season] = k
    save_csv(log, os.path.join(RESULTS, f"ex01_{season}_24h.csv"))
    plot_timeseries(log, os.path.join(RESULTS, f"ex01_{season}_24h.png"),
                    f"Ex.1  {season} 24 h - air-enthalpy measured heat drives the virtual zone",
                    ref=ref)
    print(f"== {season}")
    for key, v in k.items():
        print(f"  {key:22s} {v}")
dump(out, "ex01_24h_kpis.json")

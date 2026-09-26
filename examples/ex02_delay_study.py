"""예제 2: PLC 명령/ack 지연 0~120 s 가 결합 충실도에 주는 영향 (이상적 결합 대비).

    python examples/ex02_delay_study.py
"""
import os

import numpy as np
from _common import RESULTS, dump

from hils.config import _merge, load_config
from hils.cosim import kpis, make_zone, run_hils, season_overrides
from hils.emulator import PLCEmulator
from hils.plotting import plot_delay_sweep, plot_timeseries
from hils.transport import InMemoryTransport

HOURS = 12
IDEAL = {"ideal_chamber": True, "ack_delay_s": 0, "meas_noise_T": 0, "meas_noise_RH": 0,
         "meas_noise_V_pct": 0}


def run(season, emu):
    o = season_overrides(season)
    _merge(o, {"emulator": emu})
    cfg = load_config(overrides=o)
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    log, *_ = run_hils(cfg, tr, HOURS * 3600, plc=plc, zone=make_zone(cfg, season))
    return cfg, log


table = []
for season in ("winter", "summer"):
    cfg, ref = run(season, IDEAL)
    kr = kpis(ref, cfg)
    Er = kr["heat_kWh"] + kr["cool_kWh"]
    rows = []
    for d in (0, 3, 30, 60, 90, 120):
        cfg, log = run(season, {"ack_delay_s": d})
        k = kpis(log, cfg)
        e = np.array(log["T_z"]) - np.array(ref["T_z"])
        E = k["heat_kWh"] + k["cool_kWh"]
        r = {"season": season, "delay_s": d, "rmse_Tz_vs_ideal_K": float(np.sqrt(np.mean(e ** 2))),
             "max_Tz_vs_ideal_K": float(np.max(np.abs(e))), "energy_err_pct": 100 * (E - Er) / Er,
             "track_rmse_T_K": k["track_rmse_T_K"], "track_rmse_RH_pct": k["track_rmse_RH_pct"],
             "valid_ratio": k["valid_ratio"], "safe_stop_s": k["state_time_s"]["SAFE_STOP"],
             "ack_latency_max_s": k["ack_latency_max_s"]}
        rows.append(r)
        table.append(r)
        if d == 120 and season == "summer":
            plot_timeseries(log, os.path.join(RESULTS, "ex02_summer_delay120.png"),
                            "Ex.2  summer, 120 s PLC delay vs ideal coupling", ref=ref)
    plot_delay_sweep(rows, os.path.join(RESULTS, f"ex02_delay_sweep_{season}.png"))
dump(table, "ex02_delay_study.json")
print(f"{'season':>7} {'delay':>5} {'RMSE_Tz':>8} {'max_Tz':>7} {'E_err%':>7} {'trackT':>7} "
      f"{'trackRH':>7} {'valid':>6} {'SAFE_s':>6}")
for r in table:
    print(f"{r['season']:>7} {r['delay_s']:5d} {r['rmse_Tz_vs_ideal_K']:8.3f} {r['max_Tz_vs_ideal_K']:7.3f} "
          f"{r['energy_err_pct']:7.2f} {r['track_rmse_T_K']:7.3f} {r['track_rmse_RH_pct']:7.2f} "
          f"{r['valid_ratio']:6.3f} {r['safe_stop_s']:6.0f}")

"""예제 2: 1~2 분 HILS 지연 연구. PLC 명령/ack 지연 3~120 s, PI 게인 고정 vs 지연보상(SIMC).

계단 부하 [2.0 -> 3.5 -> 1.5 -> 3.0 kW] 를 30 분 간격으로 인가.
    python examples/ex02_delay_study.py
"""
import os

import numpy as np
from _common import RESULTS, dump

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import kpis, run_hils
from hils.emulator import PLCEmulator
from hils.plotting import plot_delay_sweep, plot_timeseries
from hils.transport import InMemoryTransport

PROFILE = [(0, 2000), (1800, 3500), (3600, 1500), (5400, 3000)]
rows, table = [], []
for comp in (False, True):
    for d in (3, 30, 60, 90, 120):
        cfg = load_config(overrides={"emulator": {"ack_delay_s": d},
                                     "realization_controller": {"delay_compensation": comp}})
        tr = InMemoryTransport(cfg)
        plc = PLCEmulator(cfg, tr.bank)
        log, sup, _ = run_hils(cfg, tr, 7200, plc=plc, building=StepProfileBuilding(PROFILE))
        k = kpis(log, cfg)
        st = np.array(log["state"])
        en = np.isin(st, (2, 3, 4))
        e = np.array(log["Q_ref"]) - np.array(log["Q_load_meas"])
        rmse_en = float(np.sqrt(np.mean(e[en] ** 2)))
        lost = float(np.sum(st[300:] != 3))
        rows.append((d, comp, rmse_en, k["energy_error_pct"], lost))
        table.append({"delay_s": d, "delay_compensation": comp, "rmse_enabled_W": rmse_en,
                      "rmse_run_W": k["rmse_valid_W"], "valid_ratio": k["valid_ratio"],
                      "energy_error_pct": k["energy_error_pct"],
                      "safe_stop_s": k["state_time_s"]["SAFE_STOP"],
                      "ack_latency_max_s": k["ack_latency_max_s"], "Kp_final": log["Kp"][-1]})
        if d == 90:
            plot_timeseries(log, os.path.join(RESULTS, f"ex02_delay90_comp{int(comp)}.png"),
                            f"Ex.2  step loads, 90 s PLC delay, "
                            f"{'delay-aware PI' if comp else 'fixed PI gains'}", xunit="min")
dump(table, "ex02_delay_study.json")
plot_delay_sweep(rows, os.path.join(RESULTS, "ex02_delay_sweep.png"))
print(f"{'delay':>5} {'comp':>5} {'RMSE_en':>8} {'RMSE_run':>8} {'valid':>6} {'E_err%':>7} {'SAFE_s':>6} {'Kp':>6}")
for r in table:
    print(f"{r['delay_s']:5d} {str(r['delay_compensation']):>5} {r['rmse_enabled_W']:8.1f} "
          f"{r['rmse_run_W']:8.1f} {r['valid_ratio']:6.2f} {r['energy_error_pct']:7.2f} "
          f"{r['safe_stop_s']:6.0f} {r['Kp_final']:6.3f}")

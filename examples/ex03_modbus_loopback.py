"""예제 3 (Phase 1): 실제 Modbus TCP 로 가짜 PLC 와 폐루프 (여름 냉방). PLC 는 별도 스레드 비동기 스캔.

    python examples/ex03_modbus_loopback.py              # 30 분을 20배속
    python examples/ex03_modbus_loopback.py --external   # 이미 실행 중인 PLC(실물/가짜)에 접속
"""
import argparse
import os

from _common import RESULTS, dump

from hils.config import load_config
from hils.cosim import kpis, make_zone, run_hils, save_csv, season_overrides
from hils.plc_server import PLCServer
from hils.plotting import plot_timeseries
from hils.transport import ModbusTransport

ap = argparse.ArgumentParser()
ap.add_argument("--external", action="store_true")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5021)
ap.add_argument("--scale", type=float, default=20.0)
ap.add_argument("--duration", type=float, default=1800)
ap.add_argument("--season", default="summer")
a = ap.parse_args()

cfg = load_config(overrides=season_overrides(a.season))
srv = None if a.external else PLCServer(cfg, a.host, a.port, time_scale=a.scale).start()
tr = ModbusTransport(cfg, a.host, a.port)
try:
    log, sup, _ = run_hils(cfg, tr, a.duration, plc=None, zone=make_zone(cfg, a.season),
                           realtime_scale=a.scale)
finally:
    tr.close()
    if srv:
        print(f"PLC scans={srv.scans} modbus requests={srv.server.requests} overruns={srv.overruns}")
        srv.stop()
k = kpis(log, cfg)
k["modbus_errors"] = tr.errors
k["ack_latencies_s"] = [l for _, _, l in sup.dm.latencies]
save_csv(log, os.path.join(RESULTS, "ex03_modbus.csv"))
dump(k, "ex03_modbus_kpis.json")
plot_timeseries(log, os.path.join(RESULTS, "ex03_modbus.png"),
                f"Ex.3  Modbus TCP loop-back, asynchronous fake PLC ({a.season}, x{a.scale:g})",
                xunit="min")
for key in ("valid_ratio", "track_rmse_T_K", "track_rmse_RH_pct", "T_z_mean", "cool_kWh", "SHR",
            "ack_latency_mean_s", "ack_latency_max_s", "fault_samples", "modbus_errors"):
    print(f"{key:22s} {k[key]}")

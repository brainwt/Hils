"""예제 3 (Phase 1): 실제 Modbus TCP 로 가짜 PLC 와 폐루프. PLC 는 별도 스레드에서 비동기 스캔.

    python examples/ex03_modbus_loopback.py            # 30 분 실험을 20배속(~90 s)
    python examples/ex03_modbus_loopback.py --external # 이미 실행 중인 PLC(실물/가짜)에 접속
외부 모드 예:  (터미널1) cd python && python -m hils.plc_server --port 5020
              (터미널2) python examples/ex03_modbus_loopback.py --external --scale 1
"""
import argparse
import os

from _common import RESULTS, dump

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import kpis, run_hils, save_csv
from hils.plc_server import PLCServer
from hils.plotting import plot_timeseries
from hils.transport import ModbusTransport

ap = argparse.ArgumentParser()
ap.add_argument("--external", action="store_true")
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=5021)
ap.add_argument("--scale", type=float, default=20.0)
ap.add_argument("--duration", type=float, default=1800)
a = ap.parse_args()

cfg = load_config()
srv = None if a.external else PLCServer(cfg, a.host, a.port, time_scale=a.scale).start()
tr = ModbusTransport(cfg, a.host, a.port)
try:
    log, sup, _ = run_hils(cfg, tr, a.duration, plc=None, realtime_scale=a.scale,
                           building=StepProfileBuilding([(0, 2000), (900, 3500)]))
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
                f"Ex.3  Modbus TCP loop-back, asynchronous fake PLC (x{a.scale:g})", xunit="min")
for key in ("valid_ratio", "rmse_valid_W", "energy_error_pct", "ack_latency_mean_s",
            "ack_latency_max_s", "fault_samples", "modbus_errors"):
    print(f"{key:22s} {k[key]}")

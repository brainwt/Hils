"""PLC 에뮬레이터를 Modbus TCP 서버로 노출 (가짜 PLC, Phase 1 시험용).

    cd python && python -m hils.plc_server --port 5020 --season summer --time-scale 1

실제 PLC 대신 이 서버에 Simulink(Industrial Communication Toolbox Modbus 블록),
MATLAB modbus() 객체, 또는 Python master 가 접속한다.
time_scale>1 이면 실시간보다 빠르게 스캔한다(시험용).
"""
import argparse
import threading
import time

from .config import load_config
from .emulator import PLCEmulator
from .modbus_server import ModbusTCPServer, RegisterBank
from .transport import BANK_SIZE


class PLCServer:
    def __init__(self, cfg, host="127.0.0.1", port=None, time_scale=1.0, seed=0):
        self.cfg = cfg
        self.time_scale = time_scale
        self.bank = RegisterBank(BANK_SIZE)
        self.plc = PLCEmulator(cfg, self.bank, seed=seed)
        self.server = ModbusTCPServer(self.bank, host, port or cfg["modbus"]["port"],
                                      cfg["modbus"]["unit_id"])
        self.host, self.port = self.server.server_address
        self._stop = threading.Event()
        self.scans = 0
        self.overruns = 0

    def _scan_loop(self):
        period = self.cfg["timing"]["Ts_plc"] / self.time_scale
        nxt = time.perf_counter()
        while not self._stop.is_set():
            with self.bank.lock:          # 스캔은 원자적으로 (읽기 중 반쯤 갱신된 프레임 방지)
                self.plc.scan()
            self.scans += 1
            nxt += period
            slack = nxt - time.perf_counter()
            if slack < 0:
                self.overruns += 1
            time.sleep(max(0.0, slack))

    def start(self):
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self._scan = threading.Thread(target=self._scan_loop, daemon=True)
        self._scan.start()
        return self

    def stop(self):
        self._stop.set()
        self._scan.join(timeout=2)
        self.server.shutdown()
        self.server.server_close()


def main():
    ap = argparse.ArgumentParser(description="HILS fake PLC (Modbus TCP)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=None)
    ap.add_argument("--time-scale", type=float, default=1.0)
    ap.add_argument("--season", choices=["winter", "summer"], default="winter",
                    help="에뮬레이터 초기조건과 히트펌프 운전모드(리모컨 설정에 해당)")
    a = ap.parse_args()
    from .cosim import season_overrides
    srv = PLCServer(load_config(overrides=season_overrides(a.season)), a.host, a.port, a.time_scale).start()
    print(f"fake PLC on {srv.host}:{srv.port} (time scale x{a.time_scale}); Ctrl+C to stop")
    try:
        while True:
            time.sleep(5)
            print(f"  scans={srv.scans} requests={srv.server.requests} overruns={srv.overruns}")
    except KeyboardInterrupt:
        srv.stop()


if __name__ == "__main__":
    main()

"""Modbus TCP 루프백: 가짜 PLC 서버 <-> supervisor, 실제 소켓 통신 + 비동기 스캔."""
import socket
import struct

import numpy as np
import pytest

from hils.config import load_config
from hils.cosim import kpis, make_zone, run_hils, season_overrides
from hils.plc_server import PLCServer


def _free_port():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def _connect(cfg, port):
    import time

    from hils.transport import ModbusTransport
    for _ in range(50):
        try:
            return ModbusTransport(cfg, "127.0.0.1", port)
        except ConnectionError:
            time.sleep(0.1)
    raise ConnectionError


def test_modbus_register_exchange_and_closed_loop():
    pytest.importorskip("pymodbus")
    cfg = load_config(overrides=season_overrides("summer"))
    port, scale = _free_port(), 20.0
    srv = PLCServer(cfg, port=port, time_scale=scale).start()
    try:
        tr = _connect(cfg, port)
        m = tr.read_measurements()
        assert m["PLC_status"] == 1 and 20 < m["T_return"] < 35 and m["P_atm"] == pytest.approx(101.33)
        tr.write_commands({"T_room_SP": 26.37, "RH_room_SP": 55.5, "T_outdoor_SP": -3.25,
                           "RH_outdoor_SP": 60, "Enable": 0, "Sequence": 7, "SIM_heartbeat": 1})
        rr = tr.client.read_holding_registers(99, count=7, device_id=1).registers
        assert rr[0] == 2637 and rr[1] == 5550 and rr[2] == 0x10000 - 325 and rr[5] == 7

        log, sup, _ = run_hils(cfg, tr, 900, plc=None, zone=make_zone(cfg, "summer"),
                               realtime_scale=scale)
        k = kpis(log, cfg)
        assert k["fault_samples"] == 0 and tr.errors == 0
        assert max(log["ack"]) == sup.seq - 1
        lat = np.array([l for _, _, l in sup.dm.latencies])
        assert lat.size >= 13 and np.all(lat < 3 * cfg["emulator"]["ack_delay_s"] + 3)
        assert k["track_rmse_T_K"] < 0.2
        tr.close()
    finally:
        srv.stop()


def test_raw_protocol_and_exceptions():
    cfg = load_config()
    srv = PLCServer(cfg, port=_free_port()).start()
    try:
        with socket.create_connection((srv.host, srv.port), timeout=2) as s:
            def req(pdu, tid=1):
                s.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, 1) + pdu)
                t, p, ln, u = struct.unpack(">HHHB", s.recv(7))
                assert t == tid and p == 0 and u == 1
                return s.recv(ln - 1)
            r = req(struct.pack(">BHH", 3, 0, 15))
            assert r[0] == 3 and r[1] == 30
            assert req(struct.pack(">BHH", 6, 101, 4550), 2) == struct.pack(">BHH", 6, 101, 4550)
            assert req(struct.pack(">BHH", 3, 101, 1), 3)[2:] == struct.pack(">H", 4550)
            assert req(struct.pack(">BHH", 3, 190, 20), 4) == bytes([0x83, 2])
            assert req(struct.pack(">BHH", 5, 0, 0), 5) == bytes([0x85, 1])
    finally:
        srv.stop()

"""Modbus TCP 루프백: 가짜 PLC 서버(pymodbus) <-> supervisor, 실제 소켓 통신 + 비동기 스캔."""
import socket

import numpy as np
import pytest

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import kpis, run_hils
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
    pytest.importorskip("pymodbus")                    # 제3자 클라이언트로 호환성 확인
    cfg = load_config()
    port = _free_port()
    scale = 20.0                                        # 20배속 (600 s -> 30 s)
    srv = PLCServer(cfg, port=port, time_scale=scale).start()
    try:
        tr = _connect(cfg, port)
        m = tr.read_measurements()
        assert m["PLC_status"] == 1 and 15 < m["T_indoor"] < 25
        tr.write_commands({"Q_load_cmd": -1234, "T_chamber_SP": 22.5, "Enable": 0, "Sequence": 7,
                           "SIM_heartbeat": 1, "T_outdoor_SP": -3.25})
        rr = tr.client.read_holding_registers(99, count=6, device_id=1).registers
        assert rr[0] == 0x10000 - 1234 and rr[1] == 2250 and rr[3] == 7 and rr[5] == 0x10000 - 325

        log, sup, _ = run_hils(cfg, tr, 600, plc=None, realtime_scale=scale,
                               building=StepProfileBuilding([(0, 2000), (300, 3000)]))
        k = kpis(log, cfg)
        assert k["fault_samples"] == 0
        # 마지막 시퀀스(t=600 s 송신)를 제외한 모든 시퀀스가 PLC 에 도달해 ack 됨
        assert max(log["ack"]) == sup.seq - 1
        lat = np.array([l for _, _, l in sup.dm.latencies])
        assert lat.size >= 9 and np.all(lat < 3 * cfg["emulator"]["ack_delay_s"] + 3)
        e = np.array(log["Q_ref"]) - np.array(log["Q_load_meas"])
        assert np.abs(e[-60:]).mean() < 150              # 비동기 통신에서도 부하 추종
        assert tr.errors == 0
        tr.close()
    finally:
        srv.stop()


def test_raw_protocol_and_exceptions():
    """pymodbus 없이 원시 소켓으로 MBAP/PDU 형식과 예외응답 확인."""
    import struct

    cfg = load_config()
    srv = PLCServer(cfg, port=_free_port()).start()
    try:
        with socket.create_connection((srv.host, srv.port), timeout=2) as s:
            def req(pdu, tid=1):
                s.sendall(struct.pack(">HHHB", tid, 0, len(pdu) + 1, 1) + pdu)
                hdr = s.recv(7)
                t, p, ln, u = struct.unpack(">HHHB", hdr)
                assert t == tid and p == 0 and u == 1
                return s.recv(ln - 1)
            r = req(struct.pack(">BHH", 3, 0, 10))
            assert r[0] == 3 and r[1] == 20
            assert req(struct.pack(">BHH", 6, 101, 2200), 2) == struct.pack(">BHH", 6, 101, 2200)
            assert req(struct.pack(">BHH", 3, 101, 1), 3)[2:] == struct.pack(">H", 2200)
            assert req(struct.pack(">BHH", 3, 190, 20), 4) == bytes([0x83, 2])   # illegal address
            assert req(struct.pack(">BHH", 5, 0, 0), 5) == bytes([0x85, 1])      # illegal function
    finally:
        srv.stop()

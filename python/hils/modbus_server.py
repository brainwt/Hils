"""의존성 없는 최소 Modbus TCP 서버 (가짜 PLC 용).

지원 function code: 03 Read Holding Registers, 06 Write Single Register,
16 Write Multiple Registers. 그 외는 exception 01(Illegal Function).
pymodbus 서버 API 는 버전마다(3.x) 크게 바뀌어, 시험 장비용으로 직접 구현했다.
클라이언트 호환성은 pymodbus / MATLAB modbus() / Simulink Modbus 블록으로 확인한다.
"""
import socketserver
import struct
import threading


class RegisterBank:
    """스레드 안전 holding register 배열. 0-based offset 인덱싱 (list 처럼 사용)."""

    def __init__(self, size):
        self.words = [0] * size
        self.lock = threading.RLock()

    def __len__(self):
        return len(self.words)

    def __getitem__(self, i):
        with self.lock:
            return self.words[i]

    def __setitem__(self, i, v):
        with self.lock:
            if isinstance(i, slice):
                self.words[i] = [int(x) & 0xFFFF for x in v]
            else:
                self.words[i] = int(v) & 0xFFFF


def _exception(fc, code):
    return struct.pack(">BB", fc | 0x80, code)


def handle_pdu(bank, pdu):
    """Modbus PDU 처리 -> 응답 PDU."""
    fc = pdu[0]
    try:
        if fc == 3:
            addr, count = struct.unpack(">HH", pdu[1:5])
            if not 1 <= count <= 125:
                return _exception(fc, 3)
            if addr + count > len(bank):
                return _exception(fc, 2)
            with bank.lock:
                vals = bank.words[addr:addr + count]
            return struct.pack(">BB", fc, 2 * count) + struct.pack(f">{count}H", *vals)
        if fc == 6:
            addr, val = struct.unpack(">HH", pdu[1:5])
            if addr >= len(bank):
                return _exception(fc, 2)
            bank[addr] = val
            return pdu[:5]
        if fc == 16:
            addr, count, nbytes = struct.unpack(">HHB", pdu[1:6])
            if not 1 <= count <= 123 or nbytes != 2 * count:
                return _exception(fc, 3)
            if addr + count > len(bank):
                return _exception(fc, 2)
            vals = struct.unpack(f">{count}H", pdu[6:6 + nbytes])
            bank[addr:addr + count] = vals
            return struct.pack(">BHH", fc, addr, count)
    except struct.error:
        return _exception(fc, 3)
    return _exception(fc, 1)


class _Handler(socketserver.BaseRequestHandler):
    def _recv(self, n):
        buf = b""
        while len(buf) < n:
            chunk = self.request.recv(n - len(buf))
            if not chunk:
                raise ConnectionError
            buf += chunk
        return buf

    def handle(self):
        srv = self.server
        try:
            while True:
                tid, pid, length, unit = struct.unpack(">HHHB", self._recv(7))
                pdu = self._recv(length - 1)
                if pid != 0:
                    continue
                if srv.unit_id not in (unit, 0) and unit != 255:
                    continue    # 다른 장치 주소: 응답하지 않음(게이트웨이 동작)
                resp = handle_pdu(srv.bank, pdu)
                srv.requests += 1
                self.request.sendall(struct.pack(">HHHB", tid, 0, len(resp) + 1, unit) + resp)
        except (ConnectionError, OSError):
            pass


class ModbusTCPServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, bank, host="127.0.0.1", port=5020, unit_id=1):
        self.bank, self.unit_id, self.requests = bank, unit_id, 0
        super().__init__((host, port), _Handler)

"""Supervisor <-> PLC 통신 계층.

InMemoryTransport : 공유 레지스터 배열 (빠른 오프라인 시험, 24 h 를 수 초에)
ModbusTransport   : Modbus TCP 클라이언트 (실제 PLC 또는 plc_server.py 에뮬레이터)
두 구현 모두 read_measurements() / write_commands() 인터페이스가 같다.
"""
from .registers import RegisterMap

BANK_SIZE = 200


class InMemoryTransport:
    def __init__(self, cfg, bank=None):
        self.rm = RegisterMap(cfg)
        self.bank = bank if bank is not None else [0] * BANK_SIZE
        self.drop_writes = False   # 통신 장애 주입용

    def read_measurements(self):
        start, count = self.rm.block("PLC2SIM")
        return self.rm.unpack("PLC2SIM", start, self.bank[start:start + count])

    def write_commands(self, cmd):
        if self.drop_writes:
            return
        start, words = self.rm.pack("SIM2PLC", cmd)
        self.bank[start:start + len(words)] = words

    def close(self):
        pass


class ModbusTransport:
    def __init__(self, cfg, host=None, port=None):
        from pymodbus.client import ModbusTcpClient

        m = cfg["modbus"]
        self.rm = RegisterMap(cfg)
        self.unit = m["unit_id"]
        self.client = ModbusTcpClient(host or m["host"], port=port or m["port"], timeout=m["timeout_s"])
        if not self.client.connect():
            raise ConnectionError(f"Modbus connect failed {host or m['host']}:{port or m['port']}")
        self.errors = 0

    def read_measurements(self):
        start, count = self.rm.block("PLC2SIM")
        rr = self.client.read_holding_registers(start, count=count, device_id=self.unit)
        if rr.isError():
            self.errors += 1
            raise IOError(f"Modbus read error: {rr}")
        return self.rm.unpack("PLC2SIM", start, rr.registers)

    def write_commands(self, cmd):
        start, words = self.rm.pack("SIM2PLC", cmd)
        wr = self.client.write_registers(start, words, device_id=self.unit)
        if wr.isError():
            self.errors += 1
            raise IOError(f"Modbus write error: {wr}")

    def close(self):
        self.client.close()

# VILLASnode 게이트웨이 설계 (PLC ↔ VILLASnode ↔ 시뮬레이터)

> 상태: **설계 초안.** 설정 파일 `villas/hils.conf`은 VILLASnode 소스(`lib/nodes/modbus.cpp`, `lib/hooks/limit_value.cpp`, `lib/format.cpp`, `etc/examples/nodes/*.conf`)를 읽고 작성했습니다. `villas-node`로 실행해 보지는 않았습니다. 공식 문서 사이트(villas.fein-aachen.org)는 작업 환경에서 접속이 차단되어 소스로만 확인했습니다.

## 1. VILLASnode 요약

| 항목 | 내용 |
|------|------|
| 정체 | RWTH Aachen ACS 연구소의 오픈소스 실시간 다중 프로토콜 게이트웨이. Apache 2.0, C++, Linux, Docker 배포 |
| 구성 요소 | **node**: 장비·프로그램과의 연결 끝점 (38종: modbus, socket, mqtt, zeromq, websocket, file, exec 등)<br>**path**: node 사이의 데이터 흐름<br>**hook**: 흐름 중 가공 (limit_value, scale, round, cast, stats, lua 등 32종) |
| 데이터 단위 | sample = 신호 값 배열 + 순번 + 타임스탬프 |
| 설정 | libconfig(`.conf`) 또는 JSON 파일 하나 |

### Modbus 노드 (소스 확인 사항)

| 항목 | 확인 내용 |
|------|-----------|
| 역할 | **Modbus 클라이언트** (PLC = 서버). TCP와 RTU 지원 (libmodbus 사용) |
| 레지스터 종류 | **holding register만** 읽고 씁니다(`modbus_read_registers`). 종류를 고르는 설정은 없음 |
| 읽기 | `rate` [Hz] 주기로 폴링. 인접 레지스터는 한 번의 요청으로 묶음 (`max_block_size`, `min_block_usage`) |
| 쓰기 | 시뮬레이터 sample이 도착할 때마다 `out` 매핑 전체를 씀 |
| 신호 형식 | `float`(IEEE 754 32비트, 레지스터 2개), `integer`(`integer_registers`개 결합), 정수를 `scale`·`offset`으로 변환한 `float`, `boolean`(비트) |
| **주의 1** | 16비트 정수를 **부호 없이** 읽음 (0xFFFF → 65535) |
| **주의 2** | 실수 → 정수 쓰기에서 **반올림 없이 자름** (`(int64_t)((d − offset)/scale)`) |

주의 1·2 때문에 기존 int16 ×100 맵을 쓰면 영하 온도가 깨지고 0.01 단위 오차가 생깁니다. 그래서 **물리량을 float32로 옮긴 레지스터 맵**(§4)을 씁니다. float32는 IEEE 754 비트를 그대로 복사하므로 부호와 반올림 문제가 모두 없어집니다.

## 2. 목표 구성

```
┌──────────────┐  Modbus TCP (FC03 26 regs / FC16 11 regs)   ┌───────────────┐   UDP, float64 × N (big endian)   ┌─────────────────────────┐
│ PLC          │ ◄─────────────────────────────────────────► │  VILLASnode   │ ◄───────────────────────────────► │ 시뮬레이터                │
│ (Modbus 서버) │        1 Hz 폴링 / sample 도착 시 쓰기       │  node: plc    │   :12000 ← 15 신호 (120 B)       │  Python  hils.cosim      │
│ 챔버 PI      │                                             │  node: sim    │   :12001 → 7 신호  (56 B)        │  MATLAB  hils_run_realtime│
│ air-enthalpy │                                             │  node: log    │                                   │  Simulink UDP 블록        │
└──────────────┘                                             │  hook: stats, │                                   └─────────────────────────┘
                                                             │   limit_value │──► logs/hils_*.csv (원시 기록)
                                                             └───────────────┘
```

## 3. 역할 분담 (현재 → VILLASnode 도입 후)

| 기능 | 현재 (직접 Modbus) | VILLASnode 도입 후 |
|------|--------------------|--------------------|
| Modbus 연결·폴링·재접속 | 시뮬레이터 (`ModbusTransport`, `hils_io_modbus`, Simulink Modbus 블록) | **VILLASnode** (`plc` 노드) |
| 레지스터 ↔ 공학단위 변환 | 시뮬레이터 (`registers.py`, `hils_encode/decode`) | **VILLASnode** (float32 매핑) |
| 원시 데이터 기록 | 시뮬레이터 로그 | **VILLASnode** `file` 노드 (+ 시뮬레이터 로그 유지) |
| 통신 통계 (손실, 지연) | 없음 | **VILLASnode** `stats` |
| 설정값 한계 | supervisor (`setpoint`) | supervisor + VILLASnode `limit_value` (이중 보호) |
| air-enthalpy, 가상 존, Sequence/Ack, 인터록, 상태머신 | 시뮬레이터 | **시뮬레이터 그대로** (변경 없음) |
| 챔버 국부 PI, watchdog | PLC | PLC 그대로 |

시뮬레이터는 Modbus를 몰라도 됩니다. **UDP로 실수 15개를 받고 7개를 보내는 일**만 합니다. 제어 로직과 교차검증 결과는 그대로 유효합니다.

## 4. float32 레지스터 맵 (VILLASnode 모드)

word·byte 순서는 모두 big endian입니다(상위 워드 먼저). 주소 열의 괄호 안은 프로토콜 오프셋입니다.

### PLC → 시뮬레이터 (40001~40026, FC03 1회)

| 주소 (오프셋) | 신호 | 형 | 단위 |
|---|---|---|---|
| 40001–02 (0) | T_supply | float32 | °C |
| 40003–04 (2) | RH_supply | float32 | % |
| 40005–06 (4) | T_return | float32 | °C |
| 40007–08 (6) | RH_return | float32 | % |
| 40009–10 (8) | V_air | float32 | m³/h |
| 40011–12 (10) | P_atm | float32 | kPa |
| 40013–14 (12) | T_chamber | float32 | °C |
| 40015–16 (14) | RH_chamber | float32 | % |
| 40017–18 (16) | T_outdoor | float32 | °C |
| 40019–20 (18) | RH_outdoor | float32 | % |
| 40021–22 (20) | P_HP | float32 | W |
| 40023 (22) | HP_status | uint16 | code |
| 40024 (23) | AckSequence | uint16 | — |
| 40025 (24) | PLC_status | uint16 | bit |
| 40026 (25) | PLC_heartbeat | uint16 | — |

### 시뮬레이터 → PLC (40100~40110, FC16 1회)

| 주소 (오프셋) | 신호 | 형 | 단위 |
|---|---|---|---|
| 40100–01 (99) | T_room_SP | float32 | °C |
| 40102–03 (101) | RH_room_SP | float32 | % |
| 40104–05 (103) | T_outdoor_SP | float32 | °C |
| 40106–07 (105) | RH_outdoor_SP | float32 | % |
| 40108 (107) | Enable | uint16 | 0/1 |
| 40109 (108) | Sequence | uint16 | 1..65535 |
| 40110 (109) | SIM_heartbeat | uint16 | — |

- 정수 레지스터(상태, Sequence, heartbeat)는 원래 부호 없는 값이라 VILLASnode의 부호 없는 읽기와 맞습니다. 쓸 때 잘라내기도 정수라서 오차가 없습니다.
- 신호 순서와 의미는 기존 맵(`docs/02_interface_spec.md` §3)과 같고, 주소와 형만 바뀝니다. 기존 int16 맵은 직접 Modbus 모드에서 계속 씁니다.
- **PLC 쪽 확인:** PLC가 float32를 어떤 워드 순서로 담는지 확인해야 합니다. 제조사마다 다르며, 하위 워드를 먼저 두는 PLC도 있습니다. 다르면 `word_endianess = "little"`로 바꿉니다.

## 5. UDP 패킷 (시뮬레이터 ↔ VILLASnode)

형식은 `raw`, 64비트, big endian, 헤더 없음(`fake = false`)입니다. 패킷 하나가 sample 하나입니다.

| 방향 | 포트 | 내용 | 크기 |
|------|------|------|------|
| VILLASnode → 시뮬레이터 | 시뮬레이터 수신 12000 | float64 × 15 (§4 PLC→Sim 표 순서) | 120 B |
| 시뮬레이터 → VILLASnode | VILLASnode 수신 12001 | float64 × 7 (§4 Sim→PLC 표 순서) | 56 B |

### 시뮬레이터 어댑터 (구현 예정, 기존 인터페이스에 끼워 넣기)

Python: `read_measurements()`, `write_commands()` 인터페이스는 그대로 두고 `VillasTransport`만 추가합니다.

```python
class VillasTransport:            # python/hils/transport.py 에 추가 예정
    IN = ["T_supply", "RH_supply", "T_return", "RH_return", "V_air", "P_atm", "T_chamber",
          "RH_chamber", "T_outdoor", "RH_outdoor", "P_HP", "HP_status", "AckSequence",
          "PLC_status", "PLC_heartbeat"]
    OUT = ["T_room_SP", "RH_room_SP", "T_outdoor_SP", "RH_outdoor_SP", "Enable", "Sequence",
           "SIM_heartbeat"]

    def __init__(self, local=("0.0.0.0", 12000), remote=("127.0.0.1", 12001), timeout=2.0):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(local); self.sock.settimeout(timeout)
        self.remote, self.last = remote, None

    def read_measurements(self):   # 다음 PLC sample 을 기다림 -> 1 Hz 폴링이 시뮬레이터 시계가 됨
        try:
            data, _ = self.sock.recvfrom(1024)
            self.last = dict(zip(self.IN, struct.unpack(">15d", data)))
        except socket.timeout:
            if self.last is None:
                raise ConnectionError("VILLASnode 에서 첫 sample 을 받지 못함")
            # 마지막 값 유지 -> PLC heartbeat 가 멈춰 10 s 뒤 인터록(bit4)이 동작
        return self.last

    def write_commands(self, cmd):
        self.sock.sendto(struct.pack(">7d", *(float(cmd[n]) for n in self.OUT)), self.remote)
```

MATLAB 스크립트 master: `hils_io_villas.m`가 `hils_io_modbus`와 같은 `io.read()`/`io.write()`를 제공하도록 합니다. `udpport("datagram","LocalPort",12000)`로 받고, `swapbytes(typecast(uint8(d.Data),'double'))`로 풉니다. 이때 io 계층에서 쓰던 레지스터 코덱(`hils_decode_meas`)은 쓰지 않고, 공학단위 구조체를 직접 만듭니다.

Simulink: Modbus 블록 대신 UDP Receive/Send 블록(Instrument Control Toolbox 또는 DSP System Toolbox)을 둡니다. double 15개와 7개, Byte Order는 big-endian입니다. `build_HILS_Controller`에 `'PlcIo','villas'` 옵션으로 추가할 예정입니다. 다만 `HILS_Core_1s` 블록 입력이 레지스터 워드를 받도록 되어 있으므로, 공학단위 입력 버전으로 나눠야 합니다.

## 6. 시간 동기와 지연

| 항목 | 설계 |
|------|------|
| 기준 시계 | VILLASnode `rate = 1` (PLC 폴링 1 Hz). 시뮬레이터는 sample이 도착하면 한 스텝을 진행 (수신 대기 = 페이싱) |
| 추가 지연 | Modbus 읽기(수 ms~수십 ms) + UDP(< 1 ms) + 쓰기. 직접 Modbus 방식과 비슷하고, 1 s 주기보다 훨씬 작음 |
| sample 누락 | 수신 timeout(2 s)이면 마지막 값을 유지. 그러면 PLC heartbeat가 멈추고 10 s 뒤 인터록(bit4)이 동작 |
| VILLASnode 정지 | 시뮬레이터: heartbeat 인터록. PLC: SIM_heartbeat watchdog이 설정값 hold (기존 이중 보호 그대로) |
| 건물 60 s 층 | 변화 없음 (시뮬레이터 내부) |
| 추후 선택 | 시각 정합이 필요하면 `fake = true`(순번 + 타임스탬프 헤더)나 `villas.binary` 형식으로 바꾸고 `stats`로 지연을 측정 |

## 7. 이 구성의 장단점

| 장점 | 단점·위험 |
|------|-----------|
| 시뮬레이터가 Modbus 세부(주소, 엔디안, 재접속)를 몰라도 됨 | 프로세스가 하나 늘어남 (Linux PC 또는 Docker 필요) |
| Python, MATLAB 스크립트, Simulink가 같은 UDP 인터페이스를 공유 | holding register만 지원. 16비트 부호 없음, 쓰기 잘라냄 → float32 맵이 필요 |
| 다른 장비 추가가 설정 파일 수정으로 끝남 (MQTT, IEC 61850, 다른 PLC, 실시간 시뮬레이터) | `rate`는 읽기에만 적용되고, 쓰기는 sample마다 발생 (시뮬레이터가 1 s마다 한 번만 보내야 함) |
| 원시 기록과 통신 통계를 기본 제공 | 설정 키 일부를 소스에서만 확인. 실행 검증은 아직 안 함 |

## 8. 적용 순서 (다음 작업)

1. **가짜 PLC에 float32 맵 추가:** `hils_config.json`에 `registers_f32`를 두고, `plc_server.py`의 에뮬레이터가 이를 선택할 수 있게 합니다.
2. **VILLASnode 설치:** Docker 이미지 또는 소스 빌드. 이어서 `villas-node villas/hils.conf`로 가짜 PLC에 접속해 `stats`와 `logs/*.csv`를 확인합니다.
3. **`VillasTransport` 구현**과 루프백 시험: 가짜 PLC → VILLASnode → Python 폐루프. 직접 Modbus 방식 결과(ex03)와 비교합니다.
4. `hils_io_villas.m`와 Simulink `'PlcIo','villas'` 추가.
5. 실제 PLC의 float32 워드 순서를 확인하고, 필요하면 `word_endianess`를 조정합니다.

## 참고

- VILLASnode 저장소: https://github.com/VILLASframework/node
- Modbus 노드 소스: `lib/nodes/modbus.cpp`, 예제: `etc/examples/nodes/modbus.conf`
- Socket 노드 예제: `etc/examples/nodes/socket.conf`, raw 형식: `lib/formats/raw.cpp`
- 논문: S. Vogel et al., "VILLASnode: An Open-Source Real-time Multi-protocol Gateway", JOSS 10(112), 8401, 2025

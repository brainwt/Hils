# HILS 인터페이스 사양 (Simulink ↔ PLC)

단일 소스: `config/hils_config.json`. Python(`hils.config`)과 MATLAB(`hils_params.m`)이 같은 파일을 읽습니다. MATLAB Function 블록용 `hils_params_const.m`은 `hils_write_params_const.m`로 생성하며, 두 값이 일치하는지는 테스트(`t_params_const_in_sync`)가 확인합니다.

## 1. 통신

| 항목 | 값 |
|------|----|
| 프로토콜 | Modbus TCP, Holding Registers |
| 기본 주소 | 127.0.0.1:5020 (가짜 PLC). 실물은 PLC IP:502 |
| Unit ID | 1 |
| 주소 표기 | 40001 = 프로토콜 오프셋 0 (Python/pymodbus, Simulink 블록 StartAddress) = MATLAB `modbus()` 주소 1 |
| 읽기 | FC03, 오프셋 0부터 10 words (40001~40010), 1 s 주기 |
| 쓰기 | FC16, 오프셋 99부터 6 words (40100~40105), 1 s 주기 |

## 2. 레지스터 맵

값 인코딩: `raw = round(value × scale)`. 반올림은 half away from zero(MATLAB `round`, PLC 표준)이고, signed이면 int16 2의 보수입니다. 범위를 넘으면 포화시키고 overflow 플래그를 세웁니다.

| 주소 | 신호 | 방향 | 스케일 | 형 | 설명 |
|------|------|------|--------|----|------|
| 40001 | T_indoor | PLC→SIM | ×100 | int16 | 실내측 챔버 온도 [°C] |
| 40002 | RH_indoor | PLC→SIM | ×100 | uint16 | 상대습도 [%] |
| 40003 | T_outdoor | PLC→SIM | ×100 | int16 | 실외측 챔버 온도 [°C] |
| 40004 | P_HP | PLC→SIM | ×1 | uint16 | 히트펌프 소비전력 [W] |
| 40005 | Q_HP | PLC→SIM | ×1 | int16 | 히트펌프 공급열량 [W] (난방 +) |
| 40006 | HP_status | PLC→SIM | code | uint16 | 0 OFF, 1 HEATING, 2 COOLING, 3 DEFROST, 9 FAULT |
| 40007 | Q_load_meas | PLC→SIM | ×1 | int16 | 부하장치 실측 재현부하 [W] (열 제거 +) |
| 40008 | AckSequence | PLC→SIM | ×1 | uint16 | PLC가 **적용 중인** 명령 프레임의 Sequence |
| 40009 | PLC_status | PLC→SIM | bit | uint16 | bit0 READY, bit1 FAULT, bit2 ESTOP, bit3 LOAD_ACTIVE, bit4 WATCHDOG |
| 40010 | PLC_heartbeat | PLC→SIM | ×1 | uint16 | PLC 스캔마다 +1 |
| 40100 | Q_load_cmd | SIM→PLC | ×1 | int16 | 부하장치 명령 [W] (열 제거 +) |
| 40101 | T_chamber_SP | SIM→PLC | ×100 | int16 | Enable=0일 때 챔버 사전조화 설정온도 [°C] |
| 40102 | Enable | SIM→PLC | ×1 | uint16 | 1 = 부하 재현 모드 |
| 40103 | Sequence | SIM→PLC | ×1 | uint16 | 건물 timestep 번호, 1..65535 순환 (0 = 명령 없음) |
| 40104 | SIM_heartbeat | SIM→PLC | ×1 | uint16 | Simulink 1 s 스텝마다 +1 (PLC watchdog용) |
| 40105 | T_outdoor_SP | SIM→PLC | ×100 | int16 | 실외측 챔버 설정온도 [°C] (가상건물 기상) |

히트펌프 자체 설정(설정온도, 운전모드)은 레지스터에 **없습니다**. native control을 보존하기 위해서입니다.

## 3. Sequence / Ack 프로토콜

```
Simulink                                   PLC
 t=60k  Sequence=k, Q_target(k) ───────►  명령 프레임 수신 (FIFO)
 매 1 s Q_load_cmd (PI 갱신), Seq=k ───►   처리/전달 지연 d 후 프레임 적용
                                  ◄─────── AckSequence = k   (적용 중인 프레임)
 latency = t_ack − t_send(k)
 Q_ref   = Q_target(AckSequence)   ← 오차 계산 기준 (측정과 같은 timestep)
```

- PLC는 명령 프레임(6 words)을 **한 번에 적용**하고, 그 프레임의 Sequence를 AckSequence로 돌려줍니다.
- Simulink는 미확인 시퀀스를 모두 추적합니다(최대 64개). ack = s를 받으면 s 이전(모듈러 순서)의 시퀀스는 모두 확인된 것으로 처리합니다.
- 가장 오래된 미확인 시퀀스의 경과시간이 `ack_timeout_s`(120 s)를 넘으면 FAULT_ACK_TIMEOUT입니다.
- 측정된 지연 θ로 PI 게인을 자동으로 낮춥니다(SIMC, τc = θ):
  `Kp = min(Kp0, τ/(K·2θ))`, `Ti = min(τ, 8θ)`, `Ki = min(Ki0, Kp/Ti)`. 공칭 τ = 20 s, K = 1.

## 4. 상태머신

| 코드 | 상태 | Enable | 진입 조건 → 다음 상태 |
|------|------|--------|-----------------------|
| 0 | INIT | 0 | 첫 스텝 → WAIT_PLC |
| 1 | WAIT_PLC | 0 | PLC READY & fault 없음 → STABILIZING / 60 s 초과 → SAFE_STOP |
| 2 | STABILIZING | 1 | \|Q_ref − Q_meas\| < 300 W를 120 s 연속 유지 → RUN / 1800 s 초과 → SAFE_STOP |
| 3 | RUN | 1 | **연구 데이터 유효 구간(valid)**. \|ΔQ_target\| > 500 W → STEP_CHANGE |
| 4 | STEP_CHANGE | 1 | 1 스텝 표시 후 → STABILIZING |
| 5 | SAFE_STOP | 0 | fault 해제 후 60 s 유지 → WAIT_PLC. 명령은 50 W/s로 0까지 ramp-down |

## 5. Safety interlock (fault bitmask)

| bit | 값 | 조건 |
|-----|----|------|
| 0 | 1 | T_indoor ∉ [10, 35] °C |
| 1 | 2 | RH > 90 % |
| 2 | 4 | HP_status = 9 (FAULT) |
| 3 | 8 | PLC_status FAULT, ESTOP 또는 WATCHDOG |
| 4 | 16 | PLC_heartbeat가 10 s 이상 변하지 않음 |
| 5 | 32 | 가장 오래된 미확인 Sequence가 120 s 초과 |
| 6 | 64 | P_HP > 4000 W |

**PLC 측 watchdog (필수 구현):** SIM_heartbeat가 10 s 이상 변하지 않으면 PLC가 스스로 부하장치를 해제하고(`Enable` 무시) bit4 WATCHDOG를 세웁니다. 히트펌프는 native 제어로 계속 운전합니다.

## 6. Simulink 모델 신호 (To Workspace)

| 변수 | 내용 |
|------|------|
| `hils_meas` | [T_indoor RH T_outdoor P_HP Q_HP HP_status Q_load_meas Ack PLC_status PLC_hb] |
| `hils_cmd` | [Q_load_cmd T_chamber_SP Enable Sequence SIM_heartbeat T_outdoor_SP] |
| `hils_status` | [state fault pending ack_age latency e valid q_ref kp integ] |
| `hils_target` | [seq Q_target T_target T_out] (60 s) |

Tunable 변수(`setVariable`): `Q_zone_target`, `T_zone_target`, `T_out_target`, `USE_EXTERNAL_TARGET`(0: 내부 가상건물, 1: 외부 master/EnergyPlus).

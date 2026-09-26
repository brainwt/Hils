# HILS 인터페이스 사양 (air-enthalpy 결합)

단일 소스는 `config/hils_config.json`입니다. Python(`hils.config`)과 MATLAB(`hils_params.m`)이 같은 파일을 읽습니다. MATLAB Function 블록용 `hils_params_const.m`은 `hils_write_params_const(season)`으로 생성하며, 일치 여부는 테스트(`t_params_const_in_sync`)가 확인합니다.

## 1. 루프 순서 (한 건물 구간 k → k+1)

```
 구간 k (60 s, 1 s 샘플)
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ 히트펌프 실내기 (native 제어: 리턴공기 온도 → 자기 설정온도)             │
 │   토출 T_s, RH_s  /  리턴 T_r, RH_r  /  노즐 풍량 V  /  대기압 P          │
 └───────────────┬──────────────────────────────────────────────────────────┘
                 │ PLC 40001~40015  (1 s)
                 ▼
 ① air-enthalpy : m = V/(3600 v_s),  Q_sens = m cp (T_s−T_r),  Q_tot = m (h_s−h_r),
                  Q_lat = Q_tot − Q_sens,  m_w = m (W_s − W_r)          → 구간 평균
 ② 가상 존 (60 s): 존 공기 T_z, W_z + 구조체 Tm 적분, 입력 = 평균 Q_sens, m_w, 기상, 내부발열
                  출력 = 존 공기 상태 (T_z, RH_z) at k+1
 ③ Supervisor   : Sequence++, 범위·변화율 제한 (0.05 K/s, 0.2 %/s), 인터록, 상태머신
                 │ PLC 40100~40106
                 ▼
 ④ 실내측 챔버  : PLC 국부 PI (가열·냉각 ±10 kW, 가습·제습 ±3 g/s)가 챔버 공기를 T_sp, RH_sp로 유지
                 → 이 공기가 실내기 리턴공기가 됨 → 구간 k+1 측정
```

- 존 공기는 **가상**(모델 상태변수)이고, 챔버 공기는 그 상태를 **재현**합니다.
- 히트펌프 운전모드와 설정온도는 제품 리모컨에서만 정합니다(native control 보존). 레지스터 맵에는 해당 항목이 없습니다.
- t = 0에는 존 초기상태를 그대로 설정값으로 보냅니다(적분 없음).

## 2. 통신

| 항목 | 값 |
|------|----|
| 프로토콜 | Modbus TCP, Holding Registers, Unit ID 1 |
| 주소 | 40001 = 프로토콜 오프셋 0 (Python·Simulink 블록) = MATLAB `modbus()` 주소 1 |
| 읽기 | FC03, 오프셋 0부터 15 words (40001~40015), 1 s |
| 쓰기 | FC16, 오프셋 99부터 7 words (40100~40106), 1 s |

## 3. 레지스터 맵

인코딩: `raw = round(value × scale)`. 반올림은 half away from zero이고, signed는 int16 2의 보수입니다. 범위를 넘으면 포화시킵니다.

| 주소 | 신호 | 방향 | 스케일 | 형 | 설명 |
|------|------|------|--------|----|------|
| 40001 | T_supply | PLC→SIM | ×100 | int16 | 실내기 토출 건구온도 [°C] |
| 40002 | RH_supply | PLC→SIM | ×100 | uint16 | 토출 상대습도 [%] |
| 40003 | T_return | PLC→SIM | ×100 | int16 | 실내기 리턴(흡입) 건구온도 [°C] |
| 40004 | RH_return | PLC→SIM | ×100 | uint16 | 리턴 상대습도 [%] |
| 40005 | V_air | PLC→SIM | ×1 | uint16 | 노즐 측정 체적풍량 [m³/h] (토출측 공기 기준) |
| 40006 | P_atm | PLC→SIM | ×100 | uint16 | 대기압 [kPa] |
| 40007 | T_chamber | PLC→SIM | ×100 | int16 | 실내측 챔버 대표 온도 [°C] (인터록용) |
| 40008 | RH_chamber | PLC→SIM | ×100 | uint16 | 챔버 대표 습도 [%] |
| 40009 | T_outdoor | PLC→SIM | ×100 | int16 | 실외측 챔버 온도 [°C] |
| 40010 | RH_outdoor | PLC→SIM | ×100 | uint16 | 실외측 챔버 습도 [%] |
| 40011 | P_HP | PLC→SIM | ×1 | uint16 | 히트펌프 소비전력 [W] |
| 40012 | HP_status | PLC→SIM | code | uint16 | 0 OFF, 1 HEATING, 2 COOLING, 3 DEFROST, 9 FAULT |
| 40013 | AckSequence | PLC→SIM | ×1 | uint16 | PLC가 적용 중인 설정값 프레임의 Sequence |
| 40014 | PLC_status | PLC→SIM | bit | uint16 | bit0 READY, 1 FAULT, 2 ESTOP, 3 TRACKING, 4 WATCHDOG |
| 40015 | PLC_heartbeat | PLC→SIM | ×1 | uint16 | PLC 스캔마다 +1 |
| 40100 | T_room_SP | SIM→PLC | ×100 | int16 | 실내측 챔버 온도 설정 = 가상 존 온도 [°C] |
| 40101 | RH_room_SP | SIM→PLC | ×100 | uint16 | 실내측 챔버 습도 설정 = 가상 존 습도 [%] |
| 40102 | T_outdoor_SP | SIM→PLC | ×100 | int16 | 실외측 챔버 온도 설정 (기상) [°C] |
| 40103 | RH_outdoor_SP | SIM→PLC | ×100 | uint16 | 실외측 챔버 습도 설정 [%] |
| 40104 | Enable | SIM→PLC | ×1 | uint16 | 1 = 설정값 추종, 0 = 마지막 설정값 유지(hold) |
| 40105 | Sequence | SIM→PLC | ×1 | uint16 | 건물 구간 번호 1..65535 순환 |
| 40106 | SIM_heartbeat | SIM→PLC | ×1 | uint16 | 1 s마다 +1 (PLC watchdog용) |

## 4. Air-enthalpy 계산 (ASHRAE 37 / KS C 9306 방식)

| 양 | 식 | 비고 |
|----|----|------|
| 절대습도 | W = 0.621945 p_w / (P − p_w), p_w = RH·p_ws(T) | p_ws: Hyland–Wexler (ASHRAE 2017) |
| 엔탈피 | h = 1.006 T + W (2501 + 1.86 T) [kJ/kg_da] | |
| 비체적 | v = 0.287042 (T+273.15)(1+1.607858 W)/P [m³/kg_da] | 노즐 = 토출측, v_s 사용 |
| 건공기 유량 | m = V / 3600 / v_s [kg/s] | |
| 현열 | Q_sens = 1000 m (1.006 + 1.86 W_r)(T_s − T_r) [W] | 난방 +, 냉방 − |
| 전열 | Q_tot = 1000 m (h_s − h_r) | |
| 잠열 | Q_lat = Q_tot − Q_sens | 제습 시 − |
| 수분 | m_w = m (W_s − W_r) [kg/s] | 존 수분수지 입력 |

가상 존에는 구간 평균 Q_sens와 m_w가 들어갑니다(잠열은 수분수지로 반영).

## 5. Sequence / Ack

- PLC는 설정값 프레임(7 words)을 한 번에 적용하고, 적용한 프레임의 Sequence를 AckSequence로 돌려줍니다.
- Simulink는 미확인 시퀀스를 모두 추적하고(최대 64), 시퀀스마다 설정값(T, RH)을 기억합니다. 챔버 추종오차는 **ack된 프레임의 설정값** 기준으로 계산합니다.
- 가장 오래된 미확인 시퀀스가 180 s를 넘으면 FAULT_ACK_TIMEOUT입니다.

## 6. 상태머신

| 코드 | 상태 | Enable | 전이 |
|------|------|--------|------|
| 0 | INIT | 0 | → WAIT_PLC |
| 1 | WAIT_PLC | 0 | PLC READY → STABILIZING / 60 s 초과 → SAFE_STOP |
| 2 | STABILIZING | 1 | \|T_r − T_sp\| < 0.3 K **그리고** \|RH_r − RH_sp\| < 3 %를 120 s 유지 → RUN / 1800 s 초과 → SAFE_STOP |
| 3 | RUN | 1 | **연구 데이터 유효 구간.** 존 상태가 한 구간에 1 K 또는 10 % 넘게 바뀌면 → STEP_CHANGE |
| 4 | STEP_CHANGE | 1 | → STABILIZING |
| 5 | SAFE_STOP | 0 | fault 해제 후 60 s → WAIT_PLC (자동 재시작). **재시작 3회 초과 시 잠김** → 운전자 `reset()` |

SAFE_STOP 중에도 가상 존은 측정 열량으로 계속 적분합니다. 다만 해당 구간은 `valid = 0`입니다. PLC는 마지막 설정값을 유지하고, 히트펌프는 native 운전을 계속합니다.

## 7. Safety interlock (fault bitmask)

| bit | 값 | 조건 |
|-----|----|------|
| 0 | 1 | T_chamber ∉ [5, 40] °C |
| 1 | 2 | RH_chamber ∉ [5, 95] % |
| 2 | 4 | HP_status = 9 |
| 3 | 8 | PLC FAULT, ESTOP 또는 WATCHDOG |
| 4 | 16 | PLC_heartbeat가 10 s 이상 변하지 않음 |
| 5 | 32 | 미확인 Sequence가 180 s 초과 |
| 6 | 64 | P_HP > 5000 W |
| 7 | 128 | 풍량 > 3000 m³/h, 또는 운전 중(HEATING/COOLING) 풍량 < 100 m³/h |
| 8 | 256 | RUN 중 \|T_return − T_sp(ack)\| > 3 K가 300 s 지속 (챔버 추종 실패) |

**PLC 측 watchdog (필수 구현):** SIM_heartbeat가 10 s 이상 변하지 않으면 PLC가 추종을 멈추고 마지막 설정값을 유지합니다. 이때 bit4 WATCHDOG를 세웁니다.

## 8. Simulink 모델 신호 (To Workspace)

| 변수 | 내용 |
|------|------|
| `hils_meas` | 40001~40015 공학단위 (15) |
| `hils_cmd` | [T_room_SP RH_room_SP T_outdoor_SP RH_outdoor_SP Enable Sequence SIM_heartbeat] |
| `hils_status` | [state fault pending ack_age latency valid eT eRH T_ref RH_ref] |
| `hils_air` | [Q_sens Q_lat Q_tot m_w] 1 s air-enthalpy |
| `hils_zone` | [T_z RH_z T_out RH_out Q_sens_int m_w_int] (_int = 직전 구간 평균) |

Tunable 변수(`setVariable`): `T_zone_ext`, `RH_zone_ext`, `T_out_ext`, `RH_out_ext`, `USE_EXTERNAL_ZONE`(0: 내장 가상 존, 1: 외부 master/EnergyPlus).

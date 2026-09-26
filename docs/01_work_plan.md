# HILS 히트펌프 평가 — 구체화된 작업계획 (air-enthalpy 결합)

> 목적: 실제 히트펌프 제품을 **air-enthalpy 법**으로 평가하면서, 그 제품이 가상 건물 안에서 운전되는 것과 같은 조건을 만든다.
> 원칙: 히트펌프 **native control 보존**. Simulink는 압축기, EEV, 팬이나 제품 설정온도를 명령하지 않습니다. 실내측 챔버 공기를 가상 존 상태로 맞출 뿐입니다.

## 0. 루프 구성 (개정)

| 단계 | 누가 | 입력 | 출력 |
|------|------|------|------|
| ① 측정 | PLC (air-enthalpy 장치) | 실내기 토출·리턴 T/RH, 노즐 풍량, 대기압 | 레지스터 40001~40015 |
| ② 열량 산정 | Simulink 1 s | 측정값 | Q_sens, Q_lat, Q_tot, m_w (구간 평균) |
| ③ 가상 존 | Simulink 60 s (EnergyPlus 교체 지점) | 구간 평균 Q_sens, m_w, 기상, 내부발열 | 다음 시각 존 공기 T_z, RH_z |
| ④ 설정값 | Simulink 1 s | 존 상태 | T_room_SP, RH_room_SP (범위·변화율 제한), Sequence |
| ⑤ 재현 | PLC 국부 PI | 설정값 | 챔버 가열·냉각·가습·제습 → 실내기 리턴공기 |

### 이전 구조에서 바뀐 점

| 항목 | 이전 (폐기) | 현재 |
|------|-------------|------|
| 시뮬레이션 입력 | 챔버 실측 온도 | **실내기에서 air-enthalpy로 측정한 열량**(현열 + 수분) |
| 시뮬레이션 출력 | 목표 부하 W | **다음 시각 실내 공기 상태** (T, RH) |
| 챔버 제어 신호 | 부하장치 W 명령 | **실내측 챔버 공기 상태 설정값** |
| 존 공기 노드 | 실측 (챔버) | 가상 (모델 상태변수) |
| Simulink 제어기 | 부하 PI + 지연보상 | 없음. 추종은 PLC 국부 PI, Simulink는 설정값 생성과 감시만 |
| 습도 | 감시만 | 존 수분수지 + 챔버 가습·제습 재현 + 잠열 평가 |
| 계절 | 난방 | 난방 + 냉방(습코일 제습) |

## 1. 시간층

| 층 | 주기 | 구현 |
|----|------|------|
| 가상 존 (EnergyPlus 자리) | 60 s (내부 10 s 적분) | `hils_zone_step.m` / `hils/zone.py` |
| Supervisor (air-enthalpy, 인터록, 상태머신, 설정값) | 1 s | `hils_supervisor_step.m` / `hils/supervisor.py` |
| PLC 측정·명령, 챔버 국부 PI | 1 s | 실물 PLC 또는 가짜 PLC(`plc_server.py`) |
| 에뮬레이터 내부 적분 | 0.1 s | `hils_plc_emulator_step.m` / `hils/emulator.py` |

## 2. 작업 분해(WBS)와 완료 기준

### Phase 1 — 통신과 측정

| ID | 작업 | 완료 기준 |
|----|------|-----------|
| 1.1 | 레지스터 맵 (측정 15, 명령 7) | 코덱 시험, 레지스터 블록 시험 (`test_registers.py`, `t_frames`) |
| 1.2 | 습공기 물성 (ASHRAE 2017) | 포화압·엔탈피·비체적·이슬점이 ASHRAE 표값과 일치 (`test_psychro.py`, `t_psychro`) |
| 1.3 | Air-enthalpy 열량 | 난방 잠열 0, 냉방 부호, Q_tot = Q_sens + Q_lat. 측정 체인 vs 플랜트 참값 ±2 % |
| 1.4 | 가짜 PLC (Modbus TCP 서버 + 챔버 + 실내기) | FC3/6/16, 예외응답, pymodbus 클라이언트 폐루프 |
| 1.5 | Sequence/Ack | 다중 미확인 시퀀스, 설정값 페이로드, timeout |

### Phase 2 — Supervisor

| ID | 작업 | 완료 기준 |
|----|------|-----------|
| 2.1 | 설정값 변화율·범위 제한 | 0.05 K/s, 0.2 %/s, 10~35 °C, 15~85 % |
| 2.2 | 인터록 9종 + PLC watchdog | 비트 단위 시험, 장애 주입 4종 |
| 2.3 | 상태머신 (T·RH 동시 추종 판정, 자동 재시작 3회 후 잠김) | 전이·잠김 시험 |
| 2.4 | Simulink 모델 자동생성 | 블록 스크립트를 Simulink와 같은 배선으로 Octave에서 실행해 검증된 경로와 일치. ⚠ `.slx` 생성은 MATLAB에서 확인 |

### Phase 3 — 가상 존 / EnergyPlus

| ID | 작업 | 완료 기준 |
|----|------|-----------|
| 3.1 | 가상 존 (존 공기 T·W + 구조체, 침기, 일사, 재실) | 정상상태 열수지 0.1 % 이내 |
| 3.2 | 계절 시나리오 (겨울 난방, 여름 냉방) | 24 h: 추종 RMSE < 0.1 K / 1 %, 존 설정온도 유지, 여름 SHR 0.6~0.95 |
| 3.3 | 이상적 결합 기준해석과 비교 | 존 온도 RMSE < 0.1 K, 에너지 차 < 2 % |
| 3.4 | MATLAB master (cosim: 60 s 정지 → 외부 존 모델) | ⚠ MATLAB R2024a+에서 `ex03_cosim_master.m` |
| 3.5 | EnergyPlus/FMU 교체 | `ZoneFcn(k, t, q) → zs` 인터페이스 구현 |

### 공통 — 검증

| ID | 작업 | 완료 기준 |
|----|------|-----------|
| V1 | Python ↔ MATLAB 교차검증 | 겨울·여름·지연 90 s에서 레지스터 값 완전 일치 |
| V2 | 지연 연구 (0~120 s) | 결합 충실도(존 온도, 에너지) 표 |
| V3 | 장애 주입 | 트립 → 자동 복귀, 잠김 |

## 3. 실물 적용 절차

1. PLC 프로그램: 레지스터 맵, 프레임 단위 적용 후 Ack, watchdog(설정값 hold), 챔버 T·RH 국부 PI를 구현합니다.
2. 측정계: 노즐 풍량은 **토출측 공기 상태 기준 체적유량**으로 보냅니다. 노즐 앞 온도·습도가 토출 센서와 다르면 노즐 위치 값을 따로 받도록 확장합니다.
3. 가짜 PLC로 리허설: `python -m hils.plc_server --season summer` 후 `ex04_script_master_modbus.m`.
4. `build_HILS_Controller('Season','summer','PlcIo','modbus','Host',…)`로 모델을 생성한 뒤, Modbus 블록 대화상자를 확인합니다.
5. 챔버 추종 성능을 확인합니다(설정값 계단 1 K / 10 %). 목표는 추종 RMSE 0.1 K / 1 % 이내입니다.
6. 히트펌프 운전모드와 설정온도는 리모컨에서 설정합니다.

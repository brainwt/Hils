# HILS 히트펌프 제어 구현 — 구체화된 작업계획

> 대상: 가상건물 – Simulink(HILS supervisor) – PLC – 챔버(realization system) – 실제 히트펌프 폐루프
> 원칙: **히트펌프 native control 보존.** Simulink는 압축기 주파수, EEV, 팬을 직접 명령하지 않습니다. 가상건물의 존부하를 챔버에서 재현하는 일만 하고, 히트펌프는 자체 제어기로 대응합니다.

## 0. 원안에서 구체화·변경한 설계 결정

| # | 원안 | 구체화 결과 | 이유 |
|---|------|-------------|------|
| D1 | 가상건물이 `Qzone,target`, `Tzone,target` 출력 | 존 **공기 노드는 실측 챔버온도(T_indoor)** 를 쓰고, 외피/구조체 질량 노드만 가상(2R2C)으로 계산한다. 가상건물의 출력은 `Q_target`, 입력은 `T_indoor`(측정) | 존 공기온도를 가상과 실물 양쪽에 두면 둘이 서로 어긋납니다. 이 방식은 히트펌프 자체 온도조절기가 보는 온도와 건물모델이 쓰는 온도가 같은 값이라 결합이 일관됩니다. `Q_HP` 측정값은 에너지 수지 검증에 씁니다 |
| D2 | `Qload` 부호 미정 | `Q_target > 0` = 난방부하 = 챔버 부하장치가 **열을 제거** | 겨울철 난방 시험 기준 |
| D3 | PI → Rate limiter → Saturation | 1 s 층에서 **feedforward(Q_target) + PI**를 쓰고, 조건부 적분 anti-windup을 둔다. Simulink에는 동일한 한계의 Rate Limiter/Saturation 블록을 **2차 보호**로 한 번 더 둔다 | 부하장치 이득/바이어스 오차 보정, 포화 시 wind-up 방지 |
| D4 | Sequence/Ack | 오차를 계산할 때 **ack된 프레임의 목표값(Q_ref)** 과 측정값을 비교한다. 미확인 시퀀스는 여러 개를 동시에 추적한다 | 지연이 건물 timestep(60 s)보다 길면 단일 추적이 영구 pending이 됩니다(시험 중 발견, 보고서 §4 E1) |
| D5 | 1–2분 지연 연구 | 측정 ack 지연으로 PI 게인을 **SIMC 규칙으로 자동 디튜닝** | 고정 게인은 지연 ≥ 90 s에서 진동하고 SAFE_STOP에 들어갑니다(보고서 §3.2) |
| D6 | Safety interlock (Simulink) | Simulink 인터록 7종에 더해 **PLC 측 watchdog**(SIM_heartbeat)으로 이중화 | Simulink→PLC 쓰기가 끊기면 PLC가 마지막 부하 명령을 계속 유지하는 문제(보고서 §4 E4) |
| D7 | 레지스터 40001~40006, 40100~40103 | 40007 `Q_load_meas`, 40008 `AckSequence`, 40009 `PLC_status`, 40010 `PLC_heartbeat`, 40104 `SIM_heartbeat`, 40105 `T_outdoor_SP` 추가 | 부하 재현 피드백, ack, 통신 감시, 실외측 챔버 설정 |

## 1. 시스템 구성과 시간층

| 층 | 주기 | 구현 |
|----|------|------|
| Virtual building | 60 s | `hils_building_step.m` / `hils/building.py` (2R2C, EnergyPlus/FMU 교체 지점) |
| HILS supervisory (Sequence 발행) | 60 s | Simulink `Supervisor_60s` / `hils_supervisor_building.m` |
| Realization controller + safety + state machine | 1 s | Simulink `Realization_1s` / `hils_supervisor_step.m` |
| PLC 측정·명령 | 1 s | Modbus TCP (실물 PLC 또는 가짜 PLC 서버) |
| Plant 내부 적분 (에뮬레이터) | 0.1 s | `hils_plc_emulator_step.m` / `hils/emulator.py` |
| Logging | 1 s | Simulink To Workspace / CSV |

## 2. 작업 분해(WBS)와 완료 기준

### Phase 1 — MATLAB/Simulink ↔ Modbus TCP ↔ PLC

| ID | 작업 | 산출물 | 완료 기준 (검증 방법) |
|----|------|--------|-----------------------|
| 1.1 | 레지스터 맵과 스케일링 확정 | `config/hils_config.json`, `docs/02_interface_spec.md` | 설계문서 예시(2357 → 23.57 °C), 음수 2의 보수, overflow 포화 (`test_registers.py`, `t_codec`) |
| 1.2 | 가짜 PLC(Modbus TCP 서버 + 챔버/히트펌프 에뮬레이터) | `hils/modbus_server.py`, `hils/plc_server.py` | FC3/6/16 정상 응답, 예외 응답 (`test_raw_protocol_and_exceptions`) |
| 1.3 | Simulink/MATLAB I/O | `build_HILS_Controller.m`(Modbus 블록), `hils_io_modbus.m` | pymodbus 클라이언트로 왕복 확인 (`test_modbus_register_exchange_and_closed_loop`) |
| 1.4 | Sequence/Ack 프로토콜 | `hils_delay_monitor_step.m`, `delay_monitor.py` | 지연 측정, 다중 미확인 시퀀스, 65535→1 wrap, timeout |

### Phase 2 — Supervisor 제어 로직

| ID | 작업 | 산출물 | 완료 기준 |
|----|------|--------|-----------|
| 2.1 | Feedforward + PI + Rate limiter + Saturation + anti-windup | `hils_pi_step.m`, `control.py` | 포화 중 적분 정지, 바이어스 플랜트 정상상태 오차 < 1 W |
| 2.2 | Safety interlock (7종 bitmask) | `hils_safety_check.m`, `safety.py` | 비트별 단위시험, 주입 시험 (heartbeat, E-stop, 쓰기 두절) |
| 2.3 | Delay monitor + 지연보상 게인 | `hils_delay_gains.m` | 지연 3~120 s에서 SAFE_STOP 0회 |
| 2.4 | State machine (INIT→WAIT_PLC→STABILIZING→RUN→STEP_CHANGE, SAFE_STOP) | `hils_state_machine_step.m` | 전이 단위시험, 계단 3회에서 STEP_CHANGE 3회 |
| 2.5 | Simulink 모델 자동생성 | `build_HILS_Controller.m` → `HILS_Controller.slx` | ⚠ MATLAB 환경에서 `ex02_build_simulink_model.m` 실행 (보고서 §5) |

### Phase 3 — 가상건물(EnergyPlus/FMU) 연동

| ID | 작업 | 산출물 | 완료 기준 |
|----|------|--------|-----------|
| 3.1 | 가상건물 모델(2R2C, EnergyPlus 대체) | `hils_building_step.m`, `building.py` | 24 h 폐루프: 에너지 오차 < 1 %, RUN 비율 > 99 % |
| 3.2 | MATLAB co-simulation master (`step(sm, PauseTime)`) | `hils_master.m` (cosim/continuous) | ⚠ MATLAB R2024a+에서 `ex03_cosim_master.m` 실행 |
| 3.3 | Simulink 없는 스크립트 master | `hils_run_realtime.m` | 오프라인 경로와 비트 단위 동일 (`t_io_emulator_equivalence`) |
| 3.4 | EnergyPlus/FMU 교체 | `BuildingFcn` 인터페이스 `@(k,t,T_indoor) -> [Q,T,Tout]` | 실물 연동 시 수행 |

### 공통 — 검증

| ID | 작업 | 완료 기준 |
|----|------|-----------|
| V1 | Python 레퍼런스 ↔ MATLAB lib 교차검증 | 3개 시나리오 7,201~14,401 스텝에서 레지스터 값 완전 일치 (`test_matlab_crosscheck.py`) |
| V2 | 지연 연구 (3/30/60/90/120 s × 게인 고정/보상) | 결과표·그림 (`ex02_delay_study.py`) |
| V3 | 장애 주입 | SAFE_STOP 진입과 자동 복귀 확인 (`ex04_fault_injection.py`) |

## 3. 실물 적용 절차 (현장 체크리스트)

1. PLC 프로그램에 `docs/02_interface_spec.md`의 레지스터 맵, ack 규칙, watchdog을 구현합니다.
2. **가짜 PLC로 먼저 시험**: `python -m hils.plc_server --port 5020`을 띄우고 `ex04_script_master_modbus.m`을 실행합니다(Simulink 없이 통신과 시퀀스만 확인).
3. `build_HILS_Controller('PlcIo','modbus','Host',<PLC IP>,'Port',502)`로 모델을 생성한 뒤, Modbus 블록 대화상자에서 주소/개수/데이터형을 확인합니다.
4. 부하장치의 이득/바이어스를 개루프 계단시험으로 측정해 `plant_gain`, `plant_tau_s`를 갱신합니다(지연보상 튜닝의 공칭 모델).
5. 측정 ack 지연을 확인하고, 필요하면 `ack_timeout_s`와 `stabilize_*`를 조정합니다.
6. 히트펌프 설정온도는 **히트펌프 리모컨에서만** 설정합니다(native control 보존). `hils_config.json`의 `emulator.hp_setpoint`는 에뮬레이터 전용 값입니다.

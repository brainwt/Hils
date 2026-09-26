# HILS 히트펌프 평가 — 결과보고서 (air-enthalpy 결합, 개정판)

작성일: 2026-09-26 · 브랜치 `claude/hils-heat-pump-control-tdlk4u`

## 1. 요약

- **루프 순서를 전면 수정했습니다.** 실제 제품을 air-enthalpy 법으로 평가하는 구조로, 실내기에서 측정한 토출·리턴 온습도, 노즐 풍량, 대기압이 열량 산정으로 들어가고, 그 열량이 가상 존 모델에 입력됩니다. 모델은 **다음 시각 실내 공기 상태**를 출력하고, 이 값이 **실내측 챔버 설정값**이 됩니다. 챔버 추종은 PLC 국부 PI가 맡습니다.
- 구현 범위
  - 습공기 물성과 air-enthalpy 열량 산정
  - 가상 존 모델: 존 공기 T·W와 구조체, 침기, 일사, 재실
  - supervisor: 설정값 생성, 인터록 9종, 자동 재시작 제한
  - 에뮬레이터: 실내측 챔버 PI, 실내기 native 제어와 습코일 모델, 실외측 챔버
  - Simulink 자동생성과 MATLAB master
- 시험 결과
  - **Python 52/52, MATLAB(Octave 8.4) 17/17 통과**
  - Python ↔ MATLAB 교차검증 3개 시나리오(겨울, 여름, 여름 지연 90 s)에서 **레지스터 값 완전 일치**
  - Simulink MATLAB Function 블록 스크립트를 Simulink와 같은 배선으로 Octave에서 실행한 결과가 검증된 경로와 **1e-12 이내로 일치**
- 24 h 결과: 챔버 추종 RMSE는 겨울·여름 모두 0.03 K / 0.30 %RH, RUN 비율 99.4 %입니다. 이상적 결합(챔버 즉시 추종, 지연 0) 대비 존 온도 RMSE는 0.03 K(겨울)·0.01 K(여름), 에너지 차는 ±0.03 %입니다.
- ⚠ 한계: 이 환경에 MATLAB이 없어 `.slx` 생성, Simulink 실행, Modbus 블록, Simulation 객체 API는 **실행 검증하지 못했습니다**(§5).

## 2. 구조

```
실내기(native) ─ 토출/리턴 T·RH, 노즐 V, P ─► PLC 40001~40015
      ▲                                              │ 1 s
      │ 리턴공기 = 챔버 공기                          ▼
실내측 챔버 ◄─ PLC 국부 PI ◄─ 40100~40106 ◄─ Supervisor ◄─ 가상 존 (60 s)
 (가열·냉각·가습·제습)        T_sp, RH_sp       air-enthalpy    입력: 평균 Q_sens, m_w
                                                  인터록·상태      출력: T_z, RH_z (k+1)
```

| 구성 | 파일 (MATLAB / Python) |
|------|------------------------|
| 습공기 물성 | `hils_psy_*.m` (8개) / `psychro.py` |
| Air-enthalpy 열량 | `hils_air_enthalpy.m` / `airside.py` |
| 가상 존 | `hils_zone_init/step/state.m` / `zone.py` |
| Supervisor | `hils_supervisor_init/interval/building/step.m` / `supervisor.py` |
| 설정값 제한 | `hils_setpoint_step.m` / `setpoint.py` |
| 인터록, 상태머신, 지연감시 | `hils_safety_check.m`, `hils_state_machine_step.m`, `hils_delay_monitor_*.m` |
| 에뮬레이터 (PLC + 챔버 + 실내기) | `hils_plc_emulator_*.m`, `hils_coil_outlet.m` / `emulator.py` |
| Simulink 생성 | `build_HILS_Controller.m` (블록: `HILS_Core_1s`, `Encode_Cmd`, `PLC_Emulator`/Modbus, `Meas_Delay`, `Decode_Meas`) |
| Master | `hils_master.m` (Simulation 객체), `hils_run_realtime.m` (스크립트), `cosim.py` |

## 3. 시험 결과

### 3.1 시험 목록

| 구분 | 파일 | 개수 | 결과 |
|------|------|------|------|
| 레지스터 코덱 | `test_registers.py` | 12 | 통과 |
| 습공기, air-enthalpy | `test_psychro.py` | 9 | 통과 |
| 가상 존 (수동 거동, 가열 응답, 정상상태 열수지) | `test_zone.py` | 3 | 통과 |
| 지연감시, 인터록, 설정값, 상태머신(잠김 포함) | `test_logic.py` | 9 | 통과 |
| 폐루프 (측정 정확도, 계절 24 h, 기준 대비 충실도, 지연, 계단, 장애 5종) | `test_closed_loop.py` | 14 | 통과 |
| Modbus TCP (pymodbus 폐루프, 원시 소켓·예외응답) | `test_modbus_loopback.py` | 2 | 통과 |
| **Python ↔ MATLAB 교차검증** | `test_matlab_crosscheck.py` | 3 | 통과 |
| MATLAB 단위·폐루프·**Simulink 블록 스크립트** | `matlab/tests/run_all_tests.m` | 17 | 통과 |

로그: `results/pytest_stdout.txt`, `results/matlab_tests_stdout.txt`

### 3.2 Ex.1 — 계절별 24 h (`results/ex01_winter_24h.png`, `ex01_summer_24h.png`)

| 지표 | 겨울 (난방, 외기 −5~5 °C) | 여름 (냉방, 외기 26~34 °C, 55 %RH) |
|------|--------------------------|-----------------------------------|
| RUN 비율 | 99.4 % | 99.4 % |
| 챔버 추종 RMSE T / RH | 0.032 K / 0.30 % | 0.031 K / 0.30 % |
| 챔버 추종 최대 오차 | 0.19 K | 0.16 K |
| 가상 존 온도 평균 (범위) | 21.99 °C (19.98~22.43) | 25.00 °C (24.33~27.00) |
| 가상 존 습도 평균 | 34.4 % | 59.7 % |
| 히트펌프 열량 (air-enthalpy) | 난방 99.5 kWh | 냉방 60.7 kWh, SHR 0.84 |
| 소비전력 / COP·EER | 28.5 kWh / 3.49 | 13.5 kWh / 4.51 |
| 이상적 결합 대비 존 온도 RMSE | 0.029 K | 0.013 K |
| 이상적 결합 대비 에너지 | −0.03 % | +0.03 % |

- 여름 한낮에는 습코일 제습으로 잠열이 최대 약 −1.2 kW 나오고, 가상 존 습도가 52 %까지 내려갑니다. 밤에는 건코일(잠열 0)로 바뀝니다.
- 존 온도는 히트펌프 자체 온도조절기(난방 22 °C, 냉방 25 °C)가 유지합니다. 제품은 가상 존 상태가 재현된 리턴공기만 보고 운전합니다.

### 3.3 Ex.2 — 통신 지연의 영향 (12 h, 이상적 결합 대비) · `results/ex02_delay_sweep_*.png`

| 계절 | 지연 [s] | 존 온도 RMSE [K] | 최대 [K] | 에너지 차 [%] | 추종 RMSE T [K] | RUN 비율 | SAFE_STOP |
|------|---:|---:|---:|---:|---:|---:|---:|
| 겨울 | 0 | 0.042 | 0.31 | −0.03 | 0.033 | 0.988 | 0 |
| 겨울 | 30 | 0.042 | 0.29 | −0.01 | 0.034 | 0.988 | 0 |
| 겨울 | 60 | 0.046 | 0.28 | 0.00 | 0.034 | 0.988 | 0 |
| 겨울 | 90 | 0.054 | 0.30 | +0.01 | 0.035 | 0.987 | 0 |
| 겨울 | 120 | 0.066 | 0.39 | +0.02 | 0.036 | 0.988 | 0 |
| 여름 | 0 | 0.018 | 0.09 | −0.10 | 0.032 | 0.989 | 0 |
| 여름 | 30 | 0.021 | 0.08 | 0.00 | 0.032 | 0.990 | 0 |
| 여름 | 60 | 0.030 | 0.14 | +0.11 | 0.033 | 0.989 | 0 |
| 여름 | 90 | 0.043 | 0.20 | +0.21 | 0.033 | 0.990 | 0 |
| 여름 | 120 | 0.058 | 0.25 | +0.33 | 0.033 | 0.989 | 0 |

해석:
- 지연이 커지면 가상 존 상태가 챔버에 늦게 재현됩니다. 히트펌프는 과거 존 상태에 반응하게 되므로 존 온도 오차가 커집니다(120 s에서 0.06~0.07 K).
- 이 구조에서는 Simulink에 피드백 제어기가 없어서, 이전 판(부하 PI)처럼 지연 때문에 진동하는 일은 없습니다.
- 지연 0에서도 남는 오차(겨울 0.04 K)는 챔버의 실제 동특성과 센서 노이즈 때문입니다.

### 3.4 Ex.3 — Modbus TCP 루프백 (여름 30 분, 20배속)

PLC 스캔 1,802회, Modbus 요청 3,602회에서 **오류 0**, 스캔 overrun 0이었습니다. ack 지연은 4.0 s(에뮬레이터 3 s + 1 스캔), 추종 RMSE는 0.047 K / 0.37 %RH, fault는 0입니다.

### 3.5 Ex.4 — 장애 주입 (겨울 2 h) · `results/ex04_transitions.json`

| 주입 | 시각 | 트립 | 원인 | RUN 복귀 |
|------|------|------|------|----------|
| PLC heartbeat 정지 100 s | 900 s | 911 s | heartbeat lost | 1181 s |
| E-stop 100 s | 2100 s | 2101 s | PLC E-stop | 2381 s |
| Simulink→PLC 쓰기 두절 300 s | 3300 s | 3311 s | PLC watchdog | 3784 s |
| 챔버 공조 용량 10 → 0.6 kW | 4800 s | 6481 s | 챔버 추종 실패 (3 K, 300 s) | 복귀 시도 중 |

별도 시험(`test_chamber_capacity_shortage_latches_after_restarts`)에서, 용량이 처음부터 부족하면 안정화 실패를 반복하다 **자동 재시작 3회 후 잠김**을 확인했습니다.

## 4. 발견한 오류와 수정

| # | 증상 (발견 경위) | 원인 | 수정 | 검증 |
|---|------------------|------|------|------|
| **R1** | 루프 순서가 제품 평가 방식과 다름 (사용자 지적) | 가상건물이 부하 W를 계산하고 챔버가 그 부하를 재현하는 구조였음. air-enthalpy로 제품 능력을 재는 방식과 맞지 않음 | 전면 재설계 (§2): 측정 열량 → 가상 존 → 공기 상태 → 챔버 설정값 | 전체 시험 재작성 |
| R2 | 여름 시나리오에서 존이 20 °C까지 떨어지고 히트펌프가 켜지지 않음 (1차 실행) | 구조체 초기온도 18 °C(겨울값)가 여름에도 쓰여 차가운 구조체가 존을 냉각 | 계절별 초기조건 함수 (`season_overrides` / `hils_params(…, season)`) | 여름 24 h 존 25.0 °C |
| R3 | 현열 + 잠열이 전열과 0.2 % 불일치 (단위시험) | 잠열을 h_fg ΔW로 따로 계산해 수증기 현열 항이 빠짐 | 잠열 = 전열 − 현열 (ASHRAE 37 관례) | `test_air_enthalpy_cooling_signs` 1e-12 |
| **R4** | 챔버 용량 부족 시 SAFE_STOP → 60 s 후 재시작 → 30분 뒤 다시 SAFE_STOP이 무한 반복 (폐루프 시험) | 자동 재시작 횟수 제한이 없었음 | 3회 초과 시 SAFE_STOP 잠김, 운전자 `reset()`으로 해제. RUN에 도달하면 횟수 초기화 | `test_state_machine_latch`, `t_state_machine_latch` |
| R5 | 시험 기대값 오류 2건 | 첫 WAIT_PLC 전이를 셈에 넣음. ack timeout 경계 계산 착오 | 기대값 수정 (코드 수정 없음) | — |

이전 판에서 이어받은 수정:
- 미확인 시퀀스 다중 추적
- MATLAB/PLC와 같은 반올림(half away from zero)
- PLC watchdog과 그 비트에 의한 즉시 트립
- 의존성 없는 Modbus 서버

이전 판의 부하 PI와 지연보상 게인은 새 구조에 필요 없어 삭제했습니다.

## 5. 검증 범위와 한계

| 항목 | 상태 |
|------|------|
| MATLAB 제어·물리 로직 (`matlab/lib`) | ✅ Octave 실행 + Python 교차검증 |
| Simulink MATLAB Function 블록 스크립트 4종 | ✅ 스크립트를 내보내 같은 배선으로 Octave 실행 (`t_simulink_block_scripts`) |
| `build_HILS_Controller.m`의 블록 배치·배선·`.slx` 저장 | ⚠ 미실행 (MATLAB 없음). `ex02_build_simulink_model.m`이 Simulink 결과와 스크립트 결과를 자동 비교 |
| Modbus Client Read/Write 블록 | ⚠ 미실행. 라이브러리 경로와 파라미터 이름을 검색하는 방식 |
| `hils_master.m` (Simulation 객체, 정지 중 로그 읽기) | ⚠ 미실행. 정지 중 `hils_air` 로그 접근은 릴리스별 fallback 포함 |
| 실물 PLC·챔버·제품 | ⚠ 에뮬레이터로 대체 |

모델과 측정의 한계:
- 노즐 풍량은 토출 센서 상태로 환산합니다. 실제 설비에서 노즐 위치 온도·습도가 다르면 해당 센서를 레지스터에 추가해야 합니다.
- 제상(DEFROST), 실내기 팬 가변, 배관·덕트 열손실, 챔버 성층은 모델링하지 않았습니다.
- 가상 존은 합성 기상의 단일 존 모델입니다. EnergyPlus/FMU는 `ZoneFcn(k, t, q) → zs`로 연결합니다.
- 에뮬레이터 파라미터(챔버 용량, PI 게인, 코일 bypass factor 0.15)는 가정값입니다.

## 6. 다음 단계

1. MATLAB에서 `cd matlab; addpath tests; run_all_tests`와 `examples/ex02_build_simulink_model`을 실행합니다(Simulink = 스크립트 확인).
2. 실제 PLC에 챔버 T·RH 국부 PI, 프레임 Ack, watchdog hold를 구현합니다.
3. 챔버 추종 시험(설정값 계단)으로 PI 튜닝을 확인하고, 추종 RMSE 0.1 K / 1 %를 목표로 합니다.
4. 가짜 PLC로 Modbus 블록 파라미터를 확정한 뒤 실물로 전환합니다.
5. EnergyPlus/FMU를 `ZoneFcn`으로 연결합니다.

## 부록 — 재현

```bash
cd python && python -m pytest -v tests                  # 52 tests (octave 있으면 교차검증 포함)
python examples/ex01_offline_24h.py                     # 저장소 루트에서, ex01~ex04
cd matlab && octave-cli --eval "addpath('lib'); addpath('tests'); run_all_tests"
```

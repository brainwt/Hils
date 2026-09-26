# HILS 히트펌프 제어 구현 — 결과보고서

작성일: 2026-09-25 · 브랜치 `claude/hils-heat-pump-control-tdlk4u`

## 1. 요약

- 작업계획(원안 1~12절)을 WBS와 완료 기준으로 구체화했습니다([01_work_plan.md](01_work_plan.md)). 레지스터 맵, Sequence/Ack, 상태머신, 인터록 사양도 확정했습니다([02_interface_spec.md](02_interface_spec.md)).
- 구현물은 다음과 같습니다.
  1. Simulink 모델 자동생성 스크립트 `build_HILS_Controller.m`
  2. Simulation 객체 master `hils_master.m` (cosim/continuous)
  3. Simulink 없이 PLC를 운전하는 스크립트 master
  4. 공용 제어 라이브러리 `matlab/lib` (23개 함수)
  5. Python 레퍼런스 구현과 **가짜 PLC Modbus TCP 서버**
- 시험 결과는 **Python 46/46, MATLAB(Octave 8.4) 12/12 통과**입니다. Python 레퍼런스와 MATLAB 라이브러리는 3개 시나리오에서 **스텝별 레지스터 값이 완전히 일치**합니다(명령 차이 < 1e-6 W).
- 시험 중 오류와 설계 결함 **5건**을 발견해 수정했습니다(§4). 가장 중요한 것은 E1입니다. PLC 지연이 건물 timestep(60 s) 이상이면 Sequence 추적이 무너져 루프가 망가지는 문제였습니다.
- 지연 연구 결과, **고정 PI 게인은 지연 90 s 이상에서 진동하고 SAFE_STOP에 들어갑니다.** 지연보상(SIMC) 게인을 쓰면 **120 s까지 SAFE_STOP 없이** RUN 구간 RMSE 58 W 이하를 유지합니다.
- ⚠ 한계: 이 환경에는 MATLAB/Simulink가 없어서 **`.slx` 생성과 Simulink 실행, Modbus 블록, Simulation 객체 API는 실행 검증을 하지 못했습니다**(§5). 이 경로들이 호출하는 제어 로직(lib)은 Octave와 교차검증으로 확인했습니다.

## 2. 구현 내용

### 2.1 구조

| 계층 | 주기 | Simulink 블록 | 공용 함수 |
|------|------|---------------|-----------|
| 가상건물 (2R2C, EnergyPlus 자리) | 60 s | `Virtual_Building` / 외부 `Q_zone_target` (Switch `USE_EXTERNAL_TARGET`) | `hils_building_step` |
| Supervisor (Sequence 발행) | 60 s | `Supervisor_60s` → `RT_slow2fast` | `hils_supervisor_building` |
| Realization (heartbeat, 지연감시, 인터록, 상태머신, FF+PI) | 1 s | `Realization_1s` → `Rate_Limiter` → `Saturation` → `Encode_Cmd` | `hils_supervisor_step` 외 |
| PLC I/O | 1 s | `PLC_Emulator` 또는 `Modbus_Write_40100`/`Modbus_Read_40001` → `Meas_Delay` | `hils_plc_emulator_step` |
| 기록 | 1 s | To Workspace `hils_meas/cmd/status/target` | — |

MATLAB Function 블록은 스크립트를 직접 담지 않고 `matlab/lib`의 함수를 호출합니다. 그래서 Simulink 모델, MATLAB 스크립트 master, Octave 시험이 **같은 코드**를 실행합니다. 파라미터는 `config/hils_config.json` 하나에서 나오고, 코드생성용 상수 함수(`hils_params_const.m`)는 이 JSON에서 자동 생성하며 일치 여부를 시험합니다.

### 2.2 주요 설계

- **Native control 보존**: 히트펌프 설정온도와 운전모드는 레지스터 맵에 없습니다. 에뮬레이터의 히트펌프는 PI, 최소 off 시간 180 s, 히스테리시스 ±0.5 K를 갖는 자체 온도조절기로만 동작합니다.
- **결합 방식**: 존 공기 노드는 실측 챔버온도를 쓰고 외피/구조체 질량은 가상으로 둡니다. 가상건물은 측정한 `T_indoor`로 `Q_target`을 계산하고, 챔버 부하장치가 이 부하를 재현합니다.
- **Sequence 정합**: PI 오차는 `Q_ref − Q_meas`로 계산합니다. `Q_ref`는 AckSequence 프레임의 목표값입니다. 피드포워드는 최신 목표를 즉시 반영합니다.
- **이중 안전**: Simulink 인터록 7종(bitmask)과 PLC watchdog(SIM_heartbeat 10 s)을 함께 둡니다.

## 3. 시험 결과

### 3.1 시험 목록

| 구분 | 파일 | 개수 | 결과 |
|------|------|------|------|
| 레지스터 코덱 | `python/tests/test_registers.py` | 12 | 통과 |
| 제어기 | `test_control.py` | 7 | 통과 |
| 지연감시, 인터록, 상태머신 | `test_logic.py` | 10 | 통과 |
| 폐루프 통합 (24 h, 계단, 지연 5종, 장애 4종) | `test_closed_loop.py` | 12 | 통과 |
| Modbus TCP (pymodbus 클라이언트, 원시 소켓, 예외응답) | `test_modbus_loopback.py` | 2 | 통과 |
| **Python ↔ MATLAB 교차검증** (Octave 실행) | `test_matlab_crosscheck.py` | 3 | 통과 |
| MATLAB 단위/폐루프 | `matlab/tests/run_all_tests.m` | 12 | 통과 |

실행 로그: `results/pytest_stdout.txt`, `results/matlab_tests_stdout.txt`, `results/examples_stdout.txt`

### 3.2 예제 결과

**Ex.1 — 24 h 겨울철 1일 (가상건물 2R2C, 외기 −5~5 °C, 일사, 재실 발열)** · `results/ex01_24h.png`

| 지표 | 값 |
|------|----|
| RUN(유효) 구간 비율 | 99.77 % (시작 후 198 s만 WAIT/STABILIZING) |
| 재현오차 RMSE / MAE (RUN) | 31.2 W / 24.7 W (측정노이즈 σ = 30 W 수준) |
| 목표 대비 재현 에너지 | 83.01 → 82.97 kWh (**−0.05 %**) |
| 히트펌프 공급열 / 소비전력 | 83.9 / 23.6 kWh (일평균 COP ≈ 3.55) |
| 챔버온도 (히트펌프 native 제어) | 평균 21.99 °C, 범위 19.98~22.17 °C (시작 과도 포함) |
| ack 지연 | 4 s (PLC 3 s + 1 스캔) |
| 실행시간 | 약 6 s (24 h를 약 14,000배속) |

**Ex.2 — 지연 연구 (계단 2.0 → 3.5 → 1.5 → 3.0 kW, 30분 간격, 2 h)** · `results/ex02_delay_sweep.png`

| PLC 지연 [s] | 게인 | RMSE 활성구간 [W] | RMSE RUN [W] | RUN 비율 | 에너지 오차 [%] | SAFE_STOP [s] | 최종 Kp |
|---:|---|---:|---:|---:|---:|---:|---:|
| 3 | 고정 | 231 | 30.1 | 0.90 | −0.67 | 0 | 0.400 |
| 30 | 고정 | 314 | 30.1 | 0.85 | −1.08 | 0 | 0.400 |
| 60 | 고정 | 427 | 49.6 | 0.68 | −1.59 | 0 | 0.400 |
| 90 | 고정 | 1158 | — | **0.00** | −4.80 | **180** | 0.400 |
| 120 | 고정 | 1789 | — | **0.00** | −4.22 | **180** | 0.400 |
| 3 | 지연보상 | 231 | 30.1 | 0.90 | −0.67 | 0 | 0.400 |
| 30 | 지연보상 | 313 | 30.0 | 0.85 | −1.07 | 0 | 0.323 |
| 60 | 지연보상 | 386 | 35.5 | 0.82 | −1.63 | 0 | 0.164 |
| 90 | 지연보상 | 432 | 54.8 | 0.83 | −2.45 | 0 | 0.110 |
| 120 | 지연보상 | 470 | 58.0 | 0.82 | −3.23 | 0 | 0.083 |

해석:
- 고정 게인(Kp 0.4, Ti 20 s)은 적분시간이 루프 지연보다 훨씬 짧습니다. 그래서 지연 90 s 이상에서 주기 약 6분의 지속진동이 생기고(`ex02_delay90_comp0.png`), 30분 안정화 timeout으로 SAFE_STOP에 들어갑니다.
- 지연보상은 측정 ack 지연 θ로 Kp와 Ki를 자동으로 낮춥니다. 계단 뒤 약 4분 안에 RUN으로 복귀합니다(`ex02_delay90_comp1.png`).
- 지연이 길수록 에너지 오차가 커지는 것은 **전달지연만큼 목표가 늦게 재현되기 때문**입니다(계단마다 약 θ × ΔQ). 이는 1~2분 HILS 지연이 가상건물 에너지 수지에 주는 영향으로, 연구에서 정량화해야 할 대상입니다.

**Ex.3 — Modbus TCP 루프백 (Phase 1)** · `results/ex03_modbus.png`
가짜 PLC 서버(별도 스레드, 비동기 스캔)와 pymodbus 클라이언트로 30분 실험을 20배속으로 실행했습니다.

| 지표 | 값 |
|------|----|
| PLC 스캔 / Modbus 요청 / 스캔 overrun | 1,802 / 3,602 / 0 |
| Modbus 오류 | 0 |
| ack 지연 (평균/최대) | 4.0 / 4.0 s |
| RMSE (RUN) | 30.0 W |
| RUN 비율 / 에너지 오차 | 0.80 / −2.98 % (30분 시험이라 시작 과도와 계단 1회 비중이 큼) |

**Ex.4 — 장애 주입** · `results/ex04_faults.png`, `results/ex04_transitions.json`

| 주입 | 시각 | 감지 → SAFE_STOP | 원인 비트 | 복귀 (RUN) |
|------|------|------------------|-----------|------------|
| PLC heartbeat 정지 100 s | 900 s | 911 s (timeout 10 s) | heartbeat lost | 1259 s |
| E-stop 100 s | 2100 s | 2101 s | PLC fault/E-stop | 2461 s |
| Simulink→PLC 쓰기 두절 300 s | 3300 s | **3311 s** (PLC watchdog) | PLC watchdog | 3862 s |

SAFE_STOP 중에는 명령이 50 W/s로 0까지 ramp-down되고, PLC는 챔버를 사전조화 모드(T_chamber_SP)로 전환합니다. 히트펌프는 계속 native 운전합니다. 장애 구간에는 재현부하가 목표에서 최대 약 2.3 kW 벗어나므로, 해당 구간은 `valid = 0`으로 기록해 분석에서 제외해야 합니다.

## 4. 발견한 오류와 수정

| # | 증상 (발견 경위) | 원인 | 수정 | 검증 |
|---|------------------|------|------|------|
| **E1** | 지연 ≥ 60 s에서 ack 지연이 NaN, 적분기가 계속 정지, 정상상태 오차 약 350 W, 안정화 timeout (지연 연구 1차 실행) | DelayMonitor가 **최신 Sequence 하나만** 추적했습니다. ack가 오기 전에 다음 Sequence(60 s 후)가 나가면 영원히 일치하지 않고, `t_sent`가 계속 갱신되어 **ack timeout도 발생하지 않았습니다**(안전 기능 무력화) | 미확인 시퀀스를 모두 추적(최대 64, 고정 버퍼, codegen 호환). ack = s이면 s 이전은 모두 확인. timeout은 가장 오래된 미확인 기준 | `test_delay_monitor_multiple_outstanding`, `_timeout`, `t_delay_monitor` |
| **E2** | 지연 90~120 s에서 지속진동, SAFE_STOP (E1 수정 후) | 고정 PI 게인의 적분시간(20 s)이 루프지연보다 훨씬 짧음. 또 현재 목표와 이전 프레임 측정값을 비교하는 시퀀스 불일치 | ① 오차 기준을 ack된 프레임의 목표(`Q_ref`)로 변경 ② 측정 지연 기반 SIMC 게인 스케줄 | §3.2 표, `test_delay_robustness_with_compensation[3..120]` |
| **E3** | Python과 MATLAB의 레지스터 인코딩이 x.5에서 다를 수 있음 (코드 리뷰) | Python `round()`는 banker's rounding(2.5 → 2), MATLAB/PLC는 half away from zero(2.5 → 3) | `round_half_away()`로 통일 | `test_round_half_away_matches_matlab`, 교차검증 완전 일치 |
| **E4** | 쓰기가 끊겨도 PLC가 마지막 부하 명령을 계속 유지하고, Simulink는 121 s 뒤에야 감지 (Ex.4 그래프) | PLC 측 통신 감시가 없었음. Simulink는 ack timeout에만 의존 | PLC watchdog(SIM_heartbeat 10 s) 추가: 부하장치 해제 + `WATCHDOG` 비트. Simulink 인터록이 이 비트로 즉시 trip | 감지 3421 s → **3311 s**. `test_plc_watchdog_releases_load_on_write_loss`, `test_command_loss_triggers_ack_timeout` |
| **E5** | 가짜 PLC 서버 기동 실패 `TypeError: 0 <= address < 65535` (Modbus 시험) | pymodbus 3.15에서 서버 datastore API가 바뀜(SimData 기반, 주소 규칙 변경) | 의존성 없는 최소 Modbus TCP 서버(FC3/6/16, 예외응답) 직접 구현. 호환성은 pymodbus 클라이언트로 검증 | `test_modbus_loopback.py` 2건 |

기타 수정:
- 테스트 기대값 오류: 마지막 Sequence는 실행 종료 시점에 송신되어 ack될 수 없음 → `seq − 1`로 수정했습니다.
- MATLAB 익명함수가 상태를 복사하는 문제를 예방해, `hils_io_emulator`를 nested function으로 작성했습니다.
- codegen 크기 불일치 위험을 예방해 `find(…,1)` 결과를 `k(1)`로 스칼라화했습니다.
- Rate Transition 기본값(deterministic)이 slow → fast에서 **60 s 지연**을 넣는 문제를 예방해, single-tasking과 Integrity/Deterministic off로 설정했습니다.
- 헤드리스 Octave에서 그림이 실패하는 문제는 try/catch로 처리했습니다.

## 5. 검증 범위와 한계 (중요)

| 항목 | 상태 | 비고 |
|------|------|------|
| 제어 로직 (`matlab/lib` 23개 함수) | ✅ Octave 실행 + Python 교차검증 | Simulink MATLAB Function 블록이 그대로 호출하는 코드 |
| `hils_run_realtime.m` + `hils_io_emulator.m` | ✅ Octave | 오프라인 경로와 비트 단위 동일 |
| Modbus TCP 프로토콜 / 가짜 PLC | ✅ Python (pymodbus 클라이언트, 원시 소켓) | |
| `build_HILS_Controller.m` → `.slx` 생성, `sim()` | ⚠ **미실행** | MATLAB 없음. `matlab/examples/ex02_build_simulink_model.m`이 Simulink 결과와 스크립트 결과를 자동 비교하도록 작성되어 있음 |
| Modbus Client Read/Write 블록 배치 | ⚠ 미실행 | 라이브러리 경로와 대화상자 파라미터 이름은 릴리스마다 다를 수 있어 이름 검색과 후보 파라미터 방식으로 작성. 실패 시 안내 메시지 출력 |
| `hils_master.m` (Simulation 객체: `step(PauseTime)`, `setVariable`) | ⚠ 미실행 | R2024a+ 필요. `stop()` 반환 형식 차이는 fallback 처리 |
| MATLAB Function 코드생성 호환성 | ⚠ 정적 검토만 | 고정 크기 버퍼, 구조체 필드 순서 일치, 가변 크기 스칼라화 |
| 실물 PLC / 챔버 / 히트펌프 | ⚠ 에뮬레이터로 대체 | 부하장치 이득 0.93, 바이어스 −150 W, τ 20 s는 가정값 |

모델 차원의 한계:
- 챔버와 실험실 사이 열교환(UA 15 W/K)은 부하 측정에 포함되지 않는 외란입니다. 실물에서는 이 항을 측정하거나 보정해야 재현 에너지 정합이 유지됩니다.
- 가상건물은 2R2C 합성 기상 모델입니다. EnergyPlus/FMU를 붙이려면 `BuildingFcn(k, t, T_indoor) → [Q_target, T_target, T_out]`만 구현하면 됩니다.
- 제습과 잠열부하(RH)는 감시만 하고 재현하지 않습니다.

## 6. 다음 단계 (권장 순서)

1. **MATLAB 환경에서** `cd matlab; addpath tests; run_all_tests` 후 `examples/ex02_build_simulink_model`을 실행해 Simulink와 스크립트의 일치를 확인합니다. 차이가 나면 블록 배선이나 Rate Transition 설정 문제입니다.
2. 가짜 PLC(`python -m hils.plc_server`)에 `build_HILS_Controller('PlcIo','modbus')` 모델을 연결해 Modbus 블록 파라미터를 확정합니다(Phase 1 완료).
3. PLC 프로그램에 ack 규칙(프레임 단위 적용 후 AckSequence)과 watchdog을 구현합니다.
4. 부하장치 개루프 계단시험으로 `plant_gain`, `plant_tau_s`를 식별한 뒤, 실측 지연으로 지연보상 게인을 확인합니다.
5. EnergyPlus/FMU를 `BuildingFcn`으로 연결합니다(Phase 3). 지연에 따른 에너지 수지 오차(§3.2)를 연구 지표로 기록합니다.

## 부록 — 재현 방법

```bash
cd python && python -m pytest -v tests            # 46 tests (octave 있으면 교차검증 포함)
python examples/ex01_offline_24h.py               # 등 ex01~ex04 (저장소 루트에서)
cd matlab && octave-cli --eval "addpath('tests'); run_all_tests"
```

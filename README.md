# HILS — 가상건물 · Simulink supervisor · PLC · 실제 히트펌프 폐루프

가상건물이 계산한 존부하를 챔버(realization system)에서 재현하고, **실제 히트펌프는 자체(native) 제어기로 대응**하게 하는 HILS 제어 코드입니다. Simulink는 압축기, EEV, 팬을 직접 명령하지 않고 부하 재현만 담당하는 supervisor입니다.

```
Virtual building (2R2C / EnergyPlus, 60 s) ──Q_target──► Simulink supervisor
     ▲                                                   ├ Supervisor_60s : Sequence 발행
     │ T_indoor (measured)                               ├ Realization_1s : 안전/지연/상태머신/FF+PI
     │                                                   └ Rate limiter → Saturation → Encode
     │                                                          │ Modbus TCP (40100~40105)
     │                                                          ▼
     └──────── 40001~40010 ◄──── PLC ──► 부하장치/챔버 ──► 실제 히트펌프 (native control)
```

## 구성

| 경로 | 내용 |
|------|------|
| `config/hils_config.json` | **단일 설정 원본**: 레지스터 맵, 주기, 게인, 한계, 안전값, 에뮬레이터 |
| `matlab/build_HILS_Controller.m` | `HILS_Controller.slx` **자동 생성** (new_system/add_block/add_line, MATLAB Function, Modbus 블록) |
| `matlab/hils_master.m` | Simulation 객체 master: `cosim` (`step(sm,'PauseTime',…)`) / `continuous` (`setVariable`) |
| `matlab/hils_run_realtime.m` | Simulink 없이 스크립트로 PLC 실시간 운전 (`hils_io_modbus` / `hils_io_emulator`) |
| `matlab/lib/` | 제어 로직 순수함수. MATLAB Function 블록과 스크립트가 공용으로 씀 (codegen 호환, Octave 검증) |
| `python/hils/` | Python 레퍼런스 구현 + **가짜 PLC Modbus TCP 서버** + 챔버/히트펌프 에뮬레이터 |
| `examples/` | Python 예제 4종 (24 h, 지연 연구, Modbus 루프백, 장애 주입) |
| `matlab/examples/` | MATLAB 예제 4종 |
| `docs/` | 작업계획 · 인터페이스 사양 · **결과보고서** |
| `results/` | 예제 결과 그림/KPI (CSV는 재생성) |

## 빠른 시작

```bash
# Python (MATLAB 불필요)
pip install -r python/requirements.txt
python examples/ex01_offline_24h.py          # 24 h 폐루프 → results/ex01_24h.png
python examples/ex02_delay_study.py          # 지연 3~120 s 연구
python examples/ex03_modbus_loopback.py      # 실제 Modbus TCP 루프백
python examples/ex04_fault_injection.py      # 인터록 시험
cd python && python -m pytest -q tests       # 46 tests (Octave 있으면 MATLAB 교차검증 포함)

# 가짜 PLC 서버 단독 실행 (Simulink / MATLAB modbus() 접속용)
cd python && python -m hils.plc_server --port 5020
```

```matlab
% MATLAB (R2024a+ 권장, Modbus 블록은 R2024b+ Industrial Communication Toolbox)
cd matlab
build_HILS_Controller()                                   % 에뮬레이터 모드 모델 생성
build_HILS_Controller('PlcIo','modbus','Host','192.168.0.10','Port',502)   % 실물 PLC
out = hils_master('Mode','cosim','Duration',3600);        % 60 s 마다 정지하는 co-sim
addpath tests; run_all_tests                              % 단위/폐루프 시험 (Octave 가능)
```

설정(JSON)을 바꾼 뒤에는 `hils_write_params_const` 를 실행하세요(MATLAB Function 블록용 상수 재생성). `build_HILS_Controller`는 이 작업을 자동으로 합니다.

## 문서
- [docs/01_work_plan.md](docs/01_work_plan.md): 구체화된 작업계획, WBS, 완료 기준
- [docs/02_interface_spec.md](docs/02_interface_spec.md): 레지스터 맵, Sequence/Ack, 상태머신, 인터록
- [docs/03_result_report.md](docs/03_result_report.md): 구현·시험 결과, 발견 오류와 수정, 한계

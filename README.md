# HILS — air-enthalpy 히트펌프 평가 · 가상 존 · Simulink · PLC 폐루프

실제 히트펌프 제품을 **air-enthalpy 법**으로 평가하면서, 제품이 가상 건물 안에서 운전되는 것과 같은 조건을 만드는 HILS 코드입니다. 제품은 **자기 제어기(native control)** 로만 운전합니다.

```
 실내기(native) ─ 토출/리턴 T·RH, 노즐 풍량, 대기압 ─► PLC 40001~40015 ─► Simulink
      ▲                                                          │ ① air-enthalpy 열량 (1 s)
      │ 리턴공기                                                  │ ② 가상 존 (60 s): 다음 공기 상태
 실내측 챔버 ◄── PLC 국부 PI ◄── 40100~40106 (T_sp, RH_sp) ◄──────┘ ③ 설정값·인터록·상태머신
```

## 구성

| 경로 | 내용 |
|------|------|
| `config/hils_config.json` | **단일 설정 원본**: 레지스터 맵, 주기, 설정값 제한, 인터록, 가상 존, 기상, 에뮬레이터 |
| `matlab/build_HILS_Controller.m` | `HILS_Controller.slx` 자동 생성 (MATLAB Function 블록, PLC 에뮬레이터 또는 Modbus 블록) |
| `matlab/hils_master.m` | Simulation 객체 master: `internal` / `cosim` (60 s 정지 → 외부 존 모델 → 재개) |
| `matlab/hils_run_realtime.m` | Simulink 없이 스크립트로 PLC 실시간 운전 (`hils_io_modbus` / `hils_io_emulator`) |
| `matlab/lib/` | 습공기, air-enthalpy, 가상 존, supervisor, 에뮬레이터. 블록과 스크립트가 공용 (Octave 검증) |
| `python/hils/` | Python 레퍼런스 구현, **가짜 PLC Modbus TCP 서버**, 챔버·실내기 에뮬레이터 |
| `examples/`, `matlab/examples/` | 예제 (24 h 겨울·여름, 지연 연구, Modbus 루프백, 장애 주입) |
| `villas/hils.conf` | VILLASnode 게이트웨이 설정 초안 (Modbus ↔ UDP, 미실행) |
| `docs/` | 작업계획, 인터페이스 사양, **결과보고서**, 비전공자용 설명 페이지 (`hils_process_explained.html`) |

## 빠른 시작

```bash
pip install -r python/requirements.txt
python examples/ex01_offline_24h.py          # 겨울·여름 24 h (+ 이상적 결합 비교)
python examples/ex02_delay_study.py          # 통신 지연 0~120 s
python examples/ex03_modbus_loopback.py      # 실제 Modbus TCP 루프백
python examples/ex04_fault_injection.py      # 인터록 시험
cd python && python -m pytest -q tests       # 52 tests (Octave 있으면 MATLAB 교차검증 포함)

cd python && python -m hils.plc_server --port 5020 --season summer   # 가짜 PLC 단독 실행
```

```matlab
cd matlab
build_HILS_Controller('Season','summer')                                   % 에뮬레이터 모드
build_HILS_Controller('Season','summer','PlcIo','modbus','Host','192.168.0.10','Port',502)
out = hils_master('Mode','cosim','Season','summer','Duration',3600);       % ZoneFcn = EnergyPlus 자리
addpath tests; run_all_tests                                                % Octave 가능
```

## 문서
- [docs/01_work_plan.md](docs/01_work_plan.md): 작업계획, WBS, 완료 기준
- [docs/02_interface_spec.md](docs/02_interface_spec.md): 루프 순서, 레지스터 맵, air-enthalpy 식, 상태머신, 인터록
- [docs/03_result_report.md](docs/03_result_report.md): 시험 결과, 발견 오류와 수정, 한계
- [docs/04_villasnode_design.md](docs/04_villasnode_design.md): VILLASnode 게이트웨이 설계 (PLC ↔ VILLASnode ↔ 시뮬레이터, float32 맵), 설정 초안 `villas/hils.conf`
- [docs/hils_process_explained.html](docs/hils_process_explained.html): 비전공자용 도식 설명

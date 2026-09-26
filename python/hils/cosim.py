"""Co-simulation master (MATLAB hils_master.m 의 Python 대응).

시간층:
  Ts_building = Ts_supervisor = 60 s : 가상건물 step -> supervisor.on_building_step
  Ts_realization = 1 s               : 측정 read -> supervisor -> 명령 write
PLC 는 InMemory 모드에서는 master 가 같은 루프에서 scan() 을 호출(결정적),
Modbus 모드에서는 별도 스레드/프로세스에서 독립적으로 스캔(비동기, 실제와 동일).
"""
import csv
import math
import time

from .building import VirtualBuilding2R2C
from .state_machine import STATE_NAMES
from .supervisor import HilsSupervisor

LOG_FIELDS = ["t", "state", "fault", "seq", "ack", "pending", "latency", "valid",
              "Q_target", "Q_ref", "Q_load_meas", "Q_load_cmd", "Q_HP", "P_HP", "T_indoor", "T_outdoor",
              "HP_status", "T_out_weather", "Tm", "integ", "Kp"]


def run_hils(cfg, transport, duration_s, plc=None, realtime_scale=None, events=None, building=None):
    """events: {t: callable(plc, transport, sup)} 장애 주입 등."""
    tm = cfg["timing"]
    Ts, Tb = tm["Ts_realization"], tm["Ts_building"]
    sup = HilsSupervisor(cfg)
    bld = building or VirtualBuilding2R2C(cfg["virtual_building"], Tb, T_set=cfg["emulator"]["hp_setpoint"])
    log = {k: [] for k in LOG_FIELDS}
    events = dict(events or {})
    b = {"T_out": math.nan, "Tm": math.nan}
    t0 = time.perf_counter()
    n = int(duration_s / Ts)
    for i in range(n + 1):
        t = i * Ts
        if t in events:
            events.pop(t)(plc, transport, sup)
        meas = transport.read_measurements()
        if i % int(round(Tb / Ts)) == 0:
            b = bld.step(t, meas["T_indoor"], meas["Q_HP"])
            sup.on_building_step(t, b["Q_target"], b["T_target"], b["T_out"])
        cmd, st = sup.realization_step(t, meas)
        transport.write_commands(cmd)
        row = dict(t=t, state=st["state"], fault=st["fault"], seq=sup.seq, ack=meas["AckSequence"],
                   pending=int(st["pending"]), latency=st["latency"], valid=int(st["valid"]),
                   Q_target=sup.Q_target, Q_ref=st["q_ref"], Q_load_meas=meas["Q_load_meas"], Q_load_cmd=cmd["Q_load_cmd"],
                   Q_HP=meas["Q_HP"], P_HP=meas["P_HP"], T_indoor=meas["T_indoor"],
                   T_outdoor=meas["T_outdoor"], HP_status=meas["HP_status"], T_out_weather=b["T_out"],
                   Tm=b["Tm"], integ=st["integ"], Kp=st["Kp"])
        for k in LOG_FIELDS:
            log[k].append(row[k])
        if plc is not None:
            plc.scan()
        if realtime_scale:
            time.sleep(max(0.0, t0 + (t + Ts) / realtime_scale - time.perf_counter()))
    return log, sup, bld


def save_csv(log, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(LOG_FIELDS)
        for row in zip(*(log[k] for k in LOG_FIELDS)):
            w.writerow(row)


def kpis(log, cfg):
    """재현 성능 지표. RUN(valid) 구간 기준."""
    import numpy as np

    L = {k: np.asarray(v, dtype=float) for k, v in log.items()}
    v = L["valid"] > 0
    e = L["Q_ref"] - L["Q_load_meas"]   # 같은 시퀀스(프레임) 끼리 비교한 재현오차
    Ts = cfg["timing"]["Ts_realization"]
    E_t = np.sum(L["Q_target"]) * Ts
    E_r = np.sum(L["Q_load_meas"]) * Ts
    lat = L["latency"][~np.isnan(L["latency"])]
    return {
        "duration_h": (L["t"][-1] - L["t"][0]) / 3600,
        "valid_ratio": float(np.mean(v)),
        "rmse_valid_W": float(np.sqrt(np.mean(e[v] ** 2))) if v.any() else math.nan,
        "mae_valid_W": float(np.mean(np.abs(e[v]))) if v.any() else math.nan,
        "energy_target_kWh": E_t / 3.6e6,
        "energy_realized_kWh": E_r / 3.6e6,
        "energy_error_pct": 100 * (E_r - E_t) / E_t if E_t else math.nan,
        "energy_hp_kWh": float(np.sum(L["Q_HP"]) * Ts / 3.6e6),
        "elec_hp_kWh": float(np.sum(L["P_HP"]) * Ts / 3.6e6),
        "T_indoor_mean": float(np.mean(L["T_indoor"])),
        "T_indoor_min": float(np.min(L["T_indoor"])),
        "T_indoor_max": float(np.max(L["T_indoor"])),
        "ack_latency_mean_s": float(np.mean(lat)) if lat.size else math.nan,
        "ack_latency_max_s": float(np.max(lat)) if lat.size else math.nan,
        "fault_samples": int(np.sum(L["fault"] > 0)),
        "state_time_s": {STATE_NAMES[s]: float(np.sum(L["state"] == s) * Ts) for s in range(6)},
    }

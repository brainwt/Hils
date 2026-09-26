"""Co-simulation master (air-enthalpy 결합). MATLAB hils_master.m / hils_run_offline.m 대응.

한 건물 구간 k (Ts_building = 60 s) 의 순서:
  1 s 마다: PLC 측정 read -> air-enthalpy 열량 계산·누적 -> 안전/상태 -> 설정값 명령 write -> PLC scan
  60 s 마다: 구간 평균 열량(Q_sens, m_w) -> 가상 존 step -> 존 상태(k+1) -> 챔버 설정값, Sequence++
t = 0 에서는 존 초기상태를 설정값으로 보낸다(적분 없음).
"""
import csv
import math
import time

from .state_machine import STATE_NAMES
from .supervisor import HilsSupervisor
from .weather import make_weather
from .zone import VirtualZone

LOG_FIELDS = ["t", "state", "fault", "seq", "ack", "latency", "valid",
              "T_z", "RH_z", "T_sp", "RH_sp", "T_ref", "RH_ref",
              "T_supply", "RH_supply", "T_return", "RH_return", "V_air", "T_outdoor",
              "Q_sens", "Q_lat", "Q_tot", "m_w", "Q_sens_int", "m_w_int",
              "P_HP", "HP_status", "T_out_weather", "Tm"]


def make_zone(cfg, season="winter"):
    tm = cfg["timing"]
    return VirtualZone(cfg["zone"], make_weather(cfg["weather"][season]), tm["Ts_building"],
                       tm["Ts_zone_sub"], cfg["emulator"]["P_atm"])


def season_overrides(season):
    """계절별 초기조건과 히트펌프 운전모드(리모컨 설정에 해당)."""
    if season == "winter":
        z = {"T0": 20.0, "RH0": 40.0, "Tm0": 18.0}
        mode = "heat"
    elif season == "summer":
        z = {"T0": 27.0, "RH0": 60.0, "Tm0": 28.0}
        mode = "cool"
    else:
        raise ValueError(season)
    return {"zone": z, "emulator": {"hp_mode": mode, "T0": z["T0"], "RH0": z["RH0"]}}


def run_hils(cfg, transport, duration_s, plc=None, zone=None, realtime_scale=None, events=None):
    tm = cfg["timing"]
    Ts, Tb = tm["Ts_realization"], tm["Ts_building"]
    nb = int(round(Tb / Ts))
    sup = HilsSupervisor(cfg)
    zone = zone or make_zone(cfg)
    log = {k: [] for k in LOG_FIELDS}
    events = dict(events or {})
    zs = zone.state()
    q = {"Q_sens": 0.0, "m_w": 0.0}
    t0 = time.perf_counter()
    for i in range(int(duration_s / Ts) + 1):
        t = i * Ts
        if t in events:
            events.pop(t)(plc, transport, sup)
        meas = transport.read_measurements()
        if i % nb == 0:
            if i > 0:
                q = sup.interval_heat()                 # 구간 k 의 air-enthalpy 평균
                zs = zone.step(q["Q_sens"], q["m_w"])   # 존 상태 k+1
            sup.on_building_step(t, zs)
        cmd, st = sup.realization_step(t, meas)
        transport.write_commands(cmd)
        row = dict(t=t, state=st["state"], fault=st["fault"], seq=sup.seq, ack=meas["AckSequence"],
                   latency=st["latency"], valid=int(st["valid"]), T_z=zs["T_z"], RH_z=zs["RH_z"],
                   T_sp=cmd["T_room_SP"], RH_sp=cmd["RH_room_SP"], T_ref=st["T_ref"], RH_ref=st["RH_ref"],
                   T_supply=meas["T_supply"], RH_supply=meas["RH_supply"], T_return=meas["T_return"],
                   RH_return=meas["RH_return"], V_air=meas["V_air"], T_outdoor=meas["T_outdoor"],
                   Q_sens=st["Q_sens"], Q_lat=st["Q_lat"], Q_tot=st["Q_tot"], m_w=st["m_w"],
                   Q_sens_int=q["Q_sens"], m_w_int=q["m_w"], P_HP=meas["P_HP"],
                   HP_status=meas["HP_status"], T_out_weather=zs["T_out"], Tm=zs["Tm"])
        for k in LOG_FIELDS:
            log[k].append(row[k])
        if plc is not None:
            plc.scan()
        if realtime_scale:
            time.sleep(max(0.0, t0 + (t + Ts) / realtime_scale - time.perf_counter()))
    return log, sup, zone


def save_csv(log, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(LOG_FIELDS)
        for row in zip(*(log[k] for k in LOG_FIELDS)):
            w.writerow(row)


def kpis(log, cfg):
    import numpy as np

    L = {k: np.asarray(v, dtype=float) for k, v in log.items()}
    Ts = cfg["timing"]["Ts_realization"]
    v = L["valid"] > 0
    eT, eRH = L["T_return"] - L["T_ref"], L["RH_return"] - L["RH_ref"]
    rms = lambda x: float(np.sqrt(np.mean(x ** 2))) if x.size else math.nan  # noqa: E731
    Qt = L["Q_tot"]
    heat = float(np.sum(np.clip(Qt, 0, None)) * Ts / 3.6e6)
    cool = float(np.sum(np.clip(-Qt, 0, None)) * Ts / 3.6e6)
    cool_sens = float(np.sum(np.clip(-L["Q_sens"], 0, None)) * Ts / 3.6e6)
    elec = float(np.sum(L["P_HP"]) * Ts / 3.6e6)
    lat = L["latency"][~np.isnan(L["latency"])]
    return {
        "duration_h": (L["t"][-1] - L["t"][0]) / 3600,
        "valid_ratio": float(np.mean(v)),
        "track_rmse_T_K": rms(eT[v]), "track_rmse_RH_pct": rms(eRH[v]),
        "track_max_T_K": float(np.max(np.abs(eT[v]))) if v.any() else math.nan,
        "T_z_mean": float(np.mean(L["T_z"])), "T_z_min": float(np.min(L["T_z"])),
        "T_z_max": float(np.max(L["T_z"])), "RH_z_mean": float(np.mean(L["RH_z"])),
        "heat_kWh": heat, "cool_kWh": cool,
        "SHR": cool_sens / cool if cool > 0 else math.nan,
        "elec_kWh": elec,
        "COP_or_EER": (heat + cool) / elec if elec > 0 else math.nan,
        "ack_latency_mean_s": float(np.mean(lat)) if lat.size else math.nan,
        "ack_latency_max_s": float(np.max(lat)) if lat.size else math.nan,
        "fault_samples": int(np.sum(L["fault"] > 0)),
        "state_time_s": {STATE_NAMES[s]: float(np.sum(L["state"] == s) * Ts) for s in range(6)},
    }

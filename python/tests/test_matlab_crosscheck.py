"""Python 레퍼런스 <-> MATLAB lib(Octave 실행) 교차검증.

같은 시나리오(노이즈 0)를 두 구현으로 돌려 스텝별 레지스터/존 상태/열량이 일치하는지 확인.
Simulink MATLAB Function 블록이 같은 lib 함수를 호출하므로 제어 로직 동등성의 근거가 된다.
"""
import os
import shutil
import subprocess

import numpy as np
import pytest

from hils.config import _merge, load_config
from hils.cosim import make_zone, run_hils, season_overrides
from hils.emulator import PLCEmulator
from hils.transport import InMemoryTransport

OCTAVE = shutil.which("octave-cli") or shutil.which("octave")
MATLAB_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "matlab")
COLS = ["t", "state", "T_z", "RH_z", "T_sp", "RH_sp", "T_supply", "RH_supply", "T_return",
        "RH_return", "V_air", "Q_sens", "Q_lat", "ack", "P_HP", "HP_status"]
REGISTER_COLS = {"T_supply", "RH_supply", "T_return", "RH_return", "V_air", "ack", "P_HP",
                 "HP_status", "state", "t"}


def _octave(season, delay, duration, csv):
    code = (f"addpath('lib'); P=hils_params([],'{season}'); P.emulator.ack_delay_s={delay}; "
            f"L=hils_run_offline(P,{duration}); M=[{' '.join('L.' + c for c in COLS)}]; "
            f"dlmwrite('{csv}',M,'precision',15);")
    subprocess.run([OCTAVE, "--no-gui", "--quiet", "--eval", code], cwd=MATLAB_DIR, check=True,
                   capture_output=True, timeout=1800)
    return np.loadtxt(csv, delimiter=",")


def _python(season, delay, duration):
    o = season_overrides(season)
    _merge(o, {"emulator": {"meas_noise_T": 0, "meas_noise_RH": 0, "meas_noise_V_pct": 0,
                            "ack_delay_s": delay}})
    cfg = load_config(overrides=o)
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    log, _, _ = run_hils(cfg, tr, duration, plc=plc, zone=make_zone(cfg, season))
    return np.column_stack([np.asarray(log[c], float) for c in COLS])


@pytest.mark.skipif(OCTAVE is None, reason="octave not installed")
@pytest.mark.parametrize("season,delay,duration", [("winter", 3, 3600), ("summer", 3, 2400),
                                                   ("summer", 90, 2400)])
def test_python_matlab_identical(tmp_path, season, delay, duration):
    O = _octave(season, delay, duration, str(tmp_path / "oct.csv"))
    Py = _python(season, delay, duration)
    assert O.shape == Py.shape
    for j, c in enumerate(COLS):
        tol = 0.0 if c in REGISTER_COLS else 1e-6     # 레지스터 값은 정수로 완전 일치
        np.testing.assert_allclose(O[:, j], Py[:, j], rtol=0, atol=tol, err_msg=c)
    assert (Py[:, COLS.index("HP_status")] == (2 if season == "summer" else 1)).any()

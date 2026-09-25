"""Python 레퍼런스 <-> MATLAB lib(Octave 실행) 교차검증.

같은 시나리오(노이즈 0)를 두 구현으로 돌려 스텝별 레지스터/명령이 일치하는지 확인한다.
Simulink MATLAB Function 블록이 같은 lib 함수를 호출하므로, 제어 로직 동등성의 근거가 된다.
"""
import os
import shutil
import subprocess

import numpy as np
import pytest

from hils.building import StepProfileBuilding
from hils.config import load_config
from hils.cosim import run_hils
from hils.emulator import PLCEmulator
from hils.transport import InMemoryTransport

OCTAVE = shutil.which("octave-cli") or shutil.which("octave")
MATLAB_DIR = os.path.join(os.path.dirname(__file__), "..", "..", "matlab")
COLS = ["t", "state", "Q_load_cmd", "Q_load_meas", "T_indoor", "ack", "Kp", "Q_HP", "P_HP"]
PROFILE = [(0, 2000), (1800, 3500), (3600, 1500), (5400, 3000)]


def _octave(delay, duration, csv, profile):
    prof = ";".join(f"{a} {b}" for a, b in profile) if profile else ""
    opts = f"struct('profile',[{prof}])" if profile else "struct()"
    code = (f"addpath('lib'); P=hils_params(); P.emulator.meas_noise_W=0; "
            f"P.emulator.ack_delay_s={delay}; L=hils_run_offline(P,{duration},{opts}); "
            f"M=[{' '.join('L.' + c for c in COLS)}]; dlmwrite('{csv}',M,'precision',15);")
    subprocess.run([OCTAVE, "--no-gui", "--quiet", "--eval", code], cwd=MATLAB_DIR, check=True,
                   capture_output=True, timeout=900)
    return np.loadtxt(csv, delimiter=",")


def _python(delay, duration, profile):
    cfg = load_config(overrides={"emulator": {"meas_noise_W": 0, "ack_delay_s": delay}})
    tr = InMemoryTransport(cfg)
    plc = PLCEmulator(cfg, tr.bank)
    bld = StepProfileBuilding(profile) if profile else None
    log, _, _ = run_hils(cfg, tr, duration, plc=plc, building=bld)
    return np.column_stack([np.asarray(log[c], float) for c in COLS])


@pytest.mark.skipif(OCTAVE is None, reason="octave not installed")
@pytest.mark.parametrize("delay,duration,profile", [(3, 7200, PROFILE), (90, 7200, PROFILE),
                                                    (3, 4 * 3600, None)])
def test_python_matlab_identical(tmp_path, delay, duration, profile):
    O = _octave(delay, duration, str(tmp_path / "oct.csv"), profile)
    Py = _python(delay, duration, profile)
    assert O.shape == Py.shape
    for j, c in enumerate(COLS):
        tol = 1e-6 if c in ("Q_load_cmd", "Kp") else 0.0     # 레지스터 값은 정수로 완전 일치
        np.testing.assert_allclose(O[:, j], Py[:, j], rtol=0, atol=tol, err_msg=c)

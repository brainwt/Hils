import json
import os
from copy import deepcopy

_DEFAULT = os.path.join(os.path.dirname(__file__), "..", "..", "config", "hils_config.json")


def load_config(path=None, overrides=None):
    """Load hils_config.json and apply nested overrides ({"emulator": {"ack_delay_s": 10}})."""
    with open(path or _DEFAULT, encoding="utf-8") as f:
        cfg = json.load(f)
    if overrides:
        _merge(cfg, overrides)
    return cfg


def _merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _merge(dst[k], v)
        else:
            dst[k] = deepcopy(v)

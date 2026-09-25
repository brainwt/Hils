"""HILS reference implementation (virtual building - supervisor - PLC - heat pump).

Simulink 모델(matlab/)과 동일한 로직을 Python으로 구현한 레퍼런스.
MATLAB 없이 폐루프 동작, Modbus TCP 통신, 시퀀스/지연 처리를 검증하는 용도.
"""
from .config import load_config

__all__ = ["load_config"]

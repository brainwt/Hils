import pytest

from hils.config import load_config
from hils.registers import RegisterMap, decode_value, encode_value, round_half_away, seq_next


def test_example_from_plan():
    # 계획서 5절: 40001 = 2357 -> 23.57 degC
    assert decode_value(2357, 100, True) == pytest.approx(23.57)
    assert encode_value(23.57, 100, True) == (2357, False)


@pytest.mark.parametrize("v,scale,signed", [(-12.34, 100, True), (0, 1, True), (-8000, 1, True),
                                            (65535, 1, False), (99.99, 100, False), (-327.68, 100, True)])
def test_roundtrip(v, scale, signed):
    w, ovf = encode_value(v, scale, signed)
    assert 0 <= w <= 0xFFFF and not ovf
    assert decode_value(w, scale, signed) == pytest.approx(v, abs=0.5 / scale)


def test_negative_twos_complement():
    assert encode_value(-1, 1, True) == (0xFFFF, False)
    assert encode_value(-5.25, 100, True)[0] == 0x10000 - 525


def test_overflow_clamps():
    assert encode_value(40000, 1, True) == (32767, True)
    assert encode_value(-1, 1, False) == (0, True)
    assert encode_value(400.0, 100, True) == (32767, True)   # 400 degC x100 은 int16 초과


def test_round_half_away_matches_matlab():
    # Python round() 는 banker's rounding(2.5->2). PLC/MATLAB 과 같게 half-away-from-zero
    assert [round_half_away(x) for x in (0.5, 1.5, 2.5, -0.5, -2.5)] == [1, 2, 3, -1, -3]


def test_sequence_wrap_skips_zero():
    assert seq_next(0) == 1 and seq_next(65535) == 1 and seq_next(41) == 42


def test_register_map_blocks():
    rm = RegisterMap(load_config())
    assert rm.offset("T_supply") == 0 and rm.offset("T_room_SP") == 99
    assert rm.block("PLC2SIM") == (0, 15)
    assert rm.block("SIM2PLC") == (99, 7)
    start, words = rm.pack("SIM2PLC", {"T_room_SP": 22.47, "RH_room_SP": 41.5, "T_outdoor_SP": -3.25,
                                       "Enable": 1, "Sequence": 125})
    back = rm.unpack("SIM2PLC", start, words)
    assert back["T_room_SP"] == 22.47 and back["RH_room_SP"] == 41.5
    assert back["T_outdoor_SP"] == -3.25 and back["Sequence"] == 125

"""Modbus holding-register map and engineering-unit codec.

값은 16-bit 레지스터 1개에 저장한다.
  raw = round(value * scale)  (signed 이면 int16 2의 보수, 아니면 uint16)
범위를 넘는 값은 포화(clamp)시키고 overflow 플래그를 돌려준다.
"""

import math

INT16_MIN, INT16_MAX = -32768, 32767
UINT16_MAX = 65535


def encode_value(value, scale=1, signed=False):
    """Engineering value -> (uint16 word, overflow flag)."""
    raw = round_half_away(float(value) * scale)
    lo, hi = (INT16_MIN, INT16_MAX) if signed else (0, UINT16_MAX)
    overflow = raw < lo or raw > hi
    raw = min(max(raw, lo), hi)
    return raw & 0xFFFF, overflow


def round_half_away(x):
    """MATLAB round / PLC 와 같은 반올림. (Python round() 는 banker's rounding: 2.5 -> 2)"""
    return int(math.copysign(math.floor(abs(x) + 0.5), x))


def decode_value(word, scale=1, signed=False):
    """uint16 word -> engineering value."""
    word = int(word) & 0xFFFF
    if signed and word >= 0x8000:
        word -= 0x10000
    return word / scale if scale != 1 else float(word)


class RegisterMap:
    def __init__(self, cfg):
        self.base = cfg["modbus"]["base_address"]
        self.regs = cfg["registers"]
        self.status_bits = cfg["plc_status_bits"]

    def offset(self, name):
        """0-based protocol offset (40001 -> 0)."""
        return self.regs[name]["addr"] - self.base

    def names(self, direction):
        return [n for n, r in self.regs.items() if r["dir"] == direction]

    def block(self, direction):
        """(start offset, count) covering all registers of one direction."""
        offs = [self.offset(n) for n in self.names(direction)]
        return min(offs), max(offs) - min(offs) + 1

    def encode(self, name, value):
        r = self.regs[name]
        return encode_value(value, r["scale"], r["signed"])

    def decode(self, name, word):
        r = self.regs[name]
        return decode_value(word, r["scale"], r["signed"])

    def pack(self, direction, values):
        """dict of engineering values -> (start, [words]) for one block write."""
        start, count = self.block(direction)
        words = [0] * count
        for n in self.names(direction):
            words[self.offset(n) - start], _ = self.encode(n, values.get(n, 0))
        return start, words

    def unpack(self, direction, start, words):
        return {n: self.decode(n, words[self.offset(n) - start]) for n in self.names(direction)}


def seq_next(seq):
    """uint16 sequence with wrap; 0 is reserved for 'no command'."""
    seq = (int(seq) + 1) & 0xFFFF
    return seq if seq != 0 else 1


def seq_equal(a, b):
    return (int(a) & 0xFFFF) == (int(b) & 0xFFFF)

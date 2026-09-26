"""Sequence / AckSequence 기반 지연 감시.

- 미확인(outstanding) 시퀀스를 모두 추적한다. 지연이 Ts_building 보다 길면
  여러 시퀀스가 동시에 대기할 수 있기 때문이다(단일 추적은 ack 를 영원히 못 맞춘다).
- AckSequence = s 를 받으면 s 이전(모듈러 순서)의 모든 시퀀스를 확인 처리한다.
- 각 시퀀스의 설정값(T_sp, RH_sp)을 기억해 두었다가 acked_target() 으로 "PLC 가 실제
  적용 중인 프레임의 설정값" 을 돌려준다 -> 챔버 추종오차를 같은 timestep 끼리 비교.
"""
from collections import OrderedDict

HISTORY = 64


def seq_diff(a, b):
    """(a - b) in uint16 modular arithmetic, mapped to [-32768, 32767]."""
    d = (int(a) - int(b)) & 0xFFFF
    return d - 0x10000 if d >= 0x8000 else d


class DelayMonitor:
    def __init__(self, ack_timeout_s):
        self.timeout = ack_timeout_s
        self.outstanding = OrderedDict()   # seq -> t_sent
        self.targets = OrderedDict()       # seq -> payload (최근 HISTORY 개)
        self.last_ack = 0
        self.last_latency = float("nan")
        self.latencies = []                # (seq, t_sent, latency)

    def on_send(self, seq, t, payload=None):
        self.outstanding[seq] = t
        self.targets[seq] = payload
        while len(self.targets) > HISTORY:
            self.targets.popitem(last=False)

    def step(self, ack, t):
        """Returns (pending, timeout)."""
        ack = int(ack)
        if ack != self.last_ack and ack in self.outstanding:
            for s in [s for s in self.outstanding if seq_diff(s, ack) <= 0]:
                ts = self.outstanding.pop(s)
                if s == ack:
                    self.last_latency = t - ts
                    self.latencies.append((s, ts, self.last_latency))
        self.last_ack = ack
        return bool(self.outstanding), self.age(t) > self.timeout

    def age(self, t):
        """가장 오래된 미확인 시퀀스의 경과시간."""
        return t - next(iter(self.outstanding.values())) if self.outstanding else 0.0

    def acked_target(self):
        return self.targets.get(self.last_ack)

    @property
    def pending(self):
        return bool(self.outstanding)

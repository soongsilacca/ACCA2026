"""E-STOP 발행/해제 규칙.

CountDebounce: 프레임 단위. 충돌 판정(LiDAR 프레임마다 1회 평가)에 사용.
    n_on 프레임 연속 성립 -> 발행, n_off 프레임 연속 불성립 -> 해제.
"""


class CountDebounce:
    def __init__(self, n_on=2, n_off=5, initial=False):
        if n_on < 1 or n_off < 1:
            raise ValueError("n_on, n_off must be >= 1")
        self.n_on = int(n_on)
        self.n_off = int(n_off)
        self.state = bool(initial)
        self._on_count = 0
        self._off_count = 0

    def update(self, condition):
        if condition:
            self._on_count += 1
            self._off_count = 0
            if not self.state and self._on_count >= self.n_on:
                self.state = True
        else:
            self._off_count += 1
            self._on_count = 0
            if self.state and self._off_count >= self.n_off:
                self.state = False
        return self.state

    def reset(self, state=False):
        self.state = bool(state)
        self._on_count = 0
        self._off_count = 0

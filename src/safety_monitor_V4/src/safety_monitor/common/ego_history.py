"""자차 pose 이력. 다른 입력의 시각에 맞춘 자차 pose를 보간해서 준다.

트랙(LiDAR 스캔 시각)과 local route를 map으로 옮길 때 최신 자차 pose 대신
그 메시지 시각의 pose를 써야 절대 위치(어느 차선인지)가 어긋나지 않는다.
"""
import math
from collections import deque

from .geometry import wrap_angle
from .types import EgoState


class EgoHistory:
    def __init__(self, keep=2.0, max_extrapolation=0.2):
        """keep: 보관 시간 [s]. max_extrapolation: 이력 밖 시각을 등속 외삽할 최대 시간 [s]."""
        self.keep = float(keep)
        self.max_extrapolation = float(max_extrapolation)
        self._buf = deque()

    def add(self, ego):
        if self._buf and ego.stamp < self._buf[-1].stamp:
            # 시각이 뒤로 가면 (bag 반복 재생, 시계 재설정) 이력을 버린다
            self._buf.clear()
        self._buf.append(ego)
        while self._buf and self._buf[-1].stamp - self._buf[0].stamp > self.keep:
            self._buf.popleft()

    def latest(self):
        return self._buf[-1] if self._buf else None

    def at(self, t):
        """시각 t의 자차 pose. 이력이 없거나 외삽 한도를 넘으면 None."""
        if not self._buf:
            return None
        first, last = self._buf[0], self._buf[-1]
        if t >= last.stamp:
            return _extrapolate(last, t) if t - last.stamp <= self.max_extrapolation else None
        if t <= first.stamp:
            return _extrapolate(first, t) if first.stamp - t <= self.max_extrapolation else None
        prev = first
        for cur in self._buf:
            if cur.stamp >= t:
                span = cur.stamp - prev.stamp
                a = 0.0 if span <= 0.0 else (t - prev.stamp) / span
                yaw = prev.yaw + a * float(wrap_angle(cur.yaw - prev.yaw))
                return EgoState(t, prev.x + a * (cur.x - prev.x), prev.y + a * (cur.y - prev.y),
                                float(wrap_angle(yaw)), prev.speed + a * (cur.speed - prev.speed))
            prev = cur
        return last


def _extrapolate(ego, t):
    dt = t - ego.stamp
    return EgoState(t, ego.x + ego.speed * math.cos(ego.yaw) * dt,
                    ego.y + ego.speed * math.sin(ego.yaw) * dt, ego.yaw, ego.speed)

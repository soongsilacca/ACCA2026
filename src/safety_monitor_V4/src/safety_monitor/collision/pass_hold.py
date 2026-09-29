"""6.1.2 예외 둘째: 대상이 자차 옆을 지나가며 관측이 끊기는 경우.

관측이 끊긴 시점 = 감시 중이던 트랙이 이번 스캔에서 측정되지 않은 첫 프레임
(is_coasting으로 바뀌었거나 목록에서 사라짐).
그 시점의 종방향 간격이 hold_trigger_gap 이하이면 통과 중으로 보고
마지막 측정 시각 + t_end + t_hold 까지 정지를 유지한다.
상대속도가 작아 t_end가 정해지지 않으면 재관측까지 유지하되 hold_max로 상한을 둔다.

속도는 박스가 ROI 경계에 잘리기 전 프레임들의 중앙값을 쓴다.
"""
from collections import deque

import numpy as np

from .overlap import INF, longitudinal_window


class _TrackMemory:
    __slots__ = ("velocities", "last_stamp", "last_gap", "last_v_ego", "last_v")

    def __init__(self, n):
        self.velocities = deque(maxlen=n)
        self.last_stamp = None
        self.last_gap = None
        self.last_v_ego = None
        self.last_v = None


class PassHold:
    def __init__(self, params):
        self.p = params
        self.reset()

    def reset(self):
        self._mem = {}      # track_id -> _TrackMemory
        self.holds = {}     # track_id -> 유지 종료 시각

    def _snapshot_velocity(self, mem):
        if mem.velocities:
            return float(np.median(np.asarray(mem.velocities)))
        return mem.last_v

    def update(self, stamp, watched, measured_ids, s_ego_rear, v_ego_s, length_total):
        """watched: 이번 프레임 감시 대상 Target 목록.
        measured_ids: 이번 스캔에서 실제 측정된 모든 트랙 ID.
        반환: 유지 중인 track_id 목록.
        """
        p = self.p
        watched_ids = set()
        for t in watched:
            watched_ids.add(t.track_id)
            if not t.measured:
                continue
            mem = self._mem.get(t.track_id)
            if mem is None:
                mem = _TrackMemory(p.hold_velocity_window)
                self._mem[t.track_id] = mem
            mem.last_stamp = stamp
            mem.last_gap = s_ego_rear - t.s_front
            mem.last_v_ego = v_ego_s
            mem.last_v = t.v_s
            if t.front_x_base <= p.hold_velocity_max_front_x:
                mem.velocities.append(t.v_s)
            self.holds.pop(t.track_id, None)   # 재관측 -> 정상 판정으로 복귀

        # 관측이 끊긴 트랙 처리
        for tid in list(self._mem.keys()):
            if tid in measured_ids:
                continue
            mem = self._mem.pop(tid)
            if tid in self.holds or mem.last_gap is None:
                continue
            if mem.last_gap > p.hold_trigger_gap:
                continue
            v_obs = self._snapshot_velocity(mem)
            v_rel = v_obs - mem.last_v_ego
            win = longitudinal_window(mem.last_gap, v_rel, length_total,
                                      p.long_margin, p.v_rel_eps)
            cap = stamp + p.hold_max
            if win is None:
                continue
            if win[1] == INF:
                self.holds[tid] = cap
            else:
                self.holds[tid] = min(mem.last_stamp + win[1] + p.hold_extra, cap)

        # 감시 대상에서 빠졌지만 아직 측정 중인 트랙은 기억만 지운다
        for tid in list(self._mem.keys()):
            if tid not in watched_ids and tid in measured_ids:
                del self._mem[tid]

        for tid in [k for k, until in self.holds.items() if until <= stamp]:
            del self.holds[tid]
        return sorted(self.holds.keys())

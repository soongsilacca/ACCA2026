"""bag에서 뽑은 .npz로 CollisionMonitor를 오프라인 재생 (ROS 불필요).

npz 형식 (scripts/bag_to_npz.py가 생성)
  trk_frame_t      (F,)    트랙 메시지 수신 시각 (bag 시각)
  trk_rows         (K,13)  frame_idx, id, x, y, dim_x, dim_y, dim_z, vx, vy,
                           is_dynamic, compensated, coasting, time_since_update
  ego              (E,5)   t, x, y, yaw(rad), speed(m/s)
  route_t          (R,)    local route 수신 시각
  route_offsets    (R+1,)  route_pts 안에서 각 route의 시작 위치
  route_pts        (P,2)   route 점 (route_frame 기준)
  route_frame      ()      "base_link" 또는 "map"
모든 시각은 bag 수신 시각으로 맞춘다 (센서별 header 시계가 다를 수 있음).
"""
import bisect

import numpy as np

from .collision.monitor import CollisionMonitor
from .common.geometry import base_to_map
from .common.types import EgoState, TrackObs


class ReplayData:
    def __init__(self, path):
        z = np.load(path, allow_pickle=False)
        self.frame_t = z["trk_frame_t"]
        rows = z["trk_rows"]
        self.ego = z["ego"]
        self.route_t = z["route_t"]
        self.route_offsets = z["route_offsets"]
        self.route_pts = z["route_pts"]
        self.route_frame = str(z["route_frame"])
        order = np.argsort(rows[:, 0], kind="stable") if len(rows) else np.array([], int)
        rows = rows[order]
        self._rows = rows
        idx = rows[:, 0].astype(int) if len(rows) else np.array([], int)
        self._starts = np.searchsorted(idx, np.arange(len(self.frame_t)), side="left")
        self._ends = np.searchsorted(idx, np.arange(len(self.frame_t)), side="right")
        self._ego_t = self.ego[:, 0].tolist()
        self._route_t = self.route_t.tolist()

    def objects(self, i):
        out = []
        for r in self._rows[self._starts[i]:self._ends[i]]:
            out.append(TrackObs(int(r[1]), r[2], r[3], r[4], r[5], r[6], r[7], r[8],
                                bool(r[9]), bool(r[10]), bool(r[11]), float(r[12])))
        return out

    def ego_at(self, t, max_age=0.3):
        if not self._ego_t:
            return None
        k = bisect.bisect_left(self._ego_t, t)
        cands = [j for j in (k - 1, k) if 0 <= j < len(self._ego_t)]
        j = min(cands, key=lambda c: abs(self._ego_t[c] - t))
        if abs(self._ego_t[j] - t) > max_age:
            return None
        e = self.ego[j]
        return EgoState(float(e[0]), float(e[1]), float(e[2]), float(e[3]), float(e[4]))

    def route_at(self, t, max_age=1.0):
        k = bisect.bisect_right(self._route_t, t) - 1
        if k < 0 or t - self.route_t[k] > max_age:
            return None
        pts = self.route_pts[self.route_offsets[k]:self.route_offsets[k + 1]]
        if self.route_frame in ("map", "world"):
            return pts
        ego = self.ego_at(float(self.route_t[k]))
        if ego is None:
            return None
        xs, ys = base_to_map(ego, pts[:, 0], pts[:, 1])
        return np.column_stack([xs, ys])


def run(data, params, lanemap):
    mon = CollisionMonitor(params, lanemap)
    results = []
    for i, t in enumerate(data.frame_t):
        t = float(t)
        ego = data.ego_at(t) or EgoState(t, 0.0, 0.0, 0.0, 0.0, valid=False)
        results.append(mon.step(t, ego, data.route_at(t), data.objects(i)))
    return results


def estop_intervals(results):
    """E-STOP 구간 목록 [(시작, 끝, 시작 사유)]."""
    out, start, why = [], None, None
    for r in results:
        if r.estop and start is None:
            start, why = r.stamp, r.reason
        elif not r.estop and start is not None:
            out.append((start, r.stamp, why))
            start = None
    if start is not None:
        out.append((start, results[-1].stamp, why))
    return out

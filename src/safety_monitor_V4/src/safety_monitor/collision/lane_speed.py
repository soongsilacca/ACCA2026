"""차선 추종 가정으로 대상 차량의 차선 방향 속도 추정.

대상 차량은 자기가 있는 MGeo 차선을 따라 움직인다고 본다.
매 관측마다 그 차선 중심선 위에서 상대 위치 z = s(대상 앞 끝) - s(자차 뒤 끝)를 재고,
[z, dz/dt] 1차원 칼만 필터로 거리 변화율을 추정한다.
  v_s(대상) = 자차의 차선 방향 속도 + dz/dt

두 점을 같은 자차 pose로 map에 옮기므로 localization 위치 흔들림은 z에서 상쇄된다.
인지 추적기의 프레임 간 위치 차분(Δt 기반) 속도는 필터 초기값으로만 쓴다
(새 트랙의 첫 프레임에서 속도를 모르는 구간을 없애기 위해).
"""
import math

import numpy as np

from ..common.geometry import base_to_map, rotate_to_map, wrap_angle


class _Track:
    __slots__ = ("x", "P", "stamp", "updates")

    def __init__(self, z, zdot, stamp, r, v0_std):
        self.x = np.array([z, zdot], dtype=float)
        self.P = np.diag([r * r, v0_std * v0_std])
        self.stamp = stamp
        self.updates = 1


class LaneSpeedEstimator:
    def __init__(self, params, lanemap):
        self.p = params
        self.lanemap = lanemap
        self.tracks = {}

    def reset(self):
        self.tracks.clear()

    def _lane_poly(self, x, y, yaw):
        p = self.p
        lid = self.lanemap.match(x, y, yaw, p.lane_match_max_dist, p.lane_match_max_heading_deg)
        if lid is None:
            return None
        return self.lanemap.chain(lid, p.chain_back, p.chain_fwd).poly

    def update(self, stamp, ego, objects):
        """objects: TrackObs 목록. 반환 {track_id: 차선 방향 속도 [m/s]} (추정 가능한 트랙만)."""
        p = self.p
        alive = {o.track_id for o in objects}
        for tid in [t for t in self.tracks if t not in alive]:
            del self.tracks[tid]

        rx, ry = base_to_map(ego, -p.ego_rear, 0.0)
        out = {}
        for o in objects:
            cx, cy = base_to_map(ego, o.x, o.y)
            poly = self._lane_poly(cx, cy, ego.yaw)
            if poly is None:
                # 대상이 차선에 매칭되지 않으면 자차 차선 기준으로 잰다 (평행 차선이면 s 차이가 같음)
                poly = self._lane_poly(rx, ry, ego.yaw)
            if poly is None:
                continue
            fx, fy = base_to_map(ego, o.x + 0.5 * o.dim_x, o.y)
            s_front, _, h = poly.project_one(fx, fy)
            s_rear, _, h_ego = poly.project_one(rx, ry)
            v_ego_s = ego.speed * math.cos(float(wrap_angle(ego.yaw - h_ego)))
            z = s_front - s_rear

            tr = self.tracks.get(o.track_id)
            if tr is None:
                if o.coasting:
                    continue
                vxm, vym = rotate_to_map(ego, o.vx, o.vy)
                v_init = vxm * math.cos(h) + vym * math.sin(h)
                tr = _Track(z, v_init - v_ego_s, stamp, p.lane_speed_meas_std, p.lane_speed_init_std)
                self.tracks[o.track_id] = tr
            else:
                self._predict(tr, stamp)
                if not o.coasting:
                    self._correct(tr, z)
            out[o.track_id] = v_ego_s + float(tr.x[1])
        return out

    def _predict(self, tr, stamp):
        dt = stamp - tr.stamp
        if dt <= 0.0:
            return
        tr.stamp = stamp
        dt = min(dt, 1.0)
        F = np.array([[1.0, dt], [0.0, 1.0]])
        q = self.p.lane_speed_accel_std ** 2
        Q = q * np.array([[dt ** 4 / 4.0, dt ** 3 / 2.0], [dt ** 3 / 2.0, dt ** 2]])
        tr.x = F @ tr.x
        tr.P = F @ tr.P @ F.T + Q

    def _correct(self, tr, z):
        r2 = self.p.lane_speed_meas_std ** 2
        y = z - tr.x[0]
        S = tr.P[0, 0] + r2
        K = tr.P[:, 0] / S
        tr.x = tr.x + K * y
        tr.P = tr.P - np.outer(K, tr.P[0, :])
        tr.updates += 1

"""후방 충돌 감시 한 프레임 처리. ROS 노드와 bag 재생 스크립트가 같이 쓴다.

순서
  local route로 차선 변경 여부와 목표 차선 결정 (①)
  -> 대상 차량을 목표 차선 Frenet으로 변환, 접근 차량 선정 (①)
  -> 대상별 ②③④ 판정 + 예외 첫째
  -> 예외 둘째(통과 차량 정지 유지)
  -> 2프레임 연속 성립 발행 / 5프레임 연속 불성립 해제
"""
import math

import numpy as np

from ..adapters.tracked_object import to_target
from ..common.debounce import CountDebounce
from ..common.geometry import base_to_map, polyline_heading, resample_polyline, wrap_angle
from ..common.types import MonitorResult
from .judge import build_route_profile, evaluate_target
from .lane_speed import LaneSpeedEstimator
from .pass_hold import PassHold
from .target_selector import LaneChangeTracker, WatchSelector


def prepare_route(route_map, ego, step):
    """자차에서 가장 가까운 점부터 앞쪽만 남기고 재표본화. 반환 (xy, heading, sigma) 또는 None."""
    if route_map is None or len(route_map) < 2:
        return None
    pts = np.asarray(route_map, dtype=float)[:, :2]
    i0 = int(np.argmin(np.hypot(pts[:, 0] - ego.x, pts[:, 1] - ego.y)))
    pts = pts[i0:]
    if len(pts) < 2:
        return None
    xy, sigma = resample_polyline(pts, step)
    if len(xy) < 2:
        return None
    return xy, polyline_heading(xy), sigma


class CollisionMonitor:
    def __init__(self, params, lanemap):
        self.p = params
        self.lanemap = lanemap
        self.lane = LaneChangeTracker(params)
        self.selector = WatchSelector(params)
        self.hold = PassHold(params)
        self.debounce = CountDebounce(params.n_on, params.n_off)
        self.speed = LaneSpeedEstimator(params, lanemap)

    def _reset_lane_state(self):
        self.selector.reset()
        self.hold.reset()

    def _finish(self, stamp, raw, reason, **kw):
        estop = self.debounce.update(raw)
        return MonitorResult(stamp=stamp, raw=raw, estop=estop, reason=reason,
                             lane_change_active=self.lane.active, **kw)

    def step(self, stamp, ego, route_map, objects):
        """stamp: 트랙 메시지 시각, ego: EgoState, route_map: (N,2) map 좌표 local route,
        objects: TrackObs 목록."""
        p = self.p
        if ego is None or not ego.valid:
            # 자차 상태를 모르면 판정 불가 -> 정지
            return self._finish(stamp, True, "ego_invalid")

        # 차선 변경 전부터 추정해 두어야 시작 시점에 속도가 수렴해 있다
        speeds = self.speed.update(stamp, ego, objects) if p.lane_speed else {}

        route = prepare_route(route_map, ego, p.route_step)
        if route is None:
            if self.lane.active:
                return self._finish(stamp, True, "no_route_during_lane_change")
            return self._finish(stamp, False, "no_route")
        route_xy, route_h, sigma = route

        status = self.lane.update(ego, route_xy, route_h, self.lanemap)
        if not self.lane.active:
            self._reset_lane_state()
            if status in ("no_ego_lane", "no_target_lane") and p.stop_on_map_failure:
                # 지도 매칭 실패로 판정 불가. 곡선 구간 오검출 우려가 있어 기본값은 정지하지 않음.
                return self._finish(stamp, True, status)
            return self._finish(stamp, False, status)

        # 자차 이동 보정이 안 된 트랙이 있으면 속도를 믿을 수 없다
        if any(not o.compensated for o in objects):
            return self._finish(stamp, True, "uncompensated_tracks")

        tgt_poly = self.lane.target.poly
        org_poly = self.lane.origin.poly
        rx, ry = base_to_map(ego, -p.ego_rear, 0.0)
        s_ego_rear, _, h_ego = tgt_poly.project_one(rx, ry)
        v_ego_s = ego.speed * math.cos(float(wrap_angle(ego.yaw - h_ego)))

        targets = [to_target(o, ego, tgt_poly, org_poly, p, speeds.get(o.track_id)) for o in objects]
        watched = self.selector.update(targets, s_ego_rear, v_ego_s)

        profile = build_route_profile(route_xy, route_h, sigma, tgt_poly)
        judgments = [evaluate_target(t, profile, s_ego_rear, v_ego_s, ego.speed, p)
                     for t in watched]

        measured_ids = {o.track_id for o in objects if not o.coasting}
        holds = self.hold.update(stamp, watched, measured_ids, s_ego_rear, v_ego_s,
                                 p.ego_length + p.obs_length_default)

        hit = [j.track_id for j in judgments if j.collision]
        raw = bool(hit) or bool(holds)
        if hit:
            reason = "collision:" + ",".join(str(i) for i in hit)
        elif holds:
            reason = "pass_hold:" + ",".join(str(i) for i in holds)
        else:
            reason = "clear"
        return self._finish(stamp, raw, reason,
                            watched=[t.track_id for t in watched],
                            judgments=judgments, holds=holds)

"""6.1.2 ① 감시 대상 선정.

LaneChangeTracker: local route 형상으로 목표 차선을 정하고, 차선 변경 종료까지 유지.
WatchSelector: 목표 차선의 대상 중 접근 차량을 이력(진입/제외 임계)으로 선정.
"""
import math

import numpy as np

from ..common.geometry import wrap_angle


class LaneChangeTracker:
    IDLE = "idle"
    ACTIVE = "active"

    def __init__(self, params):
        self.p = params
        self.reset()

    def reset(self):
        self.state = self.IDLE
        self.target = None      # LaneChain
        self.origin = None      # LaneChain
        self.status = "idle"

    @property
    def active(self):
        return self.state == self.ACTIVE

    def update(self, ego, route_xy, route_heading, lanemap):
        """route_xy: 자차 위치부터 시작하는 map 좌표 경로 (재표본화된 것)."""
        p = self.p
        if self.active:
            s_e, d_e, h_e = self.target.poly.project_one(ego.x, ego.y)
            dh = abs(float(wrap_angle(ego.yaw - h_e)))
            if abs(d_e) <= p.done_lateral and dh <= math.radians(p.done_heading_deg):
                self.reset()
                self.status = "completed"
                return self.status
            # 취소: 경로가 더 이상 목표 차선으로 가지 않고 자차도 원래 차선에 있음
            _, d_route, _ = self.target.poly.project(route_xy)
            _, d_org, _ = self.origin.poly.project_one(ego.x, ego.y)
            if np.min(np.abs(d_route)) > p.lane_half_width and abs(d_org) < p.lane_half_width:
                self.reset()
                self.status = "cancelled"
                return self.status
            self.status = "active"
            return self.status

        ego_link = lanemap.match(ego.x, ego.y, ego.yaw, p.lane_match_max_dist,
                                 p.lane_match_max_heading_deg)
        if ego_link is None:
            self.status = "no_ego_lane"
            return self.status
        origin = lanemap.chain(ego_link, p.chain_back, p.chain_fwd)
        _, d_route, _ = origin.poly.project(route_xy)
        crossing = np.where(np.abs(d_route) > p.lane_half_width)[0]
        if len(crossing) == 0:
            self.status = "idle"
            return self.status
        for i in crossing:
            lid = lanemap.match(route_xy[i, 0], route_xy[i, 1], route_heading[i],
                                p.lane_match_max_dist, p.lane_match_max_heading_deg)
            if lid is not None and lid not in origin.ids:
                self.target = lanemap.chain(lid, p.chain_back, p.chain_fwd)
                self.origin = origin
                self.state = self.ACTIVE
                self.status = "started"
                return self.status
        self.status = "no_target_lane"
        return self.status


def in_target_lane(d_target, d_origin, lane_half_width):
    """최근접 차선 매칭: 목표 차선 중심에 더 가깝고 차선 폭 안에 있으면 목표 차선."""
    return abs(d_target) <= lane_half_width and abs(d_target) < abs(d_origin)


class WatchSelector:
    def __init__(self, params):
        self.p = params
        self.watched = set()

    def reset(self):
        self.watched = set()

    def update(self, targets, s_ego_rear, v_ego_s):
        """targets: Target 목록. 반환: 감시 대상 Target 목록."""
        p = self.p
        new = set()
        out = []
        for t in targets:
            if not t.in_target_lane:
                continue
            gap = s_ego_rear - t.s_front
            v_rel = t.v_s - v_ego_s
            near = gap <= max(v_rel, 0.0) * p.horizon + p.long_margin
            if t.track_id in self.watched:
                keep = v_rel >= p.v_exit or near
            else:
                keep = v_rel >= p.v_enter or near
            if keep:
                new.add(t.track_id)
                out.append(t)
        self.watched = new
        return out

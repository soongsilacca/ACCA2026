"""한 프레임에서 대상 한 대에 대한 충돌 판정 (①~④ + 예외 첫째)."""
import numpy as np

from ..common.geometry import wrap_angle
from ..common.types import TargetJudgment
from .overlap import combine, entry_time, lateral_entry_progress, longitudinal_window


class RouteProfile:
    """자차 위치에서 시작하는 local route를 목표 차선 Frenet으로 옮긴 것."""

    def __init__(self, sigma, d, theta):
        self.sigma = sigma
        self.d = d
        self.theta = theta


def build_route_profile(route_xy, route_heading, sigma, target_poly):
    _, d, h_lane = target_poly.project(route_xy)
    theta = wrap_angle(route_heading - h_lane)
    return RouteProfile(np.asarray(sigma), d, np.asarray(theta))


def evaluate_target(target, route, s_ego_rear, v_ego_s, ego_speed, params):
    p = params
    sigma_enter = lateral_entry_progress(
        route.sigma, route.d, route.theta, p.ego_front, p.ego_rear, 0.5 * p.ego_width,
        target.d, target.width, p.lateral_margin)

    # 예외 첫째: 정지 중이면 재출발 가정 속도. 이미 횡방향으로 겹쳤으면 t_enter = 0.
    if sigma_enter is not None and sigma_enter <= 0.0:
        t_enter = 0.0
    else:
        v_ref = ego_speed if ego_speed >= p.v_stop else p.v_restart
        t_enter = entry_time(sigma_enter, v_ref)

    gap = s_ego_rear - target.s_front
    v_rel = target.v_s - v_ego_s
    window = longitudinal_window(gap, v_rel, p.ego_length + target.length,
                                 p.long_margin, p.v_rel_eps)
    hit, t_c = combine(t_enter, window, p.horizon)
    return TargetJudgment(target.track_id, hit, gap, v_rel, t_enter, window, t_c)

"""safety_monitor/TrackedObject (rear_lidar_perception_node 출력) -> TrackObs -> Target 변환.

from_msg는 속성 접근만 사용하므로 ROS 없이도(테스트, bag 재생) 쓸 수 있다.

주의 (perception 출력 특성)
- pose는 보이는 점들의 축정렬 박스 중심이다. 후방 차량은 앞면만 보이므로
  앞 끝 = pose.x + dim_x/2 를 쓰고, 뒤 끝은 가정 전장으로 정한다.
- 폭은 base_link y축 길이(dim_y)를 쓴다. min(dx, dy)를 쓰면 앞면 깊이(약 0.3 m)가 폭이 된다.
"""
import math

from ..common.geometry import base_to_map, rotate_to_map
from ..common.types import Target, TrackObs
from ..collision.target_selector import in_target_lane


def from_msg(obj):
    return TrackObs(
        track_id=int(obj.track_id),
        x=float(obj.pose.position.x),
        y=float(obj.pose.position.y),
        dim_x=float(obj.dimensions.x),
        dim_y=float(obj.dimensions.y),
        dim_z=float(obj.dimensions.z),
        vx=float(obj.velocity.x),
        vy=float(obj.velocity.y),
        is_dynamic=bool(obj.is_dynamic),
        compensated=bool(obj.odometry_compensated),
        coasting=bool(obj.is_coasting),
        time_since_update=float(obj.time_since_update),
    )


def to_target(obs, ego, target_poly, origin_poly, params):
    p = params
    front_x = obs.x + 0.5 * obs.dim_x
    fx, fy = base_to_map(ego, front_x, obs.y)
    cx, cy = base_to_map(ego, obs.x, obs.y)
    s_front, _, h = target_poly.project_one(fx, fy)
    _, d_center, _ = target_poly.project_one(cx, cy)
    _, d_origin, _ = origin_poly.project_one(cx, cy)
    vxm, vym = rotate_to_map(ego, obs.vx, obs.vy)
    v_s = vxm * math.cos(h) + vym * math.sin(h)
    width = min(max(obs.dim_y, p.obs_width_min), p.obs_width_max)
    length = max(obs.dim_x, p.obs_length_default)
    return Target(
        track_id=obs.track_id,
        s_front=s_front,
        d=d_center,
        width=width,
        length=length,
        v_s=v_s,
        front_x_base=front_x,
        measured=not obs.coasting,
        in_target_lane=in_target_lane(d_center, d_origin, p.lane_half_width),
    )

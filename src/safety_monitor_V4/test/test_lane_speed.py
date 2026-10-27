"""차선 추종 가정 속도 추정 (collision/lane_speed.py)."""
import math

import numpy as np
import pytest

from safety_monitor.collision.lane_speed import LaneSpeedEstimator
from safety_monitor.common.params import CollisionParams
from safety_monitor.common.types import EgoState, TrackObs
from safety_monitor.lanes.mgeo_lanes import LaneMap, Link

DT = 0.137
P = CollisionParams()


def straight_map():
    xs = np.linspace(-300, 300, 13)
    return LaneMap([Link("L0", np.column_stack([xs, np.zeros_like(xs)])),
                    Link("L1", np.column_stack([xs, np.full_like(xs, 3.5)]))])


def arc_map(r=60.0):
    a = np.linspace(-math.pi / 2, math.pi / 2, 400)
    return LaneMap([Link("C", np.column_stack([r * np.sin(a), r - r * np.cos(a)]))])


def obs_in_base(ego, tid, mx, my, vx=0.0, vy=0.0, coasting=False):
    """map 좌표 앞 끝 (mx, my)인 후방 차량을 base_link TrackObs로. 박스 앞 끝 = x + 0.2."""
    c, s = math.cos(ego.yaw), math.sin(ego.yaw)
    dx, dy = mx - ego.x, my - ego.y
    xb, yb = c * dx + s * dy, -s * dx + c * dy
    return TrackObs(tid, xb - 0.2, yb, 0.4, 1.8, 1.5, vx, vy, True, True, coasting)


def test_tracker_velocity_only_used_as_initial_value():
    """추적기는 새 트랙 속도 0을 내지만 차선 방향 위치 변화로 20 m/s를 찾아낸다."""
    est = LaneSpeedEstimator(P, straight_map())
    out = []
    for i in range(10):
        t = i * DT
        ego = EgoState(t, 10.0 * t, 0.0, 0.0, 10.0)
        out.append(est.update(t, ego, [obs_in_base(ego, 1, -20.0 + 20.0 * t, 3.5)])[1])
    assert out[0] == pytest.approx(0.0, abs=1e-6)      # 첫 프레임은 추적기 값
    assert out[-1] == pytest.approx(20.0, abs=0.5)


def test_box_center_jitter_smoothed():
    """박스 중심이 프레임마다 ±0.3 m 흔들려도 속도 오차가 작다 (프레임 차분이면 ±4 m/s)."""
    rng = np.random.default_rng(0)
    est = LaneSpeedEstimator(P, straight_map())
    vals = []
    for i in range(40):
        t = i * DT
        ego = EgoState(t, 10.0 * t, 0.0, 0.0, 10.0)
        n = rng.uniform(-0.3, 0.3, 2)
        o = obs_in_base(ego, 1, -25.0 + 14.0 * t + n[0], 3.5 + n[1], vx=14.0)
        vals.append(est.update(t, ego, [o])[1])
    assert max(abs(v - 14.0) for v in vals[15:]) < 1.0


def test_curved_lane_speed_along_lane():
    """반경 60 m 곡선 차선을 15 m/s로 따라오는 차량: 차선 방향 속도 15 m/s."""
    r = 60.0
    est = LaneSpeedEstimator(P, arc_map(r))
    vals = []
    for i in range(25):
        t = i * DT
        se, so = 10.0 + 8.0 * t, 15.0 * t - 5.0        # 자차 8 m/s, 대상 15 m/s (호 길이)
        ego = EgoState(t, r * math.sin(se / r), r - r * math.cos(se / r), se / r, 8.0)
        o = obs_in_base(ego, 1, r * math.sin(so / r), r - r * math.cos(so / r))
        vals.append(est.update(t, ego, [o])[1])
    assert vals[-1] == pytest.approx(15.0, abs=0.5)


def test_localization_jitter_cancels():
    """자차 map 위치가 ±0.5 m 흔들려도 같은 pose로 두 점을 옮기므로 속도에 영향이 없다."""
    rng = np.random.default_rng(1)
    est = LaneSpeedEstimator(P, straight_map())
    vals = []
    for i in range(30):
        t = i * DT
        true_ego_x = 10.0 * t
        obj_x = -20.0 + 12.0 * t
        j = rng.uniform(-0.5, 0.5)
        ego_meas = EgoState(t, true_ego_x + j, 0.0, 0.0, 10.0)   # 흔들린 localization
        ego_true = EgoState(t, true_ego_x, 0.0, 0.0, 10.0)
        o = obs_in_base(ego_true, 1, obj_x, 3.5, vx=12.0)          # LiDAR는 실제 상대 위치를 봄
        vals.append(est.update(t, ego_meas, [o])[1])
    assert max(abs(v - 12.0) for v in vals[10:]) < 0.5


def test_coasting_predicts_and_deleted_track_dropped():
    est = LaneSpeedEstimator(P, straight_map())
    for i in range(8):
        t = i * DT
        ego = EgoState(t, 10.0 * t, 0.0, 0.0, 10.0)
        est.update(t, ego, [obs_in_base(ego, 1, -20.0 + 13.0 * t, 3.5, vx=13.0)])
    t = 8 * DT
    ego = EgoState(t, 10.0 * t, 0.0, 0.0, 10.0)
    v = est.update(t, ego, [obs_in_base(ego, 1, 0.0, 3.5, vx=13.0, coasting=True)])[1]
    assert v == pytest.approx(13.0, abs=0.5)           # 위치는 무시하고 예측만
    assert est.update(t + DT, ego, []) == {} and est.tracks == {}

import math

import numpy as np
import pytest

from safety_monitor.common.debounce import CountDebounce
from safety_monitor.common.ego_history import EgoHistory
from safety_monitor.common.geometry import clip_polyline_ahead
from safety_monitor.common.types import EgoState
from safety_monitor.health.signal_watch import LOST, OK, UNINIT, HealthMonitor, SignalWatch
from safety_monitor.lanes.frenet import Polyline


def test_debounce_2_on_5_off():
    db = CountDebounce(2, 5)
    seq = [True, False, True, True, False, False, False, False, True, False,
           False, False, False, False]
    out = [db.update(c) for c in seq]
    assert out == [False, False, False, True, True, True, True, True, True,
                   True, True, True, True, False]


def test_frenet_straight_and_left_positive():
    pl = Polyline([[0, 0], [10, 0], [20, 0]])
    s, d, h = pl.project_one(12.0, 1.5)
    assert s == pytest.approx(12.0) and d == pytest.approx(1.5) and h == pytest.approx(0.0)
    s, d, _ = pl.project_one(-5.0, -2.0)      # 시작점 뒤 연장
    assert s == pytest.approx(-5.0) and d == pytest.approx(-2.0)


def test_frenet_curve():
    ang = np.linspace(0, math.pi / 2, 200)
    r = 50.0
    pl = Polyline(np.column_stack([r * np.sin(ang), r - r * np.cos(ang)]))  # 좌회전 원호
    s, d, _ = pl.project_one(0.0, 1.0)
    assert d == pytest.approx(1.0, abs=1e-3)
    assert pl.length == pytest.approx(r * math.pi / 2, rel=1e-3)


def test_signal_watch_states():
    w = SignalWatch("lidar", rate_hz=10, allowed_missing=3, release_time=0.68)
    assert w.evaluate(0.0) == UNINIT
    w.on_message(0.0)
    assert w.evaluate(0.2) == OK
    assert w.evaluate(0.31) == LOST
    t = 1.0
    w.on_message(t)                   # 복구 시작
    assert w.evaluate(t + 0.05) == LOST
    for k in range(1, 8):
        w.on_message(t + 0.1 * k)
    assert w.evaluate(t + 0.7) == OK


def test_signal_watch_gap_during_recovery_resets():
    w = SignalWatch("imu", rate_hz=10, allowed_missing=3, release_time=0.68)
    w.on_message(0.0)
    assert w.evaluate(0.5) == LOST
    w.on_message(0.6)
    assert w.evaluate(1.0) == LOST     # 0.4 s 공백 -> 다시 두절
    w.on_message(1.1)
    w.on_message(1.2)
    assert w.evaluate(1.25) == LOST    # 복구 시작 1.1, 아직 0.68 s 안 됨


def test_invalid_message_ignored():
    w = SignalWatch("ego", rate_hz=10, allowed_missing=3, release_time=0.68)
    w.on_message(0.0, valid=False)
    assert w.evaluate(0.1) == UNINIT


def test_health_monitor_not_ready_until_all_initialized():
    a = SignalWatch("a", 10, 3, 0.68)
    b = SignalWatch("b", 10, 3, 0.68)
    hm = HealthMonitor([a, b])
    a.on_message(0.0)
    estop, reason, _ = hm.evaluate(0.1)
    assert estop and reason == "not_ready:b"
    b.on_message(0.1)
    estop, reason, _ = hm.evaluate(0.15)
    assert not estop


def test_clip_polyline_ahead_starts_at_projection_and_cuts_at_length():
    # 5 m 간격, 자차 뒤 10 m부터 시작하는 local route와 같은 모양
    pts = [[x, 0.0] for x in range(-10, 101, 5)]
    out = clip_polyline_ahead(pts, 2.0, 0.4, 10.0)
    assert out[0] == pytest.approx([2.0, 0.0])
    assert out[-1] == pytest.approx([12.0, 0.0])
    assert np.sum(np.linalg.norm(np.diff(out, axis=0), axis=1)) == pytest.approx(10.0)


def test_clip_polyline_ahead_curve_and_short_route():
    ang = np.linspace(0, math.pi / 2, 200)
    pts = np.column_stack([50.0 * np.sin(ang), 50.0 - 50.0 * np.cos(ang)])
    out = clip_polyline_ahead(pts, 0.0, 0.0, 10.0)
    assert np.sum(np.linalg.norm(np.diff(out, axis=0), axis=1)) == pytest.approx(10.0)
    short = clip_polyline_ahead([[0, 0], [3, 0]], 0.0, 0.0, 10.0)
    assert short[-1] == pytest.approx([3.0, 0.0])
    full = clip_polyline_ahead([[0, 0], [30, 0]], 5.0, 0.0, 0.0)
    assert full[0] == pytest.approx([5.0, 0.0]) and full[-1] == pytest.approx([30.0, 0.0])


def test_ego_history_interpolates_to_message_time():
    h = EgoHistory()
    h.add(EgoState(0.0, 0.0, 0.0, math.pi - 0.1, 10.0))
    h.add(EgoState(0.1, 1.0, 2.0, -math.pi + 0.1, 12.0))    # yaw가 ±pi 경계를 넘음
    e = h.at(0.05)
    assert (e.x, e.y, e.speed) == pytest.approx((0.5, 1.0, 11.0))
    assert abs(e.yaw) == pytest.approx(math.pi)


def test_ego_history_extrapolation_limit():
    h = EgoHistory(max_extrapolation=0.2)
    h.add(EgoState(1.0, 0.0, 0.0, 0.0, 10.0))
    assert h.at(1.15).x == pytest.approx(1.5)                 # 등속 외삽
    assert h.at(1.3) is None and h.at(0.7) is None
    h.add(EgoState(0.5, 0.0, 0.0, 0.0, 10.0))                 # 시각이 뒤로 가면 이력 초기화
    assert h.latest().stamp == 0.5 and h.at(0.9) is None

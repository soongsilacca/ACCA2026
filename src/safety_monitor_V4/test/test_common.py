import math

import numpy as np
import pytest

from safety_monitor.common.debounce import CountDebounce
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

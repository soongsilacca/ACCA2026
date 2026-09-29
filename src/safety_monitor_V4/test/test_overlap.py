import math

import numpy as np
import pytest

from safety_monitor.collision.overlap import (INF, combine, entry_time,
                                              lateral_entry_progress, longitudinal_window)

L = 9.0
M = 0.5


class TestLongitudinalWindow:
    def test_faster_vehicle_from_behind(self):
        # 간격 10 m, 상대속도 5 m/s
        w = longitudinal_window(10.0, 5.0, L, M)
        assert w == pytest.approx((1.9, 3.9))

    def test_slower_vehicle_behind_never_overlaps(self):
        assert longitudinal_window(10.0, -2.0, L, M) is None

    def test_ego_catches_vehicle_ahead(self):
        # 대상 앞 끝이 자차 뒤 끝보다 15 m 앞, 자차가 5 m/s 빠름
        w = longitudinal_window(-15.0, -5.0, L, M)
        assert w == pytest.approx((1.1, 3.1))

    def test_relative_speed_zero_inside(self):
        assert longitudinal_window(0.0, 0.01, L, M) == (0.0, INF)

    def test_relative_speed_zero_outside(self):
        assert longitudinal_window(5.0, 0.01, L, M) is None

    def test_already_alongside_starts_now(self):
        w = longitudinal_window(-2.0, 3.0, L, M)
        assert w[0] == 0.0
        assert w[1] == pytest.approx((-2.0 + L + M) / 3.0)

    def test_already_passed(self):
        # 대상이 이미 자차 앞으로 지나감
        assert longitudinal_window(-12.0, 3.0, L, M) is None


class TestCombine:
    def test_enters_after_vehicle_passed(self):
        assert combine(4.0, (1.0, 3.0), 3.0) == (False, 4.0)

    def test_alongside_uses_entry_time(self):
        assert combine(1.0, (0.0, 2.0), 3.0) == (True, 1.0)

    def test_beyond_horizon(self):
        hit, t_c = combine(0.5, (3.5, 5.0), 3.0)
        assert not hit and t_c == 3.5

    def test_no_entry(self):
        assert combine(None, (0.0, 2.0), 3.0) == (False, None)


def _lane_change_route(n=301, length=60.0, offset=3.5):
    sigma = np.linspace(0, length, n)
    x = sigma
    u = np.clip((x - 5.0) / 30.0, 0, 1)
    y = offset * (3 * u ** 2 - 2 * u ** 3)
    dy = np.gradient(y, x)
    theta = np.arctan(dy)
    return sigma, y - offset, theta   # 목표 차선(y=offset) 기준 d


class TestLateralEntry:
    args = dict(front=3.845, rear=0.79, half_width=0.945, w_obs=1.8, lateral_margin=0.15)

    def test_straight_route_never_enters(self):
        sigma = np.linspace(0, 60, 100)
        d = np.full_like(sigma, -3.5)
        th = np.zeros_like(sigma)
        assert lateral_entry_progress(sigma, d, th, d_obs=0.0, **self.args) is None

    def test_lane_change_enters_mid_route(self):
        sigma, d, th = _lane_change_route()
        s = lateral_entry_progress(sigma, d, th, d_obs=0.0, **self.args)
        assert 5.0 < s < 30.0

    def test_heading_brings_entry_earlier(self):
        sigma, d, th = _lane_change_route()
        with_heading = lateral_entry_progress(sigma, d, th, d_obs=0.0, **self.args)
        no_heading = lateral_entry_progress(sigma, d, np.zeros_like(th), d_obs=0.0, **self.args)
        assert with_heading < no_heading

    def test_already_overlapping(self):
        sigma = np.linspace(0, 10, 11)
        d = np.full_like(sigma, -1.0)
        th = np.zeros_like(sigma)
        assert lateral_entry_progress(sigma, d, th, d_obs=0.0, **self.args) == 0.0

    def test_entry_time(self):
        assert entry_time(None, 5.0) is None
        assert entry_time(0.0, 0.0) == 0.0
        assert entry_time(10.0, 0.0) is None
        assert entry_time(10.0, 4.0) == pytest.approx(2.5)

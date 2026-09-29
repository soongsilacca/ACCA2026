"""6.1.2 ②③④: 횡방향 진입 시각, 종방향 겹침 구간, 두 구간의 결합.

모든 함수는 순수 함수이며 목표 차선 Frenet 좌표를 입력으로 받는다.
"""
import math

import numpy as np

INF = float("inf")


# ---------------------------------------------------------------- ② 횡방향
def ego_lateral_extent(route_d, route_theta, front, rear, half_width):
    """경로 위 각 지점에서 자차 차체가 차지하는 횡방향 구간 [lo, hi].

    route_d: base_link(뒤차축) 점의 목표 차선 기준 횡오프셋
    route_theta: 경로 접선과 중심선 사이 각 (자차 heading 각)
    네 모서리 중 가장 바깥 값을 쓴다. 차선 변경 방향 쪽 앞 모서리가 자동으로 선택된다.
    """
    d = np.asarray(route_d, dtype=float)[:, None]
    th = np.asarray(route_theta, dtype=float)[:, None]
    cx = np.array([front, front, -rear, -rear])[None, :]
    cy = np.array([half_width, -half_width, half_width, -half_width])[None, :]
    corners = d + cx * np.sin(th) + cy * np.cos(th)
    return corners.min(axis=1), corners.max(axis=1)


def lateral_entry_progress(route_sigma, route_d, route_theta, front, rear, half_width,
                           d_obs, w_obs, lateral_margin):
    """자차와 대상의 횡방향 구간이 처음 겹치는 경로 진행거리. 없으면 None.

    route_sigma[0] == 0 이 자차 현재 위치. 반환값 0이면 이미 횡방향으로 겹침.
    """
    lo, hi = ego_lateral_extent(route_d, route_theta, front, rear, half_width)
    obs_lo = d_obs - 0.5 * w_obs - lateral_margin
    obs_hi = d_obs + 0.5 * w_obs + lateral_margin
    hit = (hi > obs_lo) & (lo < obs_hi)
    if not np.any(hit):
        return None
    return float(np.asarray(route_sigma)[int(np.argmax(hit))])


def entry_time(sigma_enter, v_ref):
    """진입 시각 t_enter = sigma_enter / v_ref."""
    if sigma_enter is None:
        return None
    if sigma_enter <= 0.0:
        return 0.0
    if v_ref <= 1e-6:
        return None
    return sigma_enter / v_ref


# ---------------------------------------------------------------- ③ 종방향
def longitudinal_window(gap, v_rel, length_total, margin, eps=0.05):
    """종방향 겹침 구간 (t_start, t_end). 없으면 None.

    gap    = s(자차 뒤 끝) - s(대상 앞 끝)   (대상이 뒤에 떨어져 있으면 양수)
    v_rel  = v_obs - v_ego                   (대상이 빠르면 양수)
    D(t) = gap - v_rel * t 일 때 겹침 조건은 -(L_ego + L_obs + m) < D(t) < m.
    """
    d_hi = margin
    d_lo = -(length_total + margin)
    if abs(v_rel) <= eps:
        if d_lo < gap < d_hi:
            return (0.0, INF)
        return None
    t1 = (gap - d_hi) / v_rel
    t2 = (gap - d_lo) / v_rel
    t_a, t_b = (t1, t2) if t1 <= t2 else (t2, t1)
    if t_b < 0.0:
        return None
    return (max(t_a, 0.0), t_b)


# ---------------------------------------------------------------- ④ 결합
def combine(t_enter, window, horizon):
    """(충돌 여부, t_c). t_c = max(t_enter, t_start)."""
    if t_enter is None or window is None:
        return False, None
    t_c = max(t_enter, window[0])
    ok = t_c <= window[1] and t_c <= horizon
    return bool(ok), t_c


def is_finite(x):
    return x is not None and math.isfinite(x)

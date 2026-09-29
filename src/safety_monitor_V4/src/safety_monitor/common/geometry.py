import math

import numpy as np


def wrap_angle(a):
    """[-pi, pi) 범위로 정규화. 스칼라와 배열 모두 허용."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def base_to_map(ego, x, y):
    """base_link 좌표 (x, y)를 map 좌표로 변환."""
    c = math.cos(ego.yaw)
    s = math.sin(ego.yaw)
    return ego.x + c * x - s * y, ego.y + s * x + c * y


def rotate_to_map(ego, vx, vy):
    """base_link 축 벡터를 map 축으로 회전."""
    c = math.cos(ego.yaw)
    s = math.sin(ego.yaw)
    return c * vx - s * vy, s * vx + c * vy


def resample_polyline(points, step):
    """누적 길이 기준으로 등간격 재표본화. 반환: (pts, sigma)."""
    pts = np.asarray(points, dtype=float)
    if len(pts) < 2:
        return pts, np.zeros(len(pts))
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    keep = np.concatenate([[True], seg > 1e-6])
    pts = pts[keep]
    if len(pts) < 2:
        return pts, np.zeros(len(pts))
    cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
    n = max(2, int(math.floor(cum[-1] / step)) + 1)
    sigma = np.linspace(0.0, cum[-1], n)
    out = np.column_stack([np.interp(sigma, cum, pts[:, 0]),
                           np.interp(sigma, cum, pts[:, 1])])
    return out, sigma


def polyline_heading(pts):
    """각 점의 진행 방향. 마지막 점은 직전 구간 방향을 사용."""
    d = np.diff(pts, axis=0)
    h = np.arctan2(d[:, 1], d[:, 0])
    return np.concatenate([h, h[-1:]])

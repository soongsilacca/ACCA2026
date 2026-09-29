"""폴리라인 기준 Frenet 변환."""
import numpy as np

# 양 끝 구간은 이 길이까지 연장해서 투영한다.
_EXTRAPOLATE = 50.0


class Polyline:
    def __init__(self, points):
        pts = np.asarray(points, dtype=float)[:, :2]
        if len(pts) >= 2:
            seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
            pts = pts[np.concatenate([[True], seg > 1e-6])]
        if len(pts) < 2:
            raise ValueError("polyline needs at least 2 distinct points")
        self.pts = pts
        seg = np.diff(pts, axis=0)
        self.seg_len = np.linalg.norm(seg, axis=1)
        self.seg_dir = seg / self.seg_len[:, None]
        self.seg_heading = np.arctan2(self.seg_dir[:, 1], self.seg_dir[:, 0])
        self.s0 = np.concatenate([[0.0], np.cumsum(self.seg_len)])
        self.length = float(self.s0[-1])
        n = len(self.seg_len)
        self._lo = np.zeros(n)
        self._hi = self.seg_len.copy()
        self._lo[0] = -_EXTRAPOLATE
        self._hi[-1] = self.seg_len[-1] + _EXTRAPOLATE

    def project(self, xy):
        """점들을 (s, d, 중심선 heading)으로 변환. d는 좌측 양수.

        xy: (2,) 또는 (M, 2). 반환 배열 길이 M.
        """
        q = np.atleast_2d(np.asarray(xy, dtype=float))[:, :2]
        rel = q[:, None, :] - self.pts[None, :-1, :]
        t = np.einsum("mkj,kj->mk", rel, self.seg_dir)
        tc = np.clip(t, self._lo, self._hi)
        foot = self.pts[None, :-1, :] + tc[..., None] * self.seg_dir[None]
        dist2 = np.sum((q[:, None, :] - foot) ** 2, axis=2)
        k = np.argmin(dist2, axis=1)
        idx = np.arange(len(q))
        s = self.s0[k] + tc[idx, k]
        r = q - foot[idx, k]
        u = self.seg_dir[k]
        d = u[:, 0] * r[:, 1] - u[:, 1] * r[:, 0]
        return s, d, self.seg_heading[k]

    def project_one(self, x, y):
        s, d, h = self.project([x, y])
        return float(s[0]), float(d[0]), float(h[0])

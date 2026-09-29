"""MGeo 차선(link) 데이터: 최근접 차선 매칭과 차선 중심선 이어 붙이기.

MGeo link_set.json에서 사용하는 필드
- idx: link ID
- points: [[x, y, z], ...]  (MORAI map 좌표)
- from_node_idx, to_node_idx: 앞뒤 연결 (없으면 link 하나만 사용)
"""
import json
import math
import os
from collections import defaultdict

import numpy as np

from ..common.geometry import polyline_heading, resample_polyline, wrap_angle
from .frenet import Polyline

try:
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover
    cKDTree = None

_MATCH_STEP = 0.5


class Link:
    __slots__ = ("idx", "points", "from_node", "to_node", "length",
                 "start_heading", "end_heading")

    def __init__(self, idx, points, from_node=None, to_node=None):
        pts = np.asarray(points, dtype=float)[:, :2]
        self.idx = str(idx)
        self.points = pts
        self.from_node = from_node
        self.to_node = to_node
        seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        self.length = float(seg.sum())
        h = polyline_heading(pts)
        self.start_heading = float(h[0])
        self.end_heading = float(h[-2] if len(h) > 1 else h[0])


class LaneChain:
    """이어 붙인 차선 중심선과 포함된 link ID."""

    def __init__(self, polyline, link_ids, anchor):
        self.poly = polyline
        self.ids = frozenset(link_ids)
        self.ordered_ids = list(link_ids)
        self.anchor = anchor


class LaneMap:
    def __init__(self, links):
        """links: Link 객체 목록."""
        self.links = {}
        for ln in links:
            if len(ln.points) >= 2 and ln.length > 1e-3:
                self.links[ln.idx] = ln
        if not self.links:
            raise ValueError("no valid links")
        self._by_from = defaultdict(list)
        self._by_to = defaultdict(list)
        for ln in self.links.values():
            if ln.from_node is not None:
                self._by_from[ln.from_node].append(ln.idx)
            if ln.to_node is not None:
                self._by_to[ln.to_node].append(ln.idx)

        xy, head, owner = [], [], []
        for ln in self.links.values():
            pts, _ = resample_polyline(ln.points, _MATCH_STEP)
            if len(pts) < 2:
                continue
            xy.append(pts)
            head.append(polyline_heading(pts))
            owner.extend([ln.idx] * len(pts))
        self._xy = np.vstack(xy)
        self._head = np.concatenate(head)
        self._owner = np.asarray(owner, dtype=object)
        self._tree = cKDTree(self._xy) if cKDTree is not None else None
        self._chain_cache = {}

    # ------------------------------------------------------------------ load
    @classmethod
    def from_mgeo(cls, path):
        """path: link_set.json 파일 또는 그 파일이 있는 폴더."""
        if os.path.isdir(path):
            path = os.path.join(path, "link_set.json")
        with open(path, "r") as f:
            data = json.load(f)
        links = []
        for item in data:
            pts = item.get("points")
            if not pts or len(pts) < 2:
                continue
            links.append(Link(item["idx"], pts,
                              item.get("from_node_idx"), item.get("to_node_idx")))
        return cls(links)

    # ----------------------------------------------------------------- match
    def match(self, x, y, yaw=None, max_dist=3.0, max_heading_deg=45.0):
        """(x, y)에서 가장 가까운 link ID. yaw가 주어지면 진행 방향이 맞는 link만."""
        q = np.array([x, y], dtype=float)
        if self._tree is not None:
            k = min(32, len(self._xy))
            dist, idx = self._tree.query(q, k=k, distance_upper_bound=max_dist)
            dist = np.atleast_1d(dist)
            idx = np.atleast_1d(idx)
            ok = np.isfinite(dist)
            dist, idx = dist[ok], idx[ok]
        else:
            d_all = np.linalg.norm(self._xy - q, axis=1)
            idx = np.where(d_all <= max_dist)[0]
            dist = d_all[idx]
            order = np.argsort(dist)
            idx, dist = idx[order], dist[order]
        if len(idx) == 0:
            return None
        if yaw is not None:
            dh = np.abs(wrap_angle(self._head[idx] - yaw))
            good = dh <= math.radians(max_heading_deg)
            idx = idx[good]
        if len(idx) == 0:
            return None
        return self._owner[idx[0]]

    # ----------------------------------------------------------------- chain
    def _best_next(self, link_id, forward):
        ln = self.links[link_id]
        if forward:
            cands = self._by_from.get(ln.to_node, []) if ln.to_node is not None else []
            ref = ln.end_heading
            key = lambda c: abs(float(wrap_angle(self.links[c].start_heading - ref)))
        else:
            cands = self._by_to.get(ln.from_node, []) if ln.from_node is not None else []
            ref = ln.start_heading
            key = lambda c: abs(float(wrap_angle(self.links[c].end_heading - ref)))
        cands = [c for c in cands if c != link_id]
        if not cands:
            return None
        best = min(cands, key=key)
        if key(best) > math.radians(60.0):
            return None
        return best

    def chain(self, link_id, back=80.0, fwd=200.0):
        """link_id를 중심으로 앞뒤 link를 이어 붙인 중심선."""
        key = (link_id, round(back, 1), round(fwd, 1))
        if key in self._chain_cache:
            return self._chain_cache[key]
        seen = {link_id}
        fwd_ids, acc, cur = [], 0.0, link_id
        while acc < fwd:
            nxt = self._best_next(cur, True)
            if nxt is None or nxt in seen:
                break
            seen.add(nxt)
            fwd_ids.append(nxt)
            acc += self.links[nxt].length
            cur = nxt
        back_ids, acc, cur = [], 0.0, link_id
        while acc < back:
            prv = self._best_next(cur, False)
            if prv is None or prv in seen:
                break
            seen.add(prv)
            back_ids.append(prv)
            acc += self.links[prv].length
            cur = prv
        ordered = back_ids[::-1] + [link_id] + fwd_ids
        pts = []
        for lid in ordered:
            p = self.links[lid].points
            if pts and np.linalg.norm(pts[-1][-1] - p[0]) < 0.05:
                p = p[1:]
            if len(p):
                pts.append(p)
        chain = LaneChain(Polyline(np.vstack(pts)), ordered, link_id)
        if len(self._chain_cache) > 256:
            self._chain_cache.clear()
        self._chain_cache[key] = chain
        return chain

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
waypoint_global_path_planner_node.py

Self-contained ROS Waypoint Global Path Planner Node.
- Link(Lane)-based A* Graph Search (Vertices = Links)
- Transition Edges: Forward/Junction & Lane Changes with Penalties
- Smooth Diagonal Lane Change Blending (15.0m lookahead)
- Safe Same-Link Traversal Guard (Path Length > 1 for Loops)
- Monotonic Cubic Spline Path Smoothing & Resampling
- Publishes nav_msgs/Path on /global_path
- Supports multiple waypoints (Start -> Waypoint 1 -> Waypoint 2 -> ... -> Goal)
- Resets waypoints when a new start pose is received
"""

import os
import sys
import math
import json
import heapq
import numpy as np

import rospy
import rospkg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA


def make_color(r, g, b, a=1.0):
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = r, g, b, a
    return c


# =============================================================================
# Path Planner Core Logic
# =============================================================================

class GlobalPathPlanner:
    def __init__(self, node_file, link_file,
                 lane_change_penalty=15.0,
                 left_turn_penalty=10.0,
                 right_turn_penalty=5.0,
                 uturn_penalty=9999.0):
        self.lane_change_penalty = lane_change_penalty
        self.left_turn_penalty = left_turn_penalty
        self.right_turn_penalty = right_turn_penalty
        self.uturn_penalty = uturn_penalty

        self.nodes = {}
        self.links = []
        self.link_dict = {}
        self.link_lengths = {}
        self.node_outgoing_links = {}
        self.link_bboxes = []

        self._load(node_file, link_file)

    def _load(self, node_file, link_file):
        with open(node_file, 'r') as f:
            for n in json.load(f):
                self.nodes[n['idx']] = n

        with open(link_file, 'r') as f:
            self.links = json.load(f)

        self.link_dict = {lk['idx']: lk for lk in self.links}

        for lk in self.links:
            pts = lk['points']
            if not pts or len(pts) < 2:
                continue
            # Calculate 3D Link Length
            length = sum(
                math.sqrt((pts[i+1][0]-pts[i][0])**2 +
                          (pts[i+1][1]-pts[i][1])**2 +
                          (pts[i+1][2]-pts[i][2])**2)
                for i in range(len(pts)-1)
            )
            self.link_lengths[lk['idx']] = length
            
            # Map outgoing links per from-node
            fn = lk['from_node_idx']
            self.node_outgoing_links.setdefault(fn, []).append(lk)
            
            # Build Bounding Box for fast NN search
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            zs = [p[2] for p in pts]
            self.link_bboxes.append({
                'link': lk,
                'xmin': min(xs), 'xmax': max(xs),
                'ymin': min(ys), 'ymax': max(ys),
                'zmin': min(zs), 'zmax': max(zs),
            })

    def _dist3(self, a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2 + (a[2]-b[2])**2)

    def _normalize_angle(self, a):
        return (a + math.pi) % (2*math.pi) - math.pi

    def _proj_seg(self, p, a, b):
        """Project point p onto segment [a,b]. Returns (proj, dist_sq, t)."""
        vx, vy, vz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
        wx, wy, wz = p[0]-a[0], p[1]-a[1], p[2]-a[2]
        v2 = vx*vx + vy*vy + vz*vz
        if v2 < 1e-9:
            return list(a), wx*wx+wy*wy+wz*wz, 0.0
        t = max(0.0, min(1.0, (vx*wx+vy*wy+vz*wz)/v2))
        proj = [a[0]+t*vx, a[1]+t*vy, a[2]+t*vz]
        dx, dy, dz = p[0]-proj[0], p[1]-proj[1], p[2]-proj[2]
        return proj, dx*dx+dy*dy+dz*dz, t

    def _interp(self, a, b, t):
        return [a[i]+t*(b[i]-a[i]) for i in range(3)]

    def _downstream(self, link, seg_idx, seg_t, distance):
        """Walk 'distance' meters forward along link starting from (seg_idx, seg_t)."""
        pts = link['points']
        n = len(pts)
        if n < 2:
            return list(pts[0]), 0, 0.0
        seg_idx = min(seg_idx, n-2)
        p0 = self._interp(pts[seg_idx], pts[seg_idx+1], seg_t)
        accum = 0.0
        rem = self._dist3(p0, pts[seg_idx+1])
        if accum + rem >= distance:
            frac = (distance-accum)/rem if rem > 1e-9 else 0.0
            nt = seg_t + (1.0-seg_t)*frac
            return self._interp(pts[seg_idx], pts[seg_idx+1], nt), seg_idx, nt
        accum += rem
        for idx in range(seg_idx+1, n-1):
            sl = self._dist3(pts[idx], pts[idx+1])
            if accum + sl >= distance:
                t = (distance-accum)/sl if sl > 1e-9 else 0.0
                return self._interp(pts[idx], pts[idx+1], t), idx, t
            accum += sl
        return list(pts[-1]), n-2, 1.0

    def find_nearest_link(self, p, yaw=None):
        """Find the strictly nearest link, and check if user heading is valid (> 90 deg difference is invalid)."""
        px, py, pz = p[0], p[1], p[2]
        cands = []
        for bb in self.link_bboxes:
            dx = max(0.0, bb['xmin']-px, px-bb['xmax'])
            dy = max(0.0, bb['ymin']-py, py-bb['ymax'])
            dz = max(0.0, bb['zmin']-pz, pz-bb['zmax'])
            cands.append((dx*dx+dy*dy+dz*dz, bb['link']))
        cands.sort(key=lambda x: x[0])

        best_link = None
        best_seg = 0
        best_proj = None
        best_t = 0.0
        best_d2 = float('inf')

        # Strictly find the nearest link based on physical distance first
        for _, lk in cands[:30]:
            pts = lk['points']
            for i in range(len(pts)-1):
                proj, d2, t = self._proj_seg(p, pts[i], pts[i+1])
                if d2 < best_d2:
                    best_d2 = d2
                    best_link = lk
                    best_seg = i
                    best_proj = proj
                    best_t = t

        if best_link is None and cands:
            best_link = cands[0][1]; best_proj = list(best_link['points'][0])

        heading_valid = True
        if best_link is not None and yaw is not None:
            pts = best_link['points']
            seg_yaw = math.atan2(pts[best_seg+1][1]-pts[best_seg][1], pts[best_seg+1][0]-pts[best_seg][0])
            if abs(self._normalize_angle(seg_yaw - yaw)) > math.pi/2.0:
                heading_valid = False

        return best_link, best_seg, best_proj, best_t, heading_valid

    def find_nearest_link_candidates(self, p, yaw=None, max_dist=1.0):
        """Find all candidate links within max_dist from point p, sorted by distance.
        Always guarantees returning at least the single closest match even if outside max_dist."""
        px, py, pz = p[0], p[1], p[2]
        cands = []
        for bb in self.link_bboxes:
            dx = max(0.0, bb['xmin']-px, px-bb['xmax'])
            dy = max(0.0, bb['ymin']-py, py-bb['ymax'])
            dz = max(0.0, bb['zmin']-pz, pz-bb['zmax'])
            cands.append((dx*dx + dy*dy + dz*dz, bb['link']))
            
        valid_cands = []
        max_dist_sq = max_dist * max_dist
        
        # Sort candidates by coarse distance and take top 30
        cands.sort(key=lambda x: x[0])
        for _, lk in cands[:30]:
            pts = lk['points']
            best_d2 = float('inf')
            best_seg = 0
            best_proj = None
            best_t = 0.0
            for i in range(len(pts)-1):
                proj, d2, t = self._proj_seg(p, pts[i], pts[i+1])
                if d2 < best_d2:
                    best_d2 = d2
                    best_seg = i
                    best_proj = proj
                    best_t = t
            valid_cands.append((best_d2, lk, best_seg, best_proj, best_t))
            
        valid_cands.sort(key=lambda x: x[0])
        
        result = []
        if valid_cands:
            nearest_d2 = valid_cands[0][0]
            for d2, lk, seg, proj, t in valid_cands:
                # Keep only if it is within max_dist_sq, or is the single absolute closest link
                if d2 <= max_dist_sq or d2 == nearest_d2:
                    result.append((lk, seg, proj, t, d2))
                        
        return result

    def _proj_link(self, p, link):
        """Project point onto a specific link."""
        best_d2 = float('inf'); best_proj = None; best_seg = 0; best_t = 0.0
        for i in range(len(link['points'])-1):
            proj, d2, t = self._proj_seg(p, link['points'][i], link['points'][i+1])
            if d2 < best_d2:
                best_d2 = d2; best_proj = proj; best_seg = i; best_t = t
        return best_proj, best_seg, best_t

    def _classify_turn(self, la, lb):
        pa, pb = la['points'], lb['points']
        if len(pa) < 2 or len(pb) < 2:
            return 'forward'
        ya = math.atan2(pa[-1][1]-pa[-2][1], pa[-1][0]-pa[-2][0])
        yb = math.atan2(pb[1][1]-pb[0][1], pb[1][0]-pb[0][0])
        deg = math.degrees(self._normalize_angle(yb-ya))
        if abs(deg) < 20:       return 'forward'
        elif 20 <= deg < 135:   return 'left'
        elif -135 < deg <= -20: return 'right'
        else:                   return 'uturn'

    def _transitions(self, link_idx):
        lk = self.link_dict.get(link_idx)
        if not lk:
            return []
        length = self.link_lengths.get(link_idx, 0.0)
        result = []
        # Forward and junction edges (is_lc = False)
        for nl in self.node_outgoing_links.get(lk['to_node_idx'], []):
            pen = {'left': self.left_turn_penalty,
                   'right': self.right_turn_penalty,
                   'uturn': self.uturn_penalty}.get(self._classify_turn(lk, nl), 0.0)
            result.append((nl['idx'], length + pen, False))
        # Lane change transitions (is_lc = True)
        for key in ('left_lane_change_dst_link_idx', 'right_lane_change_dst_link_idx'):
            flag = 'can_move_left_lane' if 'left' in key else 'can_move_right_lane'
            if lk.get(flag) and lk.get(key) and lk[key] in self.link_dict:
                result.append((lk[key], length + self.lane_change_penalty, True))
        return result

    def _link_center(self, lk):
        pts = lk['points']
        return pts[len(pts)//2] if pts else [0,0,0]

    def a_star(self, start_idx, goal_idx, s_seg, s_t, g_seg, g_t, same_link_direct=True):
        gl = self.link_dict.get(goal_idx)
        sl = self.link_dict.get(start_idx)
        if not gl or not sl:
            return None
        gc = self._link_center(gl)
        h0 = self._dist3(self._link_center(sl), gc)
        
        # State: (f_score, g_score, cur, cur_seg, cur_t, path)
        q = [(h0, 0.0, start_idx, s_seg, s_t, [start_idx])]
        best_g = {(start_idx, s_seg): 0.0}
        
        while q:
            f, g, cur, cur_seg, cur_t, path = heapq.heappop(q)
            
            if cur == goal_idx:
                # Goal condition check:
                # 1. If same_link_direct is False and len(path) == 1, we cannot accept it.
                # 2. Otherwise, we can only accept if the final position (cur_seg, cur_t) is upstream of (g_seg, g_t).
                is_valid_goal = True
                if not same_link_direct and len(path) == 1:
                    is_valid_goal = False
                elif cur_seg > g_seg or (cur_seg == g_seg and cur_t > g_t):
                    is_valid_goal = False
                
                if is_valid_goal:
                    return path
                    
            if best_g.get((cur, cur_seg), float('inf')) < g:
                # Loopback check: if we are returning to start_idx to complete a loop, allow it
                if not (cur == start_idx and len(path) > 1):
                    continue
                    
            for nxt, cost, is_lc in self._transitions(cur):
                ng = g + cost
                
                # Determine new segment and t position on nxt link
                if is_lc:
                    Lc = self.link_dict[cur]
                    Ln = self.link_dict[nxt]
                    
                    pts_c = Lc['points']
                    curr_pos = self._interp(pts_c[cur_seg], pts_c[cur_seg+1], cur_t) if cur_seg < len(pts_c)-1 else list(pts_c[-1])
                    
                    _, sns, tns = self._proj_link(curr_pos, Ln)
                    
                    pts_n = Ln['points']
                    p_start_n = self._interp(pts_n[sns], pts_n[sns+1], tns)
                    if nxt == goal_idx:
                        p_goal_n = self._interp(pts_n[g_seg], pts_n[g_seg+1], g_t) if g_seg < len(pts_n)-1 else list(pts_n[-1])
                        if sns == g_seg:
                            rem_dist = self._dist3(p_start_n, p_goal_n)
                        else:
                            rem_dist = self._dist3(p_start_n, pts_n[sns+1])
                            for idx in range(sns+1, g_seg):
                                rem_dist += self._dist3(pts_n[idx], pts_n[idx+1])
                            rem_dist += self._dist3(pts_n[g_seg], p_goal_n)
                    else:
                        rem_dist = self._dist3(p_start_n, pts_n[sns+1])
                        for idx in range(sns+1, len(pts_n)-1):
                            rem_dist += self._dist3(pts_n[idx], pts_n[idx+1])
                            
                    lc_len = max(3.0, min(15.0, rem_dist - 0.5))
                    _, nxt_seg, nxt_t = self._downstream(Ln, sns, tns, lc_len)
                else:
                    # Normal forward transition: we start at the beginning of the next link
                    nxt_seg = 0
                    nxt_t = 0.0
                    
                is_loopback = (nxt == start_idx and not same_link_direct)
                state_key = (nxt, nxt_seg)
                if is_loopback or ng < best_g.get(state_key, float('inf')):
                    if not is_loopback:
                        best_g[state_key] = ng
                    h = self._dist3(self._link_center(self.link_dict[nxt]), gc)
                    heapq.heappush(q, (ng+h, ng, nxt, nxt_seg, nxt_t, path+[nxt]))
                    
        return None

    def _smooth(self, raw_pts, interval):
        # CubicSpline is removed to prevent scipy dependency. Falling back to linear interpolation.
        return self._linear(raw_pts, interval)

    def _linear(self, pts, interval):
        pts = np.array(pts, dtype=float)
        if len(pts) < 2: return []
        dists = np.linalg.norm(np.diff(pts, axis=0), axis=1)
        total = np.sum(dists)
        if total < 1e-6:
            return [{'x': float(pts[0,0]), 'y': float(pts[0,1]),
                     'z': 0.0, 'yaw': 0.0, 'curvature': 0.0}]
        sn = np.linspace(0, total, max(2, int(math.ceil(total/interval))+1))
        so = np.zeros(len(pts)); so[1:] = np.cumsum(dists)
        xn = np.interp(sn, so, pts[:,0])
        yn = np.interp(sn, so, pts[:,1])
        zn = np.interp(sn, so, pts[:,2])
        yaws = []
        for i in range(len(sn)):
            if i < len(sn)-1:
                yaws.append(math.atan2(yn[i+1]-yn[i], xn[i+1]-xn[i]))
            else:
                yaws.append(yaws[-1] if yaws else 0.0)
        return [{'x': float(xn[i]), 'y': float(yn[i]), 'z': 0.0,
                 'yaw': yaws[i], 'curvature': 0.0} for i in range(len(sn))]

    def plan_path(self, start_p, goal_p, start_yaw=None, goal_yaw=None,
                  use_smoothing=True, resample_interval=0.5):
        # 1. Fetch raw candidate links sorted by distance
        s_cands_raw = self.find_nearest_link_candidates(start_p, start_yaw, max_dist=1.0)
        g_cands_raw = self.find_nearest_link_candidates(goal_p, goal_yaw, max_dist=1.0)
        
        if not s_cands_raw or not g_cands_raw:
            return None, None
            
        # 2. Filter candidates based on heading matching (yaw angle difference within 90 deg)
        s_cands = []
        for lk, seg, proj, t, d2 in s_cands_raw:
            heading_valid = True
            if start_yaw is not None:
                pts = lk['points']
                seg_yaw = math.atan2(pts[seg+1][1]-pts[seg][1], pts[seg+1][0]-pts[seg][0])
                if abs(self._normalize_angle(seg_yaw - start_yaw)) > math.pi/2.0:
                    heading_valid = False
            if heading_valid:
                s_cands.append((lk, seg, proj, t, d2))
                
        g_cands = []
        for lk, seg, proj, t, d2 in g_cands_raw:
            heading_valid = True
            if goal_yaw is not None:
                pts = lk['points']
                seg_yaw = math.atan2(pts[seg+1][1]-pts[seg][1], pts[seg+1][0]-pts[seg][0])
                if abs(self._normalize_angle(seg_yaw - goal_yaw)) > math.pi/2.0:
                    heading_valid = False
            if heading_valid:
                g_cands.append((lk, seg, proj, t, d2))

        # 3. Reject if no candidate has matching heading
        if not s_cands:
            rospy.logwarn("[Path Planner] Start pose heading does not match any near lane direction! Path creation rejected.")
            return None, None
        if not g_cands:
            rospy.logwarn("[Path Planner] Goal pose heading does not match any near lane direction! Path creation rejected.")
            return None, None

        # 4. Check if we should use multi-candidate lane matching (if 1st raw closest candidate is a junction link)
        sl_1st = s_cands_raw[0][0]
        gl_1st = g_cands_raw[0][0]
        junction_signals = ['straight', 'left', 'right_unprotected', 'left_unprotected', 'uturn_normal']
        use_multi_cands = (sl_1st.get('related_signal') in junction_signals or gl_1st.get('related_signal') in junction_signals)
        
        best_pair = None
        
        if use_multi_cands:
            # Multi-candidate search over heading-valid candidates
            best_total_cost = float('inf')
            for sl_cand, s_seg_cand, s_proj_cand, s_t_cand, s_d2 in s_cands[:3]:
                for gl_cand, g_seg_cand, g_proj_cand, g_t_cand, g_d2 in g_cands[:3]:
                    same_link_direct = False
                    if sl_cand['idx'] == gl_cand['idx']:
                        if s_seg_cand < g_seg_cand or (s_seg_cand == g_seg_cand and s_t_cand <= g_t_cand):
                            same_link_direct = True
                            
                    if same_link_direct:
                        cost = self._dist3(s_proj_cand, g_proj_cand) + 0.1 * (s_d2 + g_d2)
                        seq = [sl_cand['idx']]
                    else:
                        seq = self.a_star(sl_cand['idx'], gl_cand['idx'], s_seg_cand, s_t_cand, g_seg_cand, g_t_cand, same_link_direct=same_link_direct)
                        if not seq:
                            continue
                        cost = 0.0
                        for i in range(len(seq)-1):
                            cost += self.link_lengths.get(seq[i], 0.0)
                        cost += self._dist3(s_proj_cand, self.link_dict[seq[0]]['points'][-1])
                        cost += 0.1 * (s_d2 + g_d2)
                        
                    if cost < best_total_cost:
                        best_total_cost = cost
                        best_pair = (sl_cand, s_seg_cand, s_proj_cand, s_t_cand, gl_cand, g_seg_cand, g_proj_cand, g_t_cand, same_link_direct, seq)
        else:
            # Single absolute closest lane (enforce lane change testing in normal road segments)
            sl, s_seg, s_proj, s_t, s_d2 = s_cands[0]
            gl, g_seg, g_proj, g_t, g_d2 = g_cands[0]
            
            same_link_direct = False
            if sl['idx'] == gl['idx']:
                if s_seg < g_seg or (s_seg == g_seg and s_t <= g_t):
                    same_link_direct = True
                    
            if same_link_direct:
                seq = [sl['idx']]
            else:
                seq = self.a_star(sl['idx'], gl['idx'], s_seg, s_t, g_seg, g_t, same_link_direct=same_link_direct)
                
            if seq:
                best_pair = (sl, s_seg, s_proj, s_t, gl, g_seg, g_proj, g_t, same_link_direct, seq)
                
        if best_pair is None:
            rospy.logwarn("[Path Planner] No valid path could be planned.")
            return None, None
            
        # Unpack the best pair
        sl, s_seg, s_proj, s_t, gl, g_seg, g_proj, g_t, same_link_direct, seq = best_pair
        
        # Build raw points using the corrected Path Assembly algorithm
        if same_link_direct:
            raw = [s_proj]
            if s_seg == g_seg:
                pass
            else:
                for idx in range(s_seg+1, g_seg+1):
                    raw.append(list(sl['points'][idx]))
            raw.append(g_proj)
            pts = self._smooth(raw, resample_interval) if use_smoothing else self._linear(raw, resample_interval)
            return pts, [sl['idx']]
            
        raw = [s_proj]
        curr_seg = s_seg
        curr_t = s_t

        for i in range(len(seq)-1):
            Lc = self.link_dict[seq[i]]
            Ln = self.link_dict[seq[i+1]]
            is_lc = (Lc.get('left_lane_change_dst_link_idx') == Ln['idx'] or
                     Lc.get('right_lane_change_dst_link_idx') == Ln['idx'])

            if is_lc:
                curr_pos = raw[-1]
                # Find projections on both Lc and Ln
                _, scs, tcs = self._proj_link(curr_pos, Lc)
                _, sns, tns = self._proj_link(curr_pos, Ln)

                # Calculate remaining distance on Ln
                pts_n = Ln['points']
                p_start_n = self._interp(pts_n[sns], pts_n[sns+1], tns)
                if Ln['idx'] == seq[-1]:
                    # If Ln is the final link, limit by goal point
                    p_goal_n = self._interp(pts_n[g_seg], pts_n[g_seg+1], g_t) if g_seg < len(pts_n)-1 else list(pts_n[-1])
                    if sns == g_seg:
                        rem_dist = self._dist3(p_start_n, p_goal_n)
                    else:
                        rem_dist = self._dist3(p_start_n, pts_n[sns+1])
                        for idx in range(sns+1, g_seg):
                            rem_dist += self._dist3(pts_n[idx], pts_n[idx+1])
                        rem_dist += self._dist3(pts_n[g_seg], p_goal_n)
                else:
                    # Limit by Ln end node
                    rem_dist = self._dist3(p_start_n, pts_n[sns+1])
                    for idx in range(sns+1, len(pts_n)-1):
                        rem_dist += self._dist3(pts_n[idx], pts_n[idx+1])

                # Limit lc_len appropriately (minimum 3.0m, maximum 15.0m)
                lc_len = max(3.0, min(15.0, rem_dist - 0.5))

                step_m = 0.5
                steps = int(round(lc_len/step_m))
                for step in range(1, steps+1):
                    d = step * step_m
                    w = d / lc_len
                    pc, _, _ = self._downstream(Lc, scs, tcs, d)
                    pn, _, _ = self._downstream(Ln, sns, tns, d)
                    raw.append([(1-w)*pc[j]+w*pn[j] for j in range(3)])

                _, curr_seg, curr_t = self._downstream(Ln, sns, tns, lc_len)
            else:
                # If it's a normal transition (forward/junction), traverse Lc to the end first
                for idx in range(curr_seg+1, len(Lc['points'])):
                    raw.append(list(Lc['points'][idx]))
                # Transition to Ln starting at the beginning
                curr_seg = 0
                curr_t = 0.0

        # Traverse final link up to goal segment
        Lf = self.link_dict[seq[-1]]
        for idx in range(curr_seg+1, g_seg+1):
            if idx < len(Lf['points']):
                raw.append(list(Lf['points'][idx]))
        raw.append(g_proj)

        pts = self._smooth(raw, resample_interval) if use_smoothing else self._linear(raw, resample_interval)
        return pts, seq


# =============================================================================
# ROS Node Wrapper
# =============================================================================

class WaypointGlobalPathPlannerNode:
    def __init__(self):
        rospy.init_node('waypoint_global_path_planner_node', anonymous=False)

        # 1. ROS Parameters
        rospack = rospkg.RosPack()
        try:
            hdmap_loader_path = rospack.get_path('hdmap_loader')
        except Exception:
            hdmap_loader_path = ""
            rospy.logwarn("[Waypoint Path Planner] hdmap_loader package not found, please check workspace.")

        node_file = rospy.get_param(
            '~node_file', os.path.join(hdmap_loader_path, 'scripts', 'node_set.json') if hdmap_loader_path else ""
        )
        link_file = rospy.get_param(
            '~link_file', os.path.join(hdmap_loader_path, 'scripts', 'link_set.json') if hdmap_loader_path else ""
        )

        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.use_smoothing = rospy.get_param('~use_smoothing', True)
        self.resample_interval = rospy.get_param('~resample_interval', 0.5)

        # Configurable penalties
        lane_change_penalty = rospy.get_param('~lane_change_penalty', 15.0)
        left_turn_penalty = rospy.get_param('~left_turn_penalty', 10.0)
        right_turn_penalty = rospy.get_param('~right_turn_penalty', 5.0)
        uturn_penalty = rospy.get_param('~uturn_penalty', 9999.0)

        # Initialize the advanced planner
        rospy.loginfo("[Waypoint Path Planner] Initializing Advanced Path Planner...")
        self.planner = GlobalPathPlanner(
            node_file=node_file,
            link_file=link_file,
            lane_change_penalty=lane_change_penalty,
            left_turn_penalty=left_turn_penalty,
            right_turn_penalty=right_turn_penalty,
            uturn_penalty=uturn_penalty
        )
        rospy.loginfo("[Waypoint Path Planner] Advanced Path Planner initialized successfully.")

        # State Variables for waypoint navigation
        self.start_pose = None
        self.waypoints = []  # List of geometry_msgs/Pose for intermediate goals/waypoints

        # Subscribers
        rospy.Subscriber('/initial_pose', PoseStamped, self.start_pose_stamped_cb)
        rospy.Subscriber('/initial_pose', PoseWithCovarianceStamped, self.start_pose_with_cov_cb)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.start_pose_with_cov_cb)

        rospy.Subscriber('/goal_pose', PoseStamped, self.goal_pose_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_cb)

        # Publishers
        self.path_pub = rospy.Publisher('/global_path', Path, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher('/global_path_markers', MarkerArray, queue_size=1, latch=True)

        rospy.loginfo("[Waypoint Path Planner] Waiting for start (2D Pose Estimate) and waypoints (2D Nav Goal) from RViz...")

    def start_pose_stamped_cb(self, msg):
        self.handle_start_pose(msg.pose)

    def start_pose_with_cov_cb(self, msg):
        self.handle_start_pose(msg.pose.pose)

    def handle_start_pose(self, pose):
        rospy.loginfo("[Waypoint Path Planner] Resetting path! New Start Pose set at x={:.2f}, y={:.2f}".format(
            pose.position.x, pose.position.y))
        self.start_pose = pose
        self.waypoints = []  # Reset all waypoints
        self.clear_path()
        self.clear_markers()
        self.plan_and_publish()

    def goal_pose_cb(self, msg):
        if self.start_pose is None:
            rospy.logwarn("[Waypoint Path Planner] Cannot add waypoint! Please set Start Pose first using 2D Pose Estimate.")
            return

        new_waypoint = msg.pose
        rospy.loginfo("[Waypoint Path Planner] Adding new waypoint: x={:.2f}, y={:.2f}".format(
            new_waypoint.position.x, new_waypoint.position.y))
        self.waypoints.append(new_waypoint)

        # Compute combined path and update
        success = self.plan_and_publish()
        if not success:
            rospy.logerr("[Waypoint Path Planner] Failed to plan to the new waypoint. Reverting to last valid configuration.")
            self.waypoints.pop()
            # Plan again with the remaining valid waypoints
            self.plan_and_publish()

    def get_yaw_from_pose(self, pose):
        # Extract yaw from quaternion (x, y, z, w)
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def clear_path(self):
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id
        self.path_pub.publish(path_msg)

    def clear_markers(self):
        ma = MarkerArray()
        m = Marker()
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.marker_pub.publish(ma)

    def plan_and_publish(self):
        if self.start_pose is None:
            self.clear_path()
            self.clear_markers()
            return False

        # If we have start but no waypoints, publish only the start point marker
        if not self.waypoints:
            self.clear_path()
            start_p = [self.start_pose.position.x, self.start_pose.position.y, 0.0]
            start_link, _, start_proj, _, _ = self.planner.find_nearest_link(start_p)
            if start_proj:
                start_proj[2] = 0.0
                self.publish_markers(start_p, start_proj, [])
            return True

        rospy.loginfo("[Waypoint Path Planner] Computing global path through {} waypoints...".format(len(self.waypoints)))

        all_poses = [self.start_pose] + self.waypoints
        combined_path = []
        projected_points = []

        # Find projection of the starting point
        start_p = [self.start_pose.position.x, self.start_pose.position.y, 0.0]
        _, _, start_proj, _, _ = self.planner.find_nearest_link(start_p)
        if start_proj:
            start_proj[2] = 0.0
        projected_points.append(start_proj)

        for i in range(len(all_poses) - 1):
            p_from = all_poses[i]
            p_to = all_poses[i+1]

            from_p = [p_from.position.x, p_from.position.y, 0.0]
            to_p = [p_to.position.x, p_to.position.y, 0.0]

            from_yaw = self.get_yaw_from_pose(p_from)
            to_yaw = self.get_yaw_from_pose(p_to)

            resampled_path, _ = self.planner.plan_path(
                from_p, to_p,
                start_yaw=from_yaw,
                goal_yaw=to_yaw,
                use_smoothing=self.use_smoothing,
                resample_interval=self.resample_interval
            )

            if not resampled_path:
                rospy.logerr("[Waypoint Path Planner] Failed to plan path segment from waypoint {} to {}!".format(i, i+1))
                return False

            # Record projected point for marker visualization
            _, _, to_proj, _, _ = self.planner.find_nearest_link(to_p)
            if to_proj:
                to_proj[2] = 0.0
            projected_points.append(to_proj)

            # Stitch path segments together
            if not combined_path:
                combined_path.extend(resampled_path)
            else:
                p_last = combined_path[-1]
                p_first = resampled_path[0]
                dist = math.sqrt((p_last['x'] - p_first['x'])**2 + (p_last['y'] - p_first['y'])**2)
                # Avoid inserting a duplicate node if they are very close
                if dist < 0.1:
                    combined_path.extend(resampled_path[1:])
                else:
                    combined_path.extend(resampled_path)

        rospy.loginfo("[Waypoint Path Planner] Path generated successfully. Total points: {}".format(len(combined_path)))

        # Convert to ROS nav_msgs/Path
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = self.frame_id

        for pt in combined_path:
            pose = PoseStamped()
            pose.header.stamp = path_msg.header.stamp
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = pt['x']
            pose.pose.position.y = pt['y']
            pose.pose.position.z = 0.0

            # Set orientation from yaw
            yaw = pt['yaw']
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        # Publish visual markers
        self.publish_markers(
            start_p=start_p,
            start_proj=projected_points[0],
            waypoint_data=list(zip(all_poses[1:], projected_points[1:]))
        )
        return True

    def publish_markers(self, start_p, start_proj, waypoint_data):
        # Clear existing markers
        self.clear_markers()

        ma = MarkerArray()
        stamp = rospy.Time.now()
        marker_id = 0

        # --- Start Point Markers ---
        # 1. Start clicked point (Green sphere)
        m_start_click = Marker()
        m_start_click.header.frame_id = self.frame_id
        m_start_click.header.stamp = stamp
        m_start_click.ns = "start_points"
        m_start_click.id = marker_id
        marker_id += 1
        m_start_click.type = Marker.SPHERE
        m_start_click.action = Marker.ADD
        m_start_click.pose.position.x = start_p[0]
        m_start_click.pose.position.y = start_p[1]
        m_start_click.pose.position.z = 0.0
        m_start_click.pose.orientation.w = 1.0
        m_start_click.scale.x = 1.5
        m_start_click.scale.y = 1.5
        m_start_click.scale.z = 1.5
        m_start_click.color = make_color(0.1, 0.9, 0.1, 0.9)  # Green
        ma.markers.append(m_start_click)

        # 2. Start projected point (Lime sphere)
        m_start_proj = Marker()
        m_start_proj.header.frame_id = self.frame_id
        m_start_proj.header.stamp = stamp
        m_start_proj.ns = "start_points"
        m_start_proj.id = marker_id
        marker_id += 1
        m_start_proj.type = Marker.SPHERE
        m_start_proj.action = Marker.ADD
        m_start_proj.pose.position.x = start_proj[0]
        m_start_proj.pose.position.y = start_proj[1]
        m_start_proj.pose.position.z = 0.0
        m_start_proj.pose.orientation.w = 1.0
        m_start_proj.scale.x = 1.0
        m_start_proj.scale.y = 1.0
        m_start_proj.scale.z = 1.0
        m_start_proj.color = make_color(0.5, 1.0, 0.5, 0.8)  # Lime
        ma.markers.append(m_start_proj)

        # 3. Start connection line
        m_start_line = Marker()
        m_start_line.header.frame_id = self.frame_id
        m_start_line.header.stamp = stamp
        m_start_line.ns = "connection_lines"
        m_start_line.id = marker_id
        marker_id += 1
        m_start_line.type = Marker.LINE_STRIP
        m_start_line.action = Marker.ADD
        m_start_line.scale.x = 0.25
        m_start_line.color = make_color(0.2, 0.8, 0.2, 0.6)  # Green Line
        m_start_line.pose.orientation.w = 1.0
        m_start_line.points.append(Point(start_p[0], start_p[1], 0.0))
        m_start_line.points.append(Point(start_proj[0], start_proj[1], 0.0))
        ma.markers.append(m_start_line)

        # 4. Start Text label
        m_start_text = Marker()
        m_start_text.header.frame_id = self.frame_id
        m_start_text.header.stamp = stamp
        m_start_text.ns = "text_labels"
        m_start_text.id = marker_id
        marker_id += 1
        m_start_text.type = Marker.TEXT_VIEW_FACING
        m_start_text.action = Marker.ADD
        m_start_text.pose.position.x = start_p[0]
        m_start_text.pose.position.y = start_p[1]
        m_start_text.pose.position.z = 1.5
        m_start_text.pose.orientation.w = 1.0
        m_start_text.scale.z = 1.2
        m_start_text.color = make_color(1.0, 1.0, 1.0, 1.0)
        m_start_text.text = "Start"
        ma.markers.append(m_start_text)

        # --- Waypoint/Goal Markers ---
        num_waypoints = len(waypoint_data)
        for idx, (pose, proj) in enumerate(waypoint_data):
            is_final_goal = (idx == num_waypoints - 1)

            if is_final_goal:
                ns = "goal_points"
                color_click = make_color(0.9, 0.1, 0.1, 0.9)  # Red
                color_proj = make_color(1.0, 0.5, 0.5, 0.8)   # Light Red
                color_line = make_color(0.8, 0.2, 0.2, 0.6)   # Red Line
                label = "Goal"
            else:
                ns = "waypoint_points"
                color_click = make_color(0.9, 0.5, 0.1, 0.9)  # Orange
                color_proj = make_color(1.0, 0.7, 0.4, 0.8)   # Light Orange
                color_line = make_color(0.8, 0.5, 0.2, 0.6)   # Orange Line
                label = "Waypoint {}".format(idx + 1)

            # Clicked point
            m_click = Marker()
            m_click.header.frame_id = self.frame_id
            m_click.header.stamp = stamp
            m_click.ns = ns
            m_click.id = marker_id
            marker_id += 1
            m_click.type = Marker.SPHERE
            m_click.action = Marker.ADD
            m_click.pose.position.x = pose.position.x
            m_click.pose.position.y = pose.position.y
            m_click.pose.position.z = 0.0
            m_click.pose.orientation.w = 1.0
            m_click.scale.x = 1.5
            m_click.scale.y = 1.5
            m_click.scale.z = 1.5
            m_click.color = color_click
            ma.markers.append(m_click)

            # Projected point
            m_proj = Marker()
            m_proj.header.frame_id = self.frame_id
            m_proj.header.stamp = stamp
            m_proj.ns = ns
            m_proj.id = marker_id
            marker_id += 1
            m_proj.type = Marker.SPHERE
            m_proj.action = Marker.ADD
            m_proj.pose.position.x = proj[0]
            m_proj.pose.position.y = proj[1]
            m_proj.pose.position.z = 0.0
            m_proj.pose.orientation.w = 1.0
            m_proj.scale.x = 1.0
            m_proj.scale.y = 1.0
            m_proj.scale.z = 1.0
            m_proj.color = color_proj
            ma.markers.append(m_proj)

            # Connection line
            m_line = Marker()
            m_line.header.frame_id = self.frame_id
            m_line.header.stamp = stamp
            m_line.ns = "connection_lines"
            m_line.id = marker_id
            marker_id += 1
            m_line.type = Marker.LINE_STRIP
            m_line.action = Marker.ADD
            m_line.scale.x = 0.25
            m_line.color = color_line
            m_line.pose.orientation.w = 1.0
            m_line.points.append(Point(pose.position.x, pose.position.y, 0.0))
            m_line.points.append(Point(proj[0], proj[1], 0.0))
            ma.markers.append(m_line)

            # Text label
            m_text = Marker()
            m_text.header.frame_id = self.frame_id
            m_text.header.stamp = stamp
            m_text.ns = "text_labels"
            m_text.id = marker_id
            marker_id += 1
            m_text.type = Marker.TEXT_VIEW_FACING
            m_text.action = Marker.ADD
            m_text.pose.position.x = pose.position.x
            m_text.pose.position.y = pose.position.y
            m_text.pose.position.z = 1.5
            m_text.pose.orientation.w = 1.0
            m_text.scale.z = 1.2
            m_text.color = make_color(1.0, 1.0, 1.0, 1.0)
            m_text.text = label
            ma.markers.append(m_text)

        self.marker_pub.publish(ma)


if __name__ == '__main__':
    try:
        WaypointGlobalPathPlannerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Collision Imminent E-STOP (report section 8.5.2).

Independent non-learning safety layer. Consumes raw LiDAR and ego velocity
only -- never model or controller output. Publishes /safety/estop_trigger.

Corridor source
----------------
The swept path is taken from /local_route when it is fresh and well-formed;
otherwise the node falls back to an arc derived from yaw rate (R = v/ω). The
fallback keeps the node judgeable when the route topic dies, timeouts out, or
carries non-finite values -- the same independence principle as before, now
with a more accurate primary source when the route is healthy.

Watch range
-----------
The corridor length scales with speed so distant obstacles are not missed at
high speed, but it must never shrink while an E-STOP is latched: braking
itself lowers v, and a naive v-based range would let the corridor contract
out from under an obstacle it just detected, clearing the latch mid-stop and
re-accelerating into the same obstacle. The range is therefore frozen at the
value computed at the moment of latch and only re-evaluated after release.

Frame
-----
`/velodyne_points_filtered` is published in the velodyne sensor frame, so the
node applies the MORAI extrinsics to convert into base_link before judging
(`cloud_in_base_link: false`). Getting this wrong biases every distance by the
3.854 m mounting offset in the unsafe direction, so the node also self-checks
the incoming cloud at startup and logs an error on a mismatch.
"""

import math
import threading

import numpy as np
import rospy
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs import point_cloud2
from std_msgs.msg import Bool, Float32, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from morai_msgs.msg import EgoVehicleStatus
    HAS_MORAI = True
except ImportError:
    HAS_MORAI = False

KPH_TO_MPS = 1.0 / 3.6
SAFE_TTC = 999.9


class CollisionEStop(object):

    def __init__(self):
        # --- frame ---------------------------------------------------------
        self.cloud_in_base_link = rospy.get_param("~cloud_in_base_link", False)
        self.lidar_x = rospy.get_param("~lidar_offset_x", 3.854)
        self.lidar_z = rospy.get_param("~lidar_offset_z", 0.220)

        # --- corridor geometry ----------------------------------------------
        self.ego_width = rospy.get_param("~ego_width", 1.892)      # Ioniq5
        self.lateral_margin = rospy.get_param("~lateral_margin", 0.30)
        self.front_bumper = rospy.get_param("~front_bumper_x", 3.845)
        self.ground_z = rospy.get_param("~ground_z", 0.15)
        self.ceiling_z = rospy.get_param("~ceiling_z", 2.50)
        self.min_points = rospy.get_param("~min_points", 5)

        # --- watch range: speed-scaled, floored, latched during E-STOP --------
        self.range_min_floor = rospy.get_param("~range_min_floor", 15.0)
        self.range_base = rospy.get_param("~range_base", 10.0)
        self.range_per_speed = rospy.get_param("~range_per_speed", 2.0)
        self.range_max = rospy.get_param("~range_max", 50.0)
        self.latched_range = None

        # --- local route corridor source --------------------------------------
        self.route_enable = rospy.get_param("~route_enable", True)
        self.route_timeout = rospy.get_param("~route_timeout", 0.5)
        self.route_min_points = rospy.get_param("~route_min_points", 4)
        self.route_max_gap = rospy.get_param("~route_max_gap", 5.0)
        self.route_msg = None
        self.route_time = None
        self.route_xy = None            # cached (N,2) resampled polyline

        # --- thresholds -------------------------------------------------------
        self.ttc_estop = rospy.get_param("~ttc_estop", 1.7)
        self.ttc_release = rospy.get_param("~ttc_release", 2.0)
        self.confirm_frames = rospy.get_param("~confirm_frames", 2)
        self.release_hold_sec = rospy.get_param("~release_hold_sec", 1.0)

        self.a_max = rospy.get_param("~a_max", 6.5)                # conservative
        self.brake_margin = rospy.get_param("~brake_margin", 2.5)
        self.release_margin = rospy.get_param("~release_margin", 3.0)
        self.brake_cond_min_speed = rospy.get_param("~brake_cond_min_speed", 2.0)
        self.brake_cond_ttc_gate = rospy.get_param("~brake_cond_ttc_gate", 2.0)

        self.min_rel_speed = rospy.get_param("~min_rel_speed", 0.1)
        self.lpf_alpha = rospy.get_param("~lpf_alpha", 0.3)
        self.stale_lidar_sec = rospy.get_param("~stale_lidar_sec", 0.6)
        self.stale_speed_sec = rospy.get_param("~stale_speed_sec", 0.5)
        self.reverse_speed_threshold = rospy.get_param("~reverse_speed_threshold", 0.5)
        self.publish_hz = rospy.get_param("~publish_hz", 20.0)
        self.publish_markers = rospy.get_param("~publish_markers", True)

        # --- state -------------------------------------------------------------
        self.lock = threading.Lock()
        self.ego_v = 0.0
        self.yaw_rate = 0.0
        self.yaw_rate_from_status = False
        self.last_speed_time = None
        self.last_lidar_time = None

        self.d_obs = float("inf")
        self.prev_d_obs = None
        self.prev_d_time = None
        self.closing_speed = 0.0
        self.ttc = SAFE_TTC
        self.corridor_source = "arc"

        self.danger_count = 0
        self.safe_since = None
        self.estop = False
        self.reason = "INIT"

        self.diag_done = False
        self.diag_frames = 0
        self.diag_xmin = []
        self.diag_zmin = []

        # --- ROS I/O -------------------------------------------------------------
        self.pub_trigger = rospy.Publisher("/safety/estop_trigger", Bool, queue_size=1)
        self.pub_reason = rospy.Publisher("/safety/estop_reason", String, queue_size=1)
        self.pub_ttc = rospy.Publisher("/safety/ttc", Float32, queue_size=1)
        self.pub_dobs = rospy.Publisher("/safety/d_obs", Float32, queue_size=1)
        self.pub_range = rospy.Publisher("/safety/watch_range", Float32, queue_size=1)
        if self.publish_markers:
            self.pub_marker = rospy.Publisher("/safety/corridor", MarkerArray, queue_size=1)

        rospy.Subscriber(rospy.get_param("~lidar_topic", "/velodyne_points_filtered"),
                         PointCloud2, self.lidar_cb, queue_size=1, buff_size=2 ** 24)

        speed_topic = rospy.get_param("~vehicle_status_topic", "/morai/ego_vehicle_status")
        if HAS_MORAI:
            rospy.Subscriber(speed_topic, EgoVehicleStatus, self.status_cb, queue_size=1)
        else:
            rospy.logwarn("morai_msgs missing -- falling back to /safety/ego_speed_mps")
            rospy.Subscriber("/safety/ego_speed_mps", Float32,
                             self.speed_fallback_cb, queue_size=1)

        # EgoVehicleStatus may carry no heading_rate. The IMU is subscribed
        # unconditionally and used whenever the status message has not
        # supplied a yaw rate; the arc fallback needs it to curve at all.
        rospy.Subscriber(rospy.get_param("~imu_topic", "/imu"), Imu,
                         self.imu_cb, queue_size=1)

        if self.route_enable:
            rospy.Subscriber(rospy.get_param("~local_route_topic", "/local_route"),
                             Path, self.route_cb, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.publish_hz), self.update)
        rospy.loginfo("collision_estop up | frame=%s ttc=%.2fs a_max=%.2f route=%s",
                      "base_link" if self.cloud_in_base_link else "velodyne",
                      self.ttc_estop, self.a_max, self.route_enable)

    # --------------------------------------------------------------------- I/O

    def status_cb(self, msg):
        """MORAI EgoVehicleStatus reports velocity in km/h."""
        with self.lock:
            self.ego_v = float(msg.velocity.x) * KPH_TO_MPS
            self.last_speed_time = rospy.Time.now()
            raw_w = getattr(msg, "heading_rate", None)
            if raw_w is not None:
                self.yaw_rate_from_status = True
                self.yaw_rate = (self.lpf_alpha * float(raw_w)
                                 + (1.0 - self.lpf_alpha) * self.yaw_rate)

    def imu_cb(self, msg):
        with self.lock:
            if self.yaw_rate_from_status:
                return
            self.yaw_rate = (self.lpf_alpha * float(msg.angular_velocity.z)
                             + (1.0 - self.lpf_alpha) * self.yaw_rate)

    def speed_fallback_cb(self, msg):
        with self.lock:
            self.ego_v = float(msg.data)
            self.last_speed_time = rospy.Time.now()

    def route_cb(self, msg):
        with self.lock:
            self.route_msg = msg
            self.route_time = rospy.Time.now()
        self.route_xy = self.validate_route(msg)

    def lidar_cb(self, msg):
        stamp = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        pts = np.asarray(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=np.float32)

        if not self.diag_done:
            self.diagnose(pts, msg.header.frame_id)

        with self.lock:
            v, w = self.ego_v, self.yaw_rate
            route_ok, route_xy = self.route_ready()
            watch_range = self.current_range(v)

        p = self.to_base_link(pts)
        if route_ok:
            d = self.nearest_along_route(p, route_xy, watch_range)
            source = "route"
        else:
            d = self.nearest_in_corridor(p, v, w, watch_range)
            source = "arc"

        with self.lock:
            t = stamp.to_sec()
            if (self.prev_d_time is not None and math.isfinite(d)
                    and self.prev_d_obs is not None and math.isfinite(self.prev_d_obs)):
                dt = t - self.prev_d_time
                if dt > 1e-3:
                    raw = (self.prev_d_obs - d) / dt
                    self.closing_speed = (self.lpf_alpha * raw
                                          + (1.0 - self.lpf_alpha) * self.closing_speed)
            elif not math.isfinite(d):
                self.closing_speed = 0.0
            self.prev_d_obs, self.prev_d_time = d, t
            self.d_obs = d
            self.corridor_source = source
            self.last_lidar_time = rospy.Time.now()

    # ------------------------------------------------------------- diagnostics

    def diagnose(self, points, frame_id, need=15):
        """Check the frame assumption against the observed cloud geometry.

        Ground height is only a reliable signal when the ground is present in
        the cloud. If an upstream filter has already removed it, fall back to
        where forward returns start: the vehicle footprint is cropped in
        base_link, so returns begin beyond the bumper; in the velodyne frame
        they begin at the sensor.
        """
        if points.size == 0:
            return
        p = points.reshape(-1, 3)
        fwd = p[p[:, 0] > 0.1]
        if fwd.shape[0] < 50:
            return

        self.diag_frames += 1
        self.diag_xmin.append(float(np.percentile(fwd[:, 0], 1)))
        self.diag_zmin.append(float(np.percentile(p[:, 2], 5)))
        if self.diag_frames < need:
            return

        self.diag_done = True
        x_near = float(np.median(self.diag_xmin))
        z_floor = float(np.median(self.diag_zmin))

        rospy.loginfo("[diag] frame_id=%s nearest_forward_x=%.2f m floor_z=%.2f m",
                      frame_id, x_near, z_floor)

        ground_visible = z_floor < self.ground_z
        if ground_visible:
            looks_base_link = z_floor > -0.12
        else:
            looks_base_link = x_near > 0.5 * self.front_bumper
            rospy.loginfo("[diag] ground not visible in cloud, using nearest-return "
                          "distance (%.2f m) as the frame signal instead", x_near)

        if self.cloud_in_base_link and not looks_base_link:
            rospy.logerr("[diag] cloud_in_base_link=true but the cloud looks like the "
                         "velodyne frame -- distances are overestimated by %.2f m. "
                         "FIX THE PARAMETER.", self.lidar_x)
        elif not self.cloud_in_base_link and looks_base_link:
            rospy.logerr("[diag] cloud_in_base_link=false but the cloud already looks "
                         "like base_link. FIX THE PARAMETER.")
        else:
            rospy.loginfo("[diag] frame assumption consistent with the data")

        if not ground_visible:
            rospy.logwarn("[diag] no returns below ground_z=%.2f -- ground removal is "
                          "already applied upstream, the z filter is a no-op",
                          self.ground_z)

        with self.lock:
            from_status = self.yaw_rate_from_status
        rospy.loginfo("[diag] yaw rate source: %s",
                      "EgoVehicleStatus.heading_rate" if from_status else "IMU angular_velocity.z")

    # ------------------------------------------------------------------ geometry

    def to_base_link(self, points):
        if points.size == 0:
            return points.reshape(-1, 3)
        p = points.reshape(-1, 3)
        if not self.cloud_in_base_link:
            p = p + np.array([self.lidar_x, 0.0, self.lidar_z], dtype=np.float32)
        return p

    def half_width(self):
        return 0.5 * self.ego_width + self.lateral_margin

    def current_range(self, v):
        """Speed-scaled watch range, floored, and frozen while E-STOP is latched.

        Freezing at latch time is the actual fix for the failure this exists
        to prevent: braking lowers v, and a naive v-based range would then
        shrink the corridor out from under the very obstacle that triggered
        it, clearing the latch mid-stop. The floor is a second, independent
        line of defence for cases the freeze does not cover (e.g. an obstacle
        detected right at the edge of a still-growing corridor).
        """
        raw = min(self.range_max, max(self.range_min_floor,
                                      self.range_base + self.range_per_speed * v))
        if self.estop:
            if self.latched_range is None:
                self.latched_range = raw
            return self.latched_range
        self.latched_range = None
        return raw

    # ------------------------------------------------------- local route corridor

    def validate_route(self, msg):
        """Resample /local_route into a base_link polyline, or None if unusable.

        Any of a short route, a large gap between consecutive points, or a
        non-finite pose is treated as an invalid route so the caller falls
        back to the arc corridor rather than judging against a corrupted path.
        """
        pts = msg.poses
        if len(pts) < self.route_min_points:
            return None
        xy = np.array([(p.pose.position.x, p.pose.position.y) for p in pts],
                      dtype=np.float64)
        if not np.all(np.isfinite(xy)):
            return None
        seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
        if seg.size and np.max(seg) > self.route_max_gap:
            return None
        return xy

    def route_ready(self):
        """True plus the cached polyline if /local_route is fresh and valid."""
        if not self.route_enable or self.route_xy is None or self.route_time is None:
            return False, None
        age = (rospy.Time.now() - self.route_time).to_sec()
        if age > self.route_timeout:
            return False, None
        return True, self.route_xy

    def nearest_along_route(self, p, route_xy, watch_range):
        """Closest LiDAR return within half_width of the route polyline.

        Distance is arc length along the route to the point's projection, so
        it is directly comparable to the arc-corridor result and to speed *
        time in the TTC and braking-distance tests.
        """
        if p.size == 0 or route_xy.shape[0] < 2:
            return float("inf")

        half_w = self.half_width()
        keep = ((p[:, 2] > self.ground_z) & (p[:, 2] < self.ceiling_z) &
                (p[:, 0] > -1.0) & (np.hypot(p[:, 0], p[:, 1]) < watch_range + 5.0))
        q = p[keep]
        if q.shape[0] < self.min_points:
            return float("inf")

        seg_start = route_xy[:-1]
        seg_vec = route_xy[1:] - seg_start
        seg_len2 = np.sum(seg_vec ** 2, axis=1)
        seg_len2[seg_len2 < 1e-9] = 1e-9
        cum_len = np.concatenate([[0.0], np.cumsum(np.sqrt(seg_len2))])

        best_reach = float("inf")
        # Vectorised over points, looped over route segments: the route is a
        # few dozen points at most, points are already reduced to the watch
        # box above, so this stays well within the 20 Hz budget.
        for i in range(seg_start.shape[0]):
            rel = q[:, :2] - seg_start[i]
            t = np.clip((rel[:, 0] * seg_vec[i, 0] + rel[:, 1] * seg_vec[i, 1])
                        / seg_len2[i], 0.0, 1.0)
            proj = seg_start[i] + t[:, None] * seg_vec[i]
            lateral = np.hypot(q[:, 0] - proj[:, 0], q[:, 1] - proj[:, 1])
            forward = t >= 0.0
            inside = (lateral < half_w) & forward
            if not np.any(inside):
                continue
            reach = cum_len[i] + t[inside] * np.sqrt(seg_len2[i])
            reach = reach[reach >= 0.0]
            if reach.size:
                best_reach = min(best_reach, float(np.min(reach)))

        if not math.isfinite(best_reach):
            return float("inf")
        return max(0.0, best_reach - self.front_bumper)

    # ---------------------------------------------------------------- arc corridor

    def nearest_in_corridor(self, p, v, yaw_rate, watch_range):
        """Along-path distance from the front bumper to the closest obstacle,
        using an arc derived from yaw rate. Fallback when the route is stale,
        short, or non-finite.
        """
        if p.size == 0:
            return float("inf")

        half_w = self.half_width()
        keep = ((p[:, 2] > self.ground_z) & (p[:, 2] < self.ceiling_z) &
                (p[:, 0] > 0.0) & (p[:, 0] < watch_range))
        p = p[keep]
        if p.shape[0] < self.min_points:
            return float("inf")

        if abs(yaw_rate) < 0.01 or abs(v) < 0.5:
            inside = np.abs(p[:, 1]) < half_w
            if np.count_nonzero(inside) < self.min_points:
                return float("inf")
            reach = float(np.min(p[inside, 0]))
        else:
            radius = v / yaw_rate
            gap = np.abs(np.hypot(p[:, 0], p[:, 1] - radius) - abs(radius))
            inside = gap < half_w
            if np.count_nonzero(inside) < self.min_points:
                return float("inf")
            q = p[inside]
            theta = np.arctan2(q[:, 0], np.sign(radius) * (radius - q[:, 1]))
            theta = np.where(theta < 0.0, theta + 2.0 * math.pi, theta)
            reach = float(np.min(abs(radius) * theta))

        return max(0.0, reach - self.front_bumper)

    # ----------------------------------------------------------------- decision

    def braking_distance(self, v):
        return (v * v) / (2.0 * self.a_max) + self.brake_margin

    def update(self, _event):
        now = rospy.Time.now()
        with self.lock:
            d, v = self.d_obs, self.ego_v
            closing = self.closing_speed
            last_lidar, last_speed = self.last_lidar_time, self.last_speed_time

        # No data means no judgement is possible. Braking on silence is the
        # safe reading: the alternative is trusting a frozen d_obs forever.
        if last_lidar is None or (now - last_lidar).to_sec() > self.stale_lidar_sec:
            self.latch("LIDAR_STALE")
            return self.emit(SAFE_TTC, d)
        if last_speed is None or (now - last_speed).to_sec() > self.stale_speed_sec:
            self.latch("SPEED_STALE")
            return self.emit(SAFE_TTC, d)

        # Reverse manoeuvres make the forward corridor meaningless, but a car
        # rolling back a few centimetres after a hard brake is not a reverse
        # manoeuvre -- and if an E-STOP is active the obstacle that caused it
        # is still there, so an active latch is never cleared this way.
        if v < -self.reverse_speed_threshold and not self.estop:
            self.clear("REVERSE_BYPASS")
            return self.emit(SAFE_TTC, d)

        rel_v = max(closing, v)
        ttc = d / rel_v if (rel_v > self.min_rel_speed and math.isfinite(d)) else SAFE_TTC
        d_stop = self.braking_distance(max(v, 0.0))

        cond_ttc = ttc < self.ttc_estop
        # Braking-distance test. Gated twice so it stays a last-resort check
        # rather than a cruising-distance alarm: below walking pace the margin
        # term alone would latch permanently, and without the TTC gate a slowly
        # closing lead vehicle inside d_stop would trip it during normal
        # following. It fires only when the gap is physically insufficient AND
        # the closure is already fast enough to matter.
        cond_brake = (math.isfinite(d)
                      and v > self.brake_cond_min_speed
                      and ttc < self.brake_cond_ttc_gate
                      and d < d_stop)

        if cond_ttc or cond_brake:
            self.danger_count += 1
            self.safe_since = None
            if self.danger_count >= self.confirm_frames:
                self.latch("TTC_%.2fs" % ttc if cond_ttc else "BRAKE_DIST_%.1fm" % d)
        else:
            self.danger_count = 0
            released = (ttc > self.ttc_release and
                        (not math.isfinite(d) or d > d_stop + self.release_margin))
            if self.estop and released:
                if self.safe_since is None:
                    self.safe_since = now
                elif (now - self.safe_since).to_sec() >= self.release_hold_sec:
                    self.clear("RELEASED")
            elif not self.estop:
                self.reason = "SAFE"
            else:
                self.safe_since = None

        self.ttc = ttc
        self.emit(ttc, d)
        if self.publish_markers:
            self.draw_corridor(v)

    def latch(self, reason):
        if not self.estop:
            rospy.logwarn("E-STOP engaged: %s", reason)
        self.estop, self.reason, self.safe_since = True, reason, None

    def clear(self, reason):
        if self.estop:
            rospy.loginfo("E-STOP cleared: %s", reason)
        self.estop, self.reason = False, reason
        self.danger_count, self.safe_since = 0, None
        self.latched_range = None

    def emit(self, ttc, d):
        with self.lock:
            rng = self.latched_range if self.estop else self.current_range(self.ego_v)
            src = self.corridor_source
        self.pub_trigger.publish(Bool(data=self.estop))
        self.pub_reason.publish(String(data="%s|%s" % (self.reason, src)))
        self.pub_ttc.publish(Float32(data=min(ttc, SAFE_TTC)))
        self.pub_dobs.publish(Float32(data=d if math.isfinite(d) else SAFE_TTC))
        self.pub_range.publish(Float32(data=rng))

    # ------------------------------------------------------------ visualisation

    def draw_corridor(self, v):
        with self.lock:
            w = self.yaw_rate
            rng = self.latched_range if self.estop else self.current_range(v)
            source = self.corridor_source
            route_xy = self.route_xy if source == "route" else None

        m = Marker()
        m.header.frame_id = "base_link"
        m.header.stamp = rospy.Time.now()
        m.ns, m.id = "safety_corridor", 0
        m.type, m.action = Marker.LINE_STRIP, Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = self.ego_width + 2.0 * self.lateral_margin
        m.color.a = 0.3
        if self.estop:
            m.color.r = 1.0
        else:
            m.color.g, m.color.b = 0.5, 1.0
        m.lifetime = rospy.Duration(0.3)

        if source == "route" and route_xy is not None and route_xy.shape[0] >= 2:
            cum = 0.0
            m.points.append(self._pt(route_xy[0, 0], route_xy[0, 1]))
            for i in range(1, route_xy.shape[0]):
                seg = math.hypot(route_xy[i, 0] - route_xy[i - 1, 0],
                                 route_xy[i, 1] - route_xy[i - 1, 1])
                cum += seg
                if rng == None:
                    tmp_rng = 0
                else:
                    tmp_rng = rng
                if cum > tmp_rng:
                    break
                m.points.append(self._pt(route_xy[i, 0], route_xy[i, 1]))
        else:
            speed = max(abs(v), 1.0)
            for i in range(21):
                s = self.front_bumper + (rng - self.front_bumper) * i / 20.0
                if abs(w) < 0.01:
                    m.points.append(self._pt(s, 0.0))
                else:
                    radius = speed / w
                    theta = s / abs(radius)
                    m.points.append(self._pt(abs(radius) * math.sin(theta),
                                             radius * (1.0 - math.cos(theta))))

        arr = MarkerArray()
        arr.markers.append(m)
        self.pub_marker.publish(arr)

    @staticmethod
    def _pt(x, y):
        pt = Marker().pose.position.__class__()
        pt.x, pt.y, pt.z = x, y, 0.0
        return pt


if __name__ == "__main__":
    rospy.init_node("collision_estop")
    CollisionEStop()
    rospy.spin()

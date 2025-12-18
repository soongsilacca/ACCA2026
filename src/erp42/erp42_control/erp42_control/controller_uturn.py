# ros2
import rclpy
from rclpy.qos import qos_profile_system_default

# msg
from geometry_msgs.msg import PoseArray, PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
from erp42_msgs.msg import ControlMessage

# tf
from tf_transformations import *

# stanley
from stanley import Stanley

# DB
try:
    from DB import DB
except Exception as e:
    print(e)

# utils
import time
from enum import Enum
import numpy as np
import math as m


class SpeedSupporter:
    def __init__(self, node):
        self.he_gain = node.declare_parameter("/speed_supporter/he_gain_uturn", 50.0).value
        self.ce_gain = node.declare_parameter("/speed_supporter/ce_gain_uturn", 30.0).value
        self.he_thr  = node.declare_parameter("/speed_supporter/he_thr_uturn", 0.001).value
        self.ce_thr  = node.declare_parameter("/speed_supporter/ce_thr_uturn", 0.002).value

    def func(self, x, a, b):
        return a * (x - b)

    def adaptSpeed(self, value, hdr, ctr, min_value, max_value):
        hdr = self.func(abs(hdr), -self.he_gain, self.he_thr)
        ctr = self.func(abs(ctr), -self.ce_gain, self.ce_thr)
        err = hdr + ctr
        return np.clip(value + err, min_value, max_value)


class PID:
    def __init__(self, node):
        self.node = node
        self.p_gain = node.declare_parameter("/stanley_controller/p_gain_uturn", 2.07).value
        self.i_gain = node.declare_parameter("/stanley_controller/i_gain_uturn", 0.85).value
        self.p_err = 0.0; self.i_err = 0.0; self.speed = 0.0
        now = node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1]/1e9
        self.last    = self.current

    def PIDControl(self, speed, desired_value):
        now = self.node.get_clock().now().seconds_nanoseconds()
        self.current = now[0] + now[1]/1e9
        dt = self.current - self.last
        self.last = self.current
        err = desired_value - speed
        self.p_err = err
        self.i_err += self.p_err * dt * (0.0 if speed == 0 else 1.0)
        self.speed = speed + (self.p_gain*self.p_err) + (self.i_gain*self.i_err)
        return int(np.clip(self.speed, 8, 12))


class Uturn_state(Enum):
    In = 0
    Turn = 1
    Out = 2


class Uturn:
    def __init__(self, node):
        self.node = node

        # paths
        self.in_path  = DB("U-turn_IN.db").read_db_n("Path","x","y","yaw")
        self.turn_db  = DB("U-turn.db")
        self.turn_path_template = self.turn_db.read_db_n("Path","x","y","yaw")
        self.turn_path = list(self.turn_path_template)

        # modules
        self.st  = Stanley()
        self.pid = PID(node)
        self.ss  = SpeedSupporter(node)

        self.cones = []
        self.odometry = None
        self.state = Uturn_state.In

        # io
        self.node.create_subscription(PoseArray, "/cone_pose_map", self.cone_callback, 10)
        self.pub_path = node.create_publisher(Path, "uturn_path", qos_profile_system_default)

        # trigger
        self.search_started = False
        self.search_radius  = node.declare_parameter("/uturn/search_radius", 3.0).value
        self._last_min_cone_dist = float("inf")

        # debug markers
        self.pub_dbg = node.create_publisher(MarkerArray, "uturn_debug", 1)
        self._dbg_ns = "uturn_debug"
        self._DBG_ID_RADIUS = 0
        self._DBG_ID_POSE   = 1
        self._DBG_ID_TEXT   = 2
        self._DBG_ID_CONE_BASE = 100

    def cone_callback(self, msg):
        self.cones = []
        for pose in msg.poses:
            p = Point(); p.x=pose.position.x; p.y=pose.position.y; p.z=pose.position.z
            self.cones.append(p)
        self.node.get_logger().debug(f"[UTURN] cones={len(self.cones)}")

    def _translated_turn_path(self, dx, dy):
        return [(p[0]+dx, p[1]+dy, p[2]) for p in self.turn_path_template]

    def has_cone_within_radius(self, radius: float = None) -> bool:
        if self.odometry is None or len(self.cones)==0:
            return False
        r = self.search_radius if radius is None else radius
        ox, oy = float(self.odometry.x), float(self.odometry.y)
        dists = [np.hypot(float(c.x)-ox, float(c.y)-oy) for c in self.cones]
        self._last_min_cone_dist = float(min(dists)) if dists else float("inf")
        return self._last_min_cone_dist <= r + 1e-6

    def _mk_common(self, mid: int, mtype: int):
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.ns = self._dbg_ns; m.id = mid; m.type = mtype; m.action = Marker.ADD
        m.lifetime.sec = 0; m.frame_locked = False
        return m

    def publish_debug_markers(self):
        from visualization_msgs.msg import Marker, MarkerArray
        ma = MarkerArray()
        ox = float(self.odometry.x) if self.odometry else 0.0
        oy = float(self.odometry.y) if self.odometry else 0.0

        mr = self._mk_common(self._DBG_ID_RADIUS, Marker.CYLINDER)
        mr.pose.position.x=ox; mr.pose.position.y=oy; mr.pose.position.z=0.05
        mr.scale.x=self.search_radius*2.0; mr.scale.y=self.search_radius*2.0; mr.scale.z=0.05
        if self.search_started: mr.color.r, mr.color.g, mr.color.b, mr.color.a = 0.0,0.8,0.8,0.35
        else:                   mr.color.r, mr.color.g, mr.color.b, mr.color.a = 0.1,0.9,0.3,0.35
        ma.markers.append(mr)

        mp = self._mk_common(self._DBG_ID_POSE, Marker.SPHERE)
        mp.pose.position.x=ox; mp.pose.position.y=oy; mp.pose.position.z=0.2
        mp.scale.x=mp.scale.y=mp.scale.z=0.25
        mp.color.r, mp.color.g, mp.color.b, mp.color.a = 0.2,0.4,1.0,0.9
        ma.markers.append(mp)

        mt = self._mk_common(self._DBG_ID_TEXT, Marker.TEXT_VIEW_FACING)
        mt.pose.position.x=ox; mt.pose.position.y=oy; mt.pose.position.z=0.8
        mt.scale.z=0.35
        mt.color.r, mt.color.g, mt.color.b, mt.color.a = 1.0,1.0,1.0,0.95
        min_d_txt = f"{self._last_min_cone_dist:.2f}" if np.isfinite(self._last_min_cone_dist) else "inf"
        mt.text = f"state={self.state.name} | search_started={self.search_started} | r={self.search_radius:.1f}m | min_d={min_d_txt}m"
        ma.markers.append(mt)

        cid = self._DBG_ID_CONE_BASE
        for c in self.cones:
            d = np.hypot(float(c.x)-ox, float(c.y)-oy)
            mc = self._mk_common(cid, Marker.SPHERE)
            mc.pose.position.x=float(c.x); mc.pose.position.y=float(c.y); mc.pose.position.z=0.15
            mc.scale.x=mc.scale.y=mc.scale.z=0.18
            if d <= self.search_radius: mc.color.r,mc.color.g,mc.color.b,mc.color.a = 1.0,0.25,0.25,0.95
            else:                       mc.color.r,mc.color.g,mc.color.b,mc.color.a = 0.6,0.6,0.6,0.5
            ma.markers.append(mc); cid += 1

        self.pub_dbg.publish(ma)

    def on_search_start(self):
        self.node.get_logger().info("[UTURN] TRIGGERED: preparing Turn path...")
        if len(self.turn_path_template)==0 or self.odometry is None:
            self.node.get_logger().warn("[UTURN] cannot prepare turn path (no template or odom)")
            return
        p0x, p0y = self.turn_path_template[0][0], self.turn_path_template[0][1]
        dx = self.odometry.x - p0x
        dy = self.odometry.y - p0y
        self.turn_path = self._translated_turn_path(dx, dy)
        colides = self.correct_turn()
        if not colides:
            self.state = Uturn_state.Turn
            self.node.get_logger().info("[UTURN] → Turn")

    def control_uturn(self, odometry):
        self.odometry = odometry
        msg = ControlMessage()

        # 1) 트리거 체크 (반경=search_radius=5m)
        if (not self.search_started) and self.has_cone_within_radius():
            self.search_started = True
            self.on_search_start()  # 여기서 Turn으로 전환

        self.publish_debug_markers()

        if self.state == Uturn_state.In:
            path_x = [p[0] for p in self.in_path]
            path_y = [p[1] for p in self.in_path]
            path_yaw = [p[2] for p in self.in_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path_x, path_y, path_yaw, h_gain=0.5, c_gain=0.24
            )

            target_speed = 8.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=12, max_value=14)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer) * 1e3)
            msg.gear = 2

            # 2) 안전장치: search_started=True인데 아직 In이면 강제 전환
            if self.search_started and self.state == Uturn_state.In:
                self.node.get_logger().warn("[UTURN] search_started=True but still In -> forcing on_search_start()")
                self.on_search_start()

        elif self.state == Uturn_state.Turn:
            collides = self.correct_turn()
            path_x = [p[0] for p in self.turn_path]
            path_y = [p[1] for p in self.turn_path]
            path_yaw = [p[2] for p in self.turn_path]
            self.publish_path_msg(path_x, path_y, path_yaw)

            steer, target_idx, hdr, ctr = self.st.stanley_control(
                odometry, path_x, path_y, path_yaw, h_gain=0.5, c_gain=0.24
            )
            target_speed = 8.0
            adapted_speed = self.ss.adaptSpeed(target_speed, hdr, ctr, min_value=8, max_value=10)
            speed = self.pid.PIDControl(odometry.v * 3.6, adapted_speed)
            msg.speed = int(speed) * 10
            msg.steer = int(m.degrees((-1) * steer) * 1e3)
            msg.gear = 2

            if collides:
                self.node.get_logger().warn("[UTURN] collision risk -> back to In")
                self.state = Uturn_state.In
            elif target_idx >= len(path_x) - 3:
                return msg, True  # U-turn completed

        return msg, False

    def in_uturn(self):
        if len(self.turn_path_template)==0:
            return False, 0, 0
        p0x, p0y = self.turn_path_template[0][0], self.turn_path_template[0][1]
        dx = self.odometry.x - p0x
        dy = self.odometry.y - p0y
        translated = self._translated_turn_path(dx, dy)
        for px, py, _ in translated:
            for cone in self.cones:
                if np.hypot(px - cone.x, py - cone.y) < 2.0:
                    return False, 0, 0
        return True, dx, dy

    def correct_turn(self):
        if len(self.cones)==0 or len(self.turn_path)==0:
            return False
        for px, py, _ in self.turn_path:
            for cone in self.cones:
                if np.hypot(px - cone.x, py - cone.y) < 0.5:
                    return True
        return False

    def publish_path_msg(self, path_x, path_y, path_yaw):
        path_msg = Path()
        path_msg.header.stamp = self.node.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for x, y, yaw in zip(path_x, path_y, path_yaw):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            qx, qy, qz, qw = 0.0, 0.0, np.sin(yaw/2), np.cos(yaw/2)
            pose.pose.orientation.x = qx; pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz; pose.pose.orientation.w = qw
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

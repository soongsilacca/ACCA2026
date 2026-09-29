#!/usr/bin/env python3
"""6.1 후방 충돌 감시 노드.

입력
  ~tracks_topic  safety_monitor/TrackedObjectArray (base_link, rear_lidar_perception_node 출력)
  ~ego_topic     morai_msgs/EgoVehicleStatus 또는 nav_msgs/Odometry (~ego_source)
  ~route_topic   nav_msgs/Path (local route, 기본 base_link)
출력
  /safety/collision_estop  std_msgs/Bool   트랙 메시지마다 1회 발행
  /safety/collision_status std_msgs/String 판정 사유
  /safety/collision_markers visualization_msgs/MarkerArray

트랙 메시지를 받을 때마다 발행하므로 perception이 멈추면 이 노드의 발행도 멈추고,
gate가 이를 두절로 처리한다.
"""
import math
import time

import numpy as np
import rospy
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray

from safety_monitor.msg import TrackedObjectArray
from safety_monitor.adapters.tracked_object import from_msg
from safety_monitor.collision.monitor import CollisionMonitor
from safety_monitor.common.geometry import base_to_map, resample_polyline
from safety_monitor.common.params import CollisionParams
from safety_monitor.common.types import EgoState
from safety_monitor.lanes.mgeo_lanes import LaneMap


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RearCollisionNode:
    def __init__(self):
        mgeo_path = rospy.get_param("~mgeo_path")
        self.ego_source = rospy.get_param("~ego_source", "odometry")
        ego_topic = rospy.get_param("~ego_topic", "/localization/kinematic_state")
        self.ego_velocity_scale = float(rospy.get_param("~ego_velocity_scale", 1.0 / 3.6))
        self.ego_timeout = float(rospy.get_param("~ego_timeout", 0.3))
        route_topic = rospy.get_param("~route_topic", "/local_route")
        self.route_frame = rospy.get_param("~route_frame", "")   # 비우면 메시지 frame_id 사용
        self.route_timeout = float(rospy.get_param("~route_timeout", 1.0))
        tracks_topic = rospy.get_param("~tracks_topic", "/rear_lidar/tracked_objects")
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))

        params, unknown = CollisionParams.from_dict(rospy.get_param("~collision", {}))
        if unknown:
            rospy.logwarn("unknown collision params ignored: %s", unknown)
        self.params = params

        rospy.loginfo("loading MGeo lanes from %s", mgeo_path)
        self.lanemap = LaneMap.from_mgeo(mgeo_path)
        rospy.loginfo("MGeo loaded: %d links", len(self.lanemap.links))
        self.monitor = CollisionMonitor(params, self.lanemap)

        self.ego = None
        self.ego_rx = None
        self.route_map = None
        self.route_rx = None

        self.pub_estop = rospy.Publisher("/safety/collision_estop", Bool, queue_size=1)
        self.pub_status = rospy.Publisher("/safety/collision_status", String, queue_size=1)
        self.pub_markers = rospy.Publisher("/safety/collision_markers", MarkerArray, queue_size=1)

        if self.ego_source == "ego_status":
            from morai_msgs.msg import EgoVehicleStatus
            rospy.Subscriber(ego_topic, EgoVehicleStatus, self._ego_status_cb, queue_size=5)
        elif self.ego_source == "odometry":
            rospy.Subscriber(ego_topic, Odometry, self._odom_cb, queue_size=5)
        else:
            raise ValueError("~ego_source must be ego_status or odometry")
        rospy.Subscriber(route_topic, Path, self._route_cb, queue_size=1)
        rospy.Subscriber(tracks_topic, TrackedObjectArray, self._tracks_cb, queue_size=1)

    # ------------------------------------------------------------ inputs
    def _ego_status_cb(self, msg):
        speed = float(msg.velocity.x) * self.ego_velocity_scale
        self.ego = EgoState(msg.header.stamp.to_sec(), float(msg.position.x),
                            float(msg.position.y), math.radians(float(msg.yaw)), speed)
        self.ego_rx = time.monotonic()

    def _odom_cb(self, msg):
        v = msg.twist.twist.linear
        self.ego = EgoState(msg.header.stamp.to_sec(), msg.pose.pose.position.x,
                            msg.pose.pose.position.y, _yaw_from_quat(msg.pose.pose.orientation),
                            math.hypot(v.x, v.y))
        self.ego_rx = time.monotonic()

    def _current_ego(self):
        if self.ego is None or time.monotonic() - self.ego_rx > self.ego_timeout:
            return None
        return self.ego

    def _route_cb(self, msg):
        if not msg.poses:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        frame = self.route_frame or msg.header.frame_id
        if frame in ("map", "world"):
            self.route_map = pts
        else:
            ego = self._current_ego()
            if ego is None:
                return
            xs, ys = base_to_map(ego, pts[:, 0], pts[:, 1])
            self.route_map = np.column_stack([xs, ys])
        self.route_rx = time.monotonic()

    def _current_route(self):
        if self.route_map is None or time.monotonic() - self.route_rx > self.route_timeout:
            return None
        return self.route_map

    # ------------------------------------------------------------ main
    def _tracks_cb(self, msg):
        stamp = msg.header.stamp.to_sec() if not msg.header.stamp.is_zero() else rospy.get_time()
        ego = self._current_ego()
        if ego is None:
            ego = EgoState(stamp, 0.0, 0.0, 0.0, 0.0, valid=False)
        objects = [from_msg(o) for o in msg.objects]
        try:
            res = self.monitor.step(stamp, ego, self._current_route(), objects)
        except Exception as exc:  # 판정 중 예외 -> 정지
            rospy.logerr_throttle(1.0, "collision monitor exception: %s", exc)
            self.pub_estop.publish(Bool(data=True))
            self.pub_status.publish(String(data="exception"))
            return
        self.pub_estop.publish(Bool(data=res.estop))
        self.pub_status.publish(String(data="estop=%d raw=%d lc=%d reason=%s watched=%s" % (
            res.estop, res.raw, res.lane_change_active, res.reason, res.watched)))
        if self.publish_markers:
            self._publish_markers(msg, res)

    def _publish_markers(self, msg, res):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        hit = {j.track_id for j in res.judgments if j.collision} | set(res.holds)
        watched = set(res.watched) | set(res.holds)
        mid = 0
        for o in msg.objects:
            if o.track_id not in watched:
                continue
            m = Marker()
            m.header = msg.header
            m.ns = "safety_targets"
            m.id = mid
            mid += 1
            m.type = Marker.CYLINDER
            m.pose.position = o.pose.position
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 1.2
            m.scale.z = 0.3
            m.color.a = 0.9
            m.color.r = 1.0
            m.color.g = 0.0 if o.track_id in hit else 0.8
            m.lifetime = rospy.Duration(0.3)
            arr.markers.append(m)
        ego = self._current_ego()
        if self.monitor.lane.active and ego is not None:
            # map -> base_link TF가 없어도 보이도록 base_link로 변환해서 그린다
            line = Marker()
            line.header = msg.header
            line.ns = "safety_target_lane"
            line.id = mid
            line.type = Marker.LINE_STRIP
            line.pose.orientation.w = 1.0
            line.scale.x = 0.2
            line.color.a = 0.8
            line.color.b = 1.0
            line.color.g = 0.5
            line.lifetime = rospy.Duration(0.3)
            pts, _ = resample_polyline(self.monitor.lane.target.poly.pts, 2.0)
            c, s = math.cos(ego.yaw), math.sin(ego.yaw)
            dx, dy = pts[:, 0] - ego.x, pts[:, 1] - ego.y
            xb, yb = c * dx + s * dy, -s * dx + c * dy
            near = (xb > -60.0) & (xb < 80.0) & (np.abs(yb) < 30.0)
            line.points = [Point(x=float(x), y=float(y), z=0.1) for x, y in zip(xb[near], yb[near])]
            if len(line.points) >= 2:
                arr.markers.append(line)
        self.pub_markers.publish(arr)


def main():
    rospy.init_node("rear_collision_monitor")
    RearCollisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()

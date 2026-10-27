#!/usr/bin/env python3
"""6.1 후방 충돌 감시 노드.

입력
  ~tracks_topic  safety_monitor/TrackedObjectArray (base_link, rear_lidar_perception_node 출력)
  ~ego_topic     morai_msgs/EgoVehicleStatus 또는 nav_msgs/Odometry (~ego_source)
  ~route_topic   nav_msgs/Path (local route, ~base_frame 또는 ~map_frame)
                 받을 때 자차 투영점부터 ~route_length [m] (기본 10, 0 이하면 전체)만 남긴다.
좌표계
  트랙과 local route는 각 메시지 시각의 자차 pose(이력 보간)로 map(MGeo 좌표)에 옮긴다.
  frame_id가 기대값(트랙 ~base_frame, 자차 ~map_frame, route 둘 중 하나)과 다르면
  그 입력을 쓰지 않는다. 트랙 frame이 틀리면 정지를 발행한다.
출력
  /safety/collision_estop  std_msgs/Bool   트랙 메시지마다 1회 발행
  /safety/collision_status std_msgs/String 판정 사유
  /safety/collision_markers visualization_msgs/MarkerArray
  /safety/mgeo_lanes       visualization_msgs/MarkerArray  자차 주변 MGeo 차선 (base_link, ~mgeo_viz_hz)

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
from safety_monitor.common.ego_history import EgoHistory
from safety_monitor.common.geometry import base_to_map, clip_polyline_ahead, resample_polyline
from safety_monitor.common.params import CollisionParams
from safety_monitor.common.types import EgoState
from safety_monitor.lanes.mgeo_lanes import LaneMap


def _frame(frame_id):
    return frame_id.lstrip("/")


def _yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RearCollisionNode:
    def __init__(self):
        mgeo_path = rospy.get_param("~mgeo_path")
        self.ego_source = rospy.get_param("~ego_source", "odometry")
        ego_topic = rospy.get_param("~ego_topic", "/localization/kinematic_state")
        self.ego_velocity_scale = float(rospy.get_param("~ego_velocity_scale", 1.0 / 3.6))
        self.ego_timeout = float(rospy.get_param("~ego_timeout", 0.3))
        self.map_frame = _frame(rospy.get_param("~map_frame", "map"))       # MGeo 좌표
        self.base_frame = _frame(rospy.get_param("~base_frame", "base_link"))
        self.ego_hist = EgoHistory(
            keep=2.0, max_extrapolation=float(rospy.get_param("~ego_max_extrapolation", 0.2)))
        route_topic = rospy.get_param("~route_topic", "/local_route")
        self.route_frame = rospy.get_param("~route_frame", "")   # 비우면 메시지 frame_id 사용
        self.route_timeout = float(rospy.get_param("~route_timeout", 1.0))
        self.route_length = float(rospy.get_param("~route_length", 10.0))
        self.route_frame_seen = None
        tracks_topic = rospy.get_param("~tracks_topic", "/rear_lidar/tracked_objects")
        self.publish_markers = bool(rospy.get_param("~publish_markers", True))
        mgeo_viz_hz = float(rospy.get_param("~mgeo_viz_hz", 5.0))   # 0 이하면 끔
        self.mgeo_viz_radius = float(rospy.get_param("~mgeo_viz_radius", 60.0))

        params, unknown = CollisionParams.from_dict(rospy.get_param("~collision", {}))
        if unknown:
            rospy.logwarn("unknown collision params ignored: %s", unknown)
        self.params = params

        rospy.loginfo("loading MGeo lanes from %s", mgeo_path)
        self.lanemap = LaneMap.from_mgeo(mgeo_path)
        rospy.loginfo("MGeo loaded: %d links", len(self.lanemap.links))
        self.monitor = CollisionMonitor(params, self.lanemap)

        self.ego_rx = None
        self.route_map = None
        self.route_rx = None

        self.pub_estop = rospy.Publisher("/safety/collision_estop", Bool, queue_size=1)
        self.pub_status = rospy.Publisher("/safety/collision_status", String, queue_size=1)
        self.pub_markers = rospy.Publisher("/safety/collision_markers", MarkerArray, queue_size=1)
        self.pub_lanes = rospy.Publisher("/safety/mgeo_lanes", MarkerArray, queue_size=1)
        if mgeo_viz_hz > 0.0:
            self.lane_timer = rospy.Timer(rospy.Duration(1.0 / mgeo_viz_hz), self._publish_lanes)

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
    @staticmethod
    def _stamp(header):
        return header.stamp.to_sec() if not header.stamp.is_zero() else rospy.get_time()

    def _ego_status_cb(self, msg):
        # EgoVehicleStatus는 MORAI map 좌표 (frame_id 없음)
        speed = float(msg.velocity.x) * self.ego_velocity_scale
        self.ego_hist.add(EgoState(self._stamp(msg.header), float(msg.position.x),
                                   float(msg.position.y), math.radians(float(msg.yaw)), speed))
        self.ego_rx = time.monotonic()

    def _odom_cb(self, msg):
        frame, child = _frame(msg.header.frame_id), _frame(msg.child_frame_id)
        if frame != self.map_frame or (child and child != self.base_frame):
            rospy.logerr_throttle(2.0, "ego odometry frame %s -> %s, expected %s -> %s: ignored",
                                  frame, child, self.map_frame, self.base_frame)
            return
        v = msg.twist.twist.linear
        self.ego_hist.add(EgoState(self._stamp(msg.header), msg.pose.pose.position.x,
                                   msg.pose.pose.position.y,
                                   _yaw_from_quat(msg.pose.pose.orientation), math.hypot(v.x, v.y)))
        self.ego_rx = time.monotonic()

    def _current_ego(self, t=None):
        """t가 주어지면 그 시각으로 보간한 pose, 없으면 최신 pose. 끊겼으면 None."""
        if self.ego_rx is None or time.monotonic() - self.ego_rx > self.ego_timeout:
            return None
        return self.ego_hist.latest() if t is None else self.ego_hist.at(t)

    def _route_cb(self, msg):
        if not msg.poses:
            return
        pts = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        frame = _frame(self.route_frame or msg.header.frame_id)
        if frame != self.route_frame_seen:
            rospy.loginfo("local route frame: %s (route_length=%.1f m)", frame, self.route_length)
            self.route_frame_seen = frame
        if frame not in (self.map_frame, self.base_frame):
            rospy.logerr_throttle(2.0, "local route frame %s is neither %s nor %s: ignored",
                                  frame, self.map_frame, self.base_frame)
            return
        ego = self._current_ego(self._stamp(msg.header))
        if ego is None:
            return
        if frame == self.base_frame:
            xs, ys = base_to_map(ego, pts[:, 0], pts[:, 1])
            pts = np.column_stack([xs, ys])
        if self.route_length > 0.0:
            pts = clip_polyline_ahead(pts, ego.x, ego.y, self.route_length)
        if len(pts) < 2:
            return
        self.route_map = pts
        self.route_rx = time.monotonic()

    def _current_route(self):
        if self.route_map is None or time.monotonic() - self.route_rx > self.route_timeout:
            return None
        return self.route_map

    # ------------------------------------------------------------ main
    def _tracks_cb(self, msg):
        stamp = self._stamp(msg.header)
        frame = _frame(msg.header.frame_id)
        if frame != self.base_frame:
            rospy.logerr_throttle(2.0, "tracked objects frame %s, expected %s: stop",
                                  frame, self.base_frame)
            self.pub_estop.publish(Bool(data=True))
            self.pub_status.publish(String(data="tracks_frame:%s" % frame))
            return
        ego = self._current_ego(stamp)
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
            self._publish_markers(msg, res, ego)

    def _publish_markers(self, msg, res, ego):
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
        if self.monitor.lane.active and ego.valid:
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

    def _publish_lanes(self, _evt):
        """판정에 쓰는 것과 같은 자차 상태로 주변 MGeo 차선을 base_link에 그린다.
        회색: 일반 차선, 주황: 교차로 연결 link, 초록: 자차 차선(chain), 하늘색: MGeo 옆 차선."""
        ego = self._current_ego()
        if ego is None:
            return
        p = self.params
        lm = self.lanemap
        r = self.mgeo_viz_radius
        c, s = math.cos(ego.yaw), math.sin(ego.yaw)

        def to_base(pts):
            dx, dy = pts[:, 0] - ego.x, pts[:, 1] - ego.y
            xb, yb = c * dx + s * dy, -s * dx + c * dy
            return xb, yb, np.hypot(xb, yb) < r

        def seg_points(link_pts):
            xb, yb, ok = to_base(link_pts)
            both = ok[:-1] & ok[1:]
            out = []
            for i in np.where(both)[0]:
                out.append(Point(x=float(xb[i]), y=float(yb[i]), z=0.05))
                out.append(Point(x=float(xb[i + 1]), y=float(yb[i + 1]), z=0.05))
            return out

        def marker(ns, mid, mtype, width, rgba):
            m = Marker()
            m.header.frame_id = "base_link"
            m.header.stamp = rospy.Time.now()
            m.ns = ns
            m.id = mid
            m.type = mtype
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m

        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        ego_link = lm.match(ego.x, ego.y, ego.yaw, p.lane_match_max_dist,
                            p.lane_match_max_heading_deg)
        normal = marker("mgeo_lanes", 0, Marker.LINE_LIST, 0.08, (0.6, 0.6, 0.6, 0.7))
        junction = marker("mgeo_junction_lanes", 1, Marker.LINE_LIST, 0.08, (1.0, 0.55, 0.0, 0.7))
        for lid in lm.links_near(ego.x, ego.y, r):
            ln = lm.links[lid]
            (junction if ln.link_type == "1" else normal).points.extend(seg_points(ln.points))
        for m in (normal, junction):
            if m.points:
                arr.markers.append(m)

        info = marker("mgeo_ego_info", 4, Marker.TEXT_VIEW_FACING, 0.0, (1.0, 1.0, 1.0, 0.9))
        info.pose.position.z = 2.5
        info.scale.z = 0.6
        if ego_link is None:
            info.text = "MGeo: NO_MATCH"
        else:
            ln = lm.links[ego_link]
            chain = lm.chain(ego_link, p.chain_back, p.chain_fwd)
            _, d, _ = chain.poly.project_one(ego.x, ego.y)
            info.text = "MGeo %s type=%s d=%+.2f m L=%s R=%s" % (
                ego_link, ln.link_type, d, ln.left_dst or "-", ln.right_dst or "-")
            lane = marker("mgeo_ego_lane", 2, Marker.LINE_STRIP, 0.25, (0.1, 0.9, 0.2, 0.9))
            xb, yb, ok = to_base(chain.poly.pts)
            lane.points = [Point(x=float(x), y=float(y), z=0.08) for x, y in zip(xb[ok], yb[ok])]
            if len(lane.points) >= 2:
                arr.markers.append(lane)
            adj = marker("mgeo_adjacent_lanes", 3, Marker.LINE_LIST, 0.18, (0.0, 0.8, 1.0, 0.9))
            for dst in (ln.left_dst, ln.right_dst):
                if dst in lm.links:
                    adj.points.extend(seg_points(lm.links[dst].points))
            if adj.points:
                arr.markers.append(adj)
        arr.markers.append(info)
        self.pub_lanes.publish(arr)


def main():
    rospy.init_node("rear_collision_monitor")
    RearCollisionNode()
    rospy.spin()


if __name__ == "__main__":
    main()

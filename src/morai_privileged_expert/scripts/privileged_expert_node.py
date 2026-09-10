#!/usr/bin/env python3
import json
import math
import os
import sys
import threading

# catkin_install_python creates an executable relay beside pdm_lite_core.py.
# Prefer this source directory so the relay cannot shadow the actual module.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rospy
from geometry_msgs.msg import Point, PoseStamped
from morai_msgs.msg import ObjectStatusList, SetTrafficLight
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from pdm_lite_core import (evaluate_proposal, idm_target_speed,
                           proposal_cost, smooth_shift)


class PrivilegedExpert:
    ACTIONS = ("CRUISE", "FOLLOW", "STOP", "AVOID", "MISSION_COMPLETE", "EMERGENCY_STOP")

    def __init__(self):
        rospy.init_node("morai_privileged_expert")
        self.hz = float(rospy.get_param("~publish_hz", 20.0))
        self.horizon = float(rospy.get_param("~planning_horizon_m", 45.0))
        self.spacing = float(rospy.get_param("~path_spacing_m", 0.5))
        self.offsets = list(map(float, rospy.get_param("~proposal_offsets_m", [-2.5, -1.5, 0.0, 1.5, 2.5])))
        self.speed_factors = list(map(float, rospy.get_param("~proposal_speed_factors", [0.0, .35, .65, 1.0])))
        self.ego_length = float(rospy.get_param("~ego_length_m", 4.8))
        self.ego_width = float(rospy.get_param("~ego_width_m", 1.9))
        self.margin = float(rospy.get_param("~safety_margin_m", .8))
        self.predict_sec = float(rospy.get_param("~prediction_horizon_sec", 7.0))
        self.emergency_ttc = float(rospy.get_param("~emergency_ttc_sec", 1.0))
        self.stop_distance = float(rospy.get_param("~stop_distance_m", 4.0))
        self.follow_distance = float(rospy.get_param("~follow_distance_m", 18.0))
        self.interaction_clearance = float(rospy.get_param("~interaction_clearance_m", 1.5))
        self.avoid_distance = float(rospy.get_param("~avoid_distance_m", 30.0))
        self.pedestrian_caution_clearance = float(
            rospy.get_param("~pedestrian_caution_clearance_m", 3.0))
        self.pedestrian_caution_time = float(
            rospy.get_param("~pedestrian_caution_time_sec", 3.5))
        self.pedestrian_width_padding = float(
            rospy.get_param("~pedestrian_width_padding_m", 1.2))
        self.npc_overtake_distance = float(
            rospy.get_param("~npc_overtake_distance_m", 25.0))
        self.npc_overtake_max_speed = float(
            rospy.get_param("~npc_overtake_max_speed_mps", 5.0))
        self.max_offset = float(rospy.get_param("~max_lateral_offset_m", 2.5))
        self.input_timeout = float(rospy.get_param("~input_timeout_sec", 1.0))
        self.object_frame = rospy.get_param("~object_frame", "map")
        self.require_objects = bool(rospy.get_param("~require_objects", False))
        self.min_cruise_speed = float(rospy.get_param("~min_cruise_speed_mps", 2.0))
        self.maximum_speed = float(rospy.get_param("~maximum_speed_mps", 27.78))
        self.lead_vehicle_distance = float(rospy.get_param("~lead_vehicle_distance_m", 100.0))
        self.lane_change_length = float(rospy.get_param("~lane_change_length_m", 12.0))
        self.signal_stop_buffer = float(rospy.get_param("~signal_stop_buffer_m", 13.0))
        self.signal_full_stop_lead = float(rospy.get_param("~signal_full_stop_lead_m", 5.0))
        self.signal_slowdown_distance = float(rospy.get_param("~signal_slowdown_distance_m", 35.0))
        self.signal_path_tolerance = float(rospy.get_param("~signal_path_tolerance_m", 15.0))
        self.traffic_state_timeout = float(rospy.get_param("~traffic_state_timeout_sec", 1.5))
        self.speed_ratio = float(rospy.get_param("~target_speed_limit_ratio", 1.5))
        self.weights = rospy.get_param("~weights", {})
        self.lock = threading.Lock(); self.odom = None
        self.route = None; self.global_path = None; self.objects = None; self.road_speed = 0.0
        self.last_behavior = "STOP"; self.last_plan = None
        self.odom_stamp = rospy.Time(0); self.object_stamp = rospy.Time(0)
        self.traffic_states = {}
        self.object_tracks = {}
        self.active_traffic_id = None
        self.traffic_state_stamp = rospy.Time(0)
        self.signal_points, self.signal_movements = self.load_signal_map(
            rospy.get_param("~map_dir"))

        self.path_pub = rospy.Publisher("/privileged_expert/path", Path, queue_size=1)
        self.speed_pub = rospy.Publisher("/privileged_expert/target_velocity", Float32, queue_size=1)
        self.behavior_pub = rospy.Publisher("/privileged_expert/behavior", String, queue_size=1)
        self.action_pub = rospy.Publisher("/privileged_expert/action", String, queue_size=1)
        self.mode_pub = rospy.Publisher("/privileged_expert/mode_scores", Float32MultiArray, queue_size=1)
        self.score_pub = rospy.Publisher("/privileged_expert/proposal_scores", Float32MultiArray, queue_size=1)
        self.label_pub = rospy.Publisher("/privileged_expert/teacher_labels", String, queue_size=1)
        self.marker_pub = rospy.Publisher("/privileged_expert/markers", MarkerArray, queue_size=1)
        self.ready_pub = rospy.Publisher("/privileged_expert/ready", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/localization/kinematic_state", Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber("/local_route", Path, self.route_cb, queue_size=1)
        rospy.Subscriber("/global_path", Path, self.global_cb, queue_size=1)
        rospy.Subscriber("/mgeo_target_velocity", Float32, self.speed_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~object_topic", "/Object_topic"), ObjectStatusList, self.object_cb, queue_size=1)
        rospy.Subscriber(rospy.get_param("~traffic_command_topic", "/SetTrafficLight"),
                         SetTrafficLight, self.traffic_cb, queue_size=20)
        rospy.Timer(rospy.Duration(1.0 / max(self.hz, 1.0)), self.plan)
        rospy.loginfo("MORAI privileged expert ready; object ground truth: %s", rospy.get_param("~object_topic", "/Object_topic"))

    def odom_cb(self, msg):
        with self.lock: self.odom = msg; self.odom_stamp = rospy.Time.now()
    def route_cb(self, msg):
        with self.lock: self.route = msg
    def global_cb(self, msg):
        with self.lock: self.global_path = msg
    def speed_cb(self, msg):
        with self.lock: self.road_speed = max(float(msg.data), 0.0)
    def object_cb(self, msg):
        with self.lock: self.objects = msg; self.object_stamp = rospy.Time.now()

    def traffic_cb(self, msg):
        with self.lock:
            light_id = str(msg.trafficLightIndex)
            # The traffic manager publishes the signal currently controlling
            # the ego approach. Do not retain old red phases from intersections
            # that have already been passed.
            self.active_traffic_id = light_id
            self.traffic_states = {light_id: int(msg.trafficLightStatus)}
            self.traffic_state_stamp = rospy.Time.now()

    @staticmethod
    def load_signal_map(map_dir):
        nodes = json.load(open(os.path.join(map_dir, "node_set.json"), encoding="utf-8"))
        links = json.load(open(os.path.join(map_dir, "link_set.json"), encoding="utf-8"))
        node_light = {str(node.get("idx")): str(node.get("traffic_light_id"))
                      for node in nodes if node.get("traffic_light_id") and
                      not str(node.get("traffic_light_id")).upper().startswith("LCS")}
        points = [(light, np.asarray(node["point"][:2], dtype=float))
                  for node in nodes if node.get("point")
                  for light in [node_light.get(str(node.get("idx")))] if light]
        
        try:
            controls = json.load(open(os.path.join(map_dir, "traffic_light_control_set.json"), encoding="utf-8"))
            existing = set(l for l, _ in points)
            for c in controls:
                idx = c.get("idx")
                if c.get("type") == "car" and idx and idx not in existing:
                    points.append((idx, np.asarray(c["point"][:2], dtype=float)))
        except Exception: pass
        movements = {}
        for link in links:
            light = node_light.get(str(link.get("from_node_idx")))
            related = str(link.get("related_signal") or "").lower()
            xy = np.asarray(link.get("points", []), dtype=float)
            if light and related and len(xy) >= 2:
                movements.setdefault(light, []).append((related, xy[:, :2]))
        return points, movements

    def next_signal(self, reference):
        if not self.traffic_states or not self.signal_points:
            return None
        if ((rospy.Time.now() - self.traffic_state_stamp).to_sec() >
                self.traffic_state_timeout):
            return None
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
        best = None
        for light_id, point in self.signal_points:
            if self.active_traffic_id is not None and light_id != self.active_traffic_id:
                continue
            if light_id not in self.traffic_states:
                continue
            distance = np.linalg.norm(reference - point, axis=1)
            index = int(np.argmin(distance))
            if distance[index] > self.signal_path_tolerance or arc[index] < 0.5:
                continue
            before = max(index - 6, 0); after = min(index + 6, len(reference) - 1)
            yaw_in = math.atan2(reference[index, 1] - reference[before, 1], reference[index, 0] - reference[before, 0])
            yaw_out = math.atan2(reference[after, 1] - reference[index, 1], reference[after, 0] - reference[index, 0])
            turn_angle = math.atan2(math.sin(yaw_out-yaw_in), math.cos(yaw_out-yaw_in))
            maneuver = "LEFT" if turn_angle > math.radians(20.0) else ("RIGHT" if turn_angle < -math.radians(20.0) else "STRAIGHT")
            # Prefer authoritative MGeo movement semantics over curvature.
            # Match the route after the stop line against outgoing links.
            movement_candidates = []
            route_after = reference[index:]
            for related, movement_xy in self.signal_movements.get(light_id, []):
                separation = float(np.min(np.linalg.norm(
                    route_after[:, None, :] - movement_xy[None, :, :], axis=2)))
                movement_candidates.append((separation, related))
            if movement_candidates:
                separation, related = min(movement_candidates)
                if separation <= self.signal_path_tolerance:
                    if related == "left": maneuver = "LEFT"
                    elif related == "left_unprotected": maneuver = "LEFT_UNPROTECTED"
                    elif related.startswith("right"): maneuver = "RIGHT"
                    elif related == "straight": maneuver = "STRAIGHT"
            candidate = (float(arc[index]), light_id, self.traffic_states[light_id], maneuver)
            if best is None or candidate[0] < best[0]:
                best = candidate
        return best

    @staticmethod
    def yaw(odom):
        q = odom.pose.pose.orientation
        return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]

    def route_map_xy(self, route, odom):
        xy = np.asarray([[p.pose.position.x, p.pose.position.y] for p in route.poses], dtype=float)
        if route.header.frame_id not in ("base_link", "base_footprint"):
            return xy
        yaw = self.yaw(odom); c, s = math.cos(yaw), math.sin(yaw)
        return np.column_stack((odom.pose.pose.position.x + c * xy[:,0] - s * xy[:,1],
                                odom.pose.pose.position.y + s * xy[:,0] + c * xy[:,1]))

    def resample(self, xy):
        length = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
        keep = np.r_[True, np.diff(length) > 1e-4]; xy, length = xy[keep], length[keep]
        if len(xy) < 2: return None
        stations = np.arange(0.0, min(length[-1], self.horizon) + self.spacing * .5, self.spacing)
        return np.column_stack((np.interp(stations, length, xy[:,0]), np.interp(stations, length, xy[:,1])))

    def global_reference(self, global_path, odom):
        points = np.asarray([
            [pose.pose.position.x, pose.pose.position.y]
            for pose in global_path.poses
        ], dtype=float)
        if len(points) < 2:
            return None
        ego = np.asarray([
            odom.pose.pose.position.x, odom.pose.pose.position.y
        ])
        nearest = int(np.argmin(np.linalg.norm(points - ego, axis=1)))
        closed = len(points) > 2 and np.linalg.norm(points[0] - points[-1]) < 1.0
        ordered = points[nearest:]
        if closed:
            unique = points[:-1]
            nearest = min(nearest, len(unique) - 1)
            ordered = np.vstack((unique[nearest:], unique[:nearest + 1]))
        distance = np.r_[
            0.0, np.cumsum(np.linalg.norm(np.diff(ordered, axis=0), axis=1))
        ]
        end = int(np.searchsorted(distance, self.horizon, side="right"))
        return self.resample(ordered[:max(end, 2)])

    @staticmethod
    def path_geometry(xy):
        delta = np.gradient(xy, axis=0); yaw = np.arctan2(delta[:,1], delta[:,0])
        normal = np.column_stack((-np.sin(yaw), np.cos(yaw)))
        return yaw, normal

    def object_array(self, objects, odom):
        result = []
        if objects is None: return result
        source_stamp = (objects.header.stamp.to_sec()
                        if objects.header.stamp != rospy.Time(0)
                        else rospy.Time.now().to_sec())
        ego_yaw = self.yaw(odom); c, s = math.cos(ego_yaw), math.sin(ego_yaw)
        for kind, items in (("npc", objects.npc_list), ("pedestrian", objects.pedestrian_list), ("obstacle", objects.obstacle_list)):
            for item in items:
                position = np.asarray([item.position.x, item.position.y], dtype=float)
                velocity = np.asarray([item.velocity.x, item.velocity.y], dtype=float)
                if self.object_frame in ("base_link", "base_footprint", "ego"):
                    position = np.asarray([odom.pose.pose.position.x + c * position[0] - s * position[1],
                                           odom.pose.pose.position.y + s * position[0] + c * position[1]])
                    velocity = np.asarray([c * velocity[0] - s * velocity[1], s * velocity[0] + c * velocity[1]])
                key = (kind, int(item.unique_id))
                previous = self.object_tracks.get(key)
                if previous is not None and source_stamp > previous[1] + .02:
                    dt = source_stamp - previous[1]
                    inferred = (position - previous[0]) / dt
                    inferred_speed = float(np.linalg.norm(inferred))
                    # ObjectInfo can report zero walker velocity. Recover it
                    # from privileged position history while rejecting jumps.
                    if kind == "pedestrian" and .1 < inferred_speed < 8.0:
                        velocity = inferred
                if previous is None or source_stamp > previous[1] + .02:
                    self.object_tracks[key] = (position.copy(), source_stamp)
                width = max(float(item.size.y), .5)
                if kind == "pedestrian":
                    width += self.pedestrian_width_padding
                result.append({"kind": kind, "id": int(item.unique_id),
                               "p": position, "v": velocity,
                               "yaw": math.radians(float(item.heading)),
                               "length": max(float(item.size.x), .5),
                               "width": width,
                               # UDP sample is vehicle length in x, width in y.
                               "r": .5 * width})
        return result

    def static_path_hazard(self, reference, objects):
        """Find static objects in the MGeo center-path corridor.

        This geometric precheck is intentionally independent of the temporal
        rollout. It starts avoidance before a collision proposal is reached.
        """
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
        best = None
        for obj in objects:
            if obj["kind"] != "obstacle":
                continue
            distances = np.linalg.norm(reference - obj["p"], axis=1)
            index = int(np.argmin(distances))
            station = float(arc[index])
            corridor = .5 * self.ego_width + .5 * obj["width"] + self.margin
            if .5 < station <= self.avoid_distance and distances[index] <= corridor:
                candidate = (station, obj)
                if best is None or station < best[0]:
                    best = candidate
        return best

    def leading_vehicle(self, reference, objects):
        """Return the closest NPC occupying the MGeo center corridor."""
        arc = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(reference, axis=0), axis=1))]
        best = None
        for obj in objects:
            if obj["kind"] != "npc":
                continue
            distances = np.linalg.norm(reference - obj["p"], axis=1)
            index = int(np.argmin(distances))
            station = float(arc[index])
            corridor = .5 * self.ego_width + .5 * obj["width"] + self.margin
            if .5 < station <= self.lead_vehicle_distance and distances[index] <= corridor:
                candidate = (station, obj)
                if best is None or station < best[0]:
                    best = candidate
        return best

    def score(self, xy, initial_speed, target_speed, offset, objects):
        meta = evaluate_proposal(xy, initial_speed, target_speed, objects,
                                 self.ego_length, self.ego_width, self.margin,
                                 self.predict_sec, .2)
        return proposal_cost(meta, offset, target_speed,
                             max(self.road_speed, self.min_cruise_speed),
                             self.weights), meta

    def make_path(self, xy, stamp):
        yaw, _ = self.path_geometry(xy); msg = Path(); msg.header.stamp = stamp; msg.header.frame_id = "map"
        for point, angle in zip(xy, yaw):
            pose = PoseStamped(); pose.header = msg.header; pose.pose.position.x = float(point[0]); pose.pose.position.y = float(point[1])
            q = quaternion_from_euler(0, 0, float(angle)); pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = q
            msg.poses.append(pose)
        return msg

    def classify(self, meta, speed, road_speed, offset, ego_speed):
        distance = meta["object_distance"]
        interacts = meta["clearance"] <= self.interaction_clearance
        kind = meta["object_kind"]
        # A pedestrian near the road is not itself a hazard. Stop only when
        # its predicted oriented box overlaps the ego rollout.
        if kind == "pedestrian":
            if meta["collision"]:
                if meta.get("collision_time", float("inf")) <= self.emergency_ttc:
                    return "EMERGENCY_STOP", 0.0
                return "STOP", 0.0
            approaching = (meta["object_speed"] > .15 and
                           meta["clearance"] <= self.pedestrian_caution_clearance and
                           meta.get("closest_time", float("inf")) <= self.pedestrian_caution_time)
            if approaching:
                target = idm_target_speed(road_speed, meta["object_length"], ego_speed,
                                          0.0, max(distance, .1), 6.0, 1.0)
                return "YIELD_PEDESTRIAN", min(speed, target)
            return "CRUISE", speed
        # Static obstacles are handled geometrically by the lateral proposal
        # selector. A safe shifted path must not be stopped by circle clearance.
        if kind == "obstacle" and not meta["collision"]:
            return ("AVOID", speed) if abs(offset) > .2 else ("CRUISE", speed)
        if (meta["collision"] and
                meta.get("collision_time", float("inf")) <= self.emergency_ttc):
            return "EMERGENCY_STOP", 0.0
        if interacts and speed > .1 and distance / speed < self.emergency_ttc: return "EMERGENCY_STOP", 0.0
        if interacts and distance <= self.stop_distance: return "STOP", 0.0
        if abs(offset) > .2 and distance <= self.avoid_distance: return "AVOID", speed
        if interacts and distance <= self.follow_distance:
            minimum_distance = 4.0
            time_headway = 0.1 if meta["object_kind"] in ("pedestrian", "obstacle") else 0.25
            target = idm_target_speed(road_speed, meta["object_length"], ego_speed,
                                      meta["object_speed"], distance,
                                      minimum_distance, time_headway)
            return "FOLLOW", min(speed, target)
        return "CRUISE", speed

    def markers(self, proposals, selected, behavior, objects, target, signal):
        array = MarkerArray(); stamp = rospy.Time.now()
        clear = Marker(); clear.action = Marker.DELETEALL; array.markers.append(clear)
        for index, proposal in enumerate(proposals):
            marker = Marker(); marker.header.frame_id = "map"; marker.header.stamp = stamp; marker.ns = "expert_proposals"; marker.id = index
            marker.type = Marker.LINE_STRIP; marker.action = Marker.ADD; marker.pose.orientation.w = 1.0; marker.scale.x = .18 if index != selected else .42
            marker.color.a = .35 if index != selected else 1.0; marker.color.r = 1.0 if proposal["meta"]["collision"] else .1; marker.color.g = 1.0 if index == selected else .4; marker.color.b = .9
            marker.points = [Point(float(p[0]), float(p[1]), .15) for p in proposal["xy"]]; array.markers.append(marker)
        for index, obj in enumerate(objects):
            body = Marker(); body.header.frame_id = "map"; body.header.stamp = stamp
            body.ns = "privileged_objects"; body.id = 20000 + index; body.type = Marker.CUBE; body.action = Marker.ADD
            body.pose.position.x = float(obj["p"][0]); body.pose.position.y = float(obj["p"][1]); body.pose.position.z = .55
            body.pose.orientation.w = 1.0; body.scale.x = obj["length"]; body.scale.y = obj["width"]; body.scale.z = 1.1
            body.color.a = .85
            if obj["kind"] == "pedestrian": body.color.r, body.color.g, body.color.b = 1.0, .15, .9
            elif obj["kind"] == "obstacle": body.color.r, body.color.g, body.color.b = 1.0, .25, .05
            else: body.color.r, body.color.g, body.color.b = .1, .65, 1.0
            array.markers.append(body)
            prediction = Marker(); prediction.header = body.header; prediction.ns = "object_prediction"; prediction.id = 30000 + index
            prediction.type = Marker.ARROW; prediction.action = Marker.ADD; prediction.pose.orientation.w = 1.0
            prediction.scale.x = .16; prediction.scale.y = .32; prediction.scale.z = .32; prediction.color = body.color
            prediction.points = [Point(float(obj["p"][0]), float(obj["p"][1]), .7),
                                 Point(float(obj["p"][0] + obj["v"][0] * self.predict_sec),
                                       float(obj["p"][1] + obj["v"][1] * self.predict_sec), .7)]
            array.markers.append(prediction)
            tag = Marker(); tag.header = body.header; tag.ns = "object_labels"; tag.id = 40000 + index
            tag.type = Marker.TEXT_VIEW_FACING; tag.action = Marker.ADD; tag.pose.orientation.w = 1.0
            tag.pose.position.x = float(obj["p"][0]); tag.pose.position.y = float(obj["p"][1]); tag.pose.position.z = 1.8
            tag.scale.z = .45; tag.color.r = tag.color.g = tag.color.b = tag.color.a = 1.0
            tag.text = "%s #%d  %.1fkm/h" % (obj["kind"].upper(), obj["id"], np.linalg.norm(obj["v"]) * 3.6)
            array.markers.append(tag)
        text = Marker(); text.header.frame_id = "map"; text.header.stamp = stamp; text.ns = "expert_hud"; text.id = 10000; text.type = Marker.TEXT_VIEW_FACING; text.action = Marker.ADD; text.pose.orientation.w = 1.0
        if proposals: text.pose.position.x = float(proposals[selected]["xy"][0,0]); text.pose.position.y = float(proposals[selected]["xy"][0,1]); text.pose.position.z = 2.2
        best = proposals[selected]; meta = best["meta"]
        signal_text = "none" if signal is None else "%s %s=%d %.1fm" % (signal[1], signal[3], signal[2], signal[0])
        text.scale.z = .55; text.color.r = text.color.g = text.color.b = text.color.a = 1.0
        text.text = ("EXPERT %s  v=%.1fkm/h  offset=%+.1fm\n"
                     "risk=%s#%d  path_dist=%.1fm  clearance=%.2fm\n"
                     "signal=%s" % (behavior, target * 3.6, best["offset"], meta["object_kind"],
                                      meta["object_id"], meta["object_distance"], meta["clearance"], signal_text))
        array.markers.append(text)
        return array

    def plan(self, _event):
        with self.lock:
            odom, route, objects, road_speed, global_path = self.odom, self.route, self.objects, self.road_speed, self.global_path
            odom_stamp, object_stamp = self.odom_stamp, self.object_stamp
        now = rospy.Time.now()
        objects_ready = (objects is not None and
                         (now - object_stamp).to_sec() <= self.input_timeout)
        inputs_ready = (odom is not None and global_path is not None and len(global_path.poses) >= 2
                        and (now - odom_stamp).to_sec() <= self.input_timeout
                        and (objects_ready or not self.require_objects))
        self.ready_pub.publish(Bool(inputs_ready))
        if not inputs_ready:
            self.speed_pub.publish(Float32(0.0))
            self.mode_pub.publish(Float32MultiArray(data=[1.0, 0.0]))
            self.behavior_pub.publish(String("INPUT_STALE"))
            rospy.logwarn_throttle(2.0, "Privileged expert safe stop: odometry/path%s missing or stale",
                                   "/Object_topic" if self.require_objects else "")
            return
        if not objects_ready:
            objects = None
            rospy.logwarn_throttle(5.0, "Object_topic unavailable: MGeo cruise active without privileged obstacle avoidance")
        reference = self.global_reference(global_path, odom)
        if reference is None and route is not None and len(route.poses) >= 2:
            reference = self.resample(self.route_map_xy(route, odom))
        if reference is None: self.speed_pub.publish(Float32(0.0)); return
        road_speed = min(road_speed * self.speed_ratio, self.maximum_speed)
        _, normal = self.path_geometry(reference); object_list = self.object_array(objects, odom)
        ego_twist = odom.twist.twist.linear
        ego_speed = math.hypot(ego_twist.x, ego_twist.y)
        proposals = []
        for offset in self.offsets:
            if abs(offset) > self.max_offset: continue
            shifted = smooth_shift(reference, normal, offset, self.lane_change_length)
            for factor in self.speed_factors:
                target_speed = max(road_speed * factor, 0.0)
                score, meta = self.score(shifted, ego_speed, target_speed,
                                         offset, object_list)
                proposals.append({"xy": shifted, "speed": target_speed,
                                  "offset": offset, "speed_factor": factor,
                                  "score": score, "meta": meta})
        center_fast = min((i for i, p in enumerate(proposals) if abs(p["offset"]) < .2),
                          key=lambda i: abs(proposals[i]["speed"] - road_speed))
        center_meta = proposals[center_fast]["meta"]
        hazard_kind = center_meta["object_kind"]
        static_hazard = self.static_path_hazard(reference, object_list)
        lead = self.leading_vehicle(reference, object_list)
        signal = self.next_signal(reference)
        rospy.loginfo_throttle(1.0, f"Signal check: active_id={self.active_traffic_id}, signal={signal}")
        signal_blocks = False
        if signal is not None:
            signal_status, maneuver = signal[2], signal[3]
            signal_green = (bool(signal_status & 32) if maneuver == "LEFT"
                            else bool(signal_status & 16))
            signal_blocks = not signal_green
        obstacle_distance = (static_hazard[0] if static_hazard is not None
                             else (center_meta["object_distance"]
                                   if hazard_kind == "obstacle" else float("inf")))
        signal_before_obstacle = (signal_blocks and
                                  signal[0] <= obstacle_distance)
        pedestrian_blocks = (center_meta["object_kind"] == "pedestrian" and
                             center_meta["collision"])
        npc_overtake = (lead is not None and
                        lead[0] <= self.npc_overtake_distance and
                        float(np.linalg.norm(lead[1]["v"])) <= self.npc_overtake_max_speed and
                        not signal_blocks and not pedestrian_blocks)
        avoid_needed = (not signal_before_obstacle and not pedestrian_blocks and
                        (static_hazard is not None or
                        npc_overtake or
                        (center_meta["collision"] and hazard_kind == "obstacle"
                         and center_meta["object_distance"] <= self.avoid_distance)))
        if avoid_needed:
            safe_moving = [i for i, p in enumerate(proposals)
                           if not p["meta"]["collision"] and p["speed"] > .1
                           and abs(p["offset"]) > .2]
            if static_hazard is not None:
                _, static_obj = static_hazard
                required = (.5 * self.ego_width + .5 * static_obj["width"]
                            + self.margin)
                geometry_safe = [i for i in safe_moving
                                 if float(np.min(np.linalg.norm(
                                     proposals[i]["xy"] - static_obj["p"], axis=1)))
                                 > required]
                if geometry_safe:
                    safe_moving = geometry_safe
            fast_safe = [i for i in safe_moving
                         if proposals[i]["speed"] >= .65 * max(road_speed, .1)]
            if fast_safe:
                safe_moving = fast_safe
            selected = min(safe_moving, key=lambda i: proposals[i]["score"]) if safe_moving else center_fast
        elif center_meta["collision"]:
            # Pedestrians and moving actors are handled longitudinally while
            # keeping the MGeo geometry. Their forecast meta drives STOP/IDM.
            selected = center_fast
        else:
            selected = center_fast
        best = proposals[selected]
        behavior, target = self.classify(best["meta"], best["speed"], road_speed,
                                         best["offset"], ego_speed)
        overtaking = npc_overtake and abs(best["offset"]) > .2
        if (lead is not None and not overtaking and
                behavior not in ("STOP", "EMERGENCY_STOP")):
            lead_distance, lead_actor = lead
            lead_target = idm_target_speed(
                road_speed, lead_actor["length"], ego_speed,
                float(np.linalg.norm(lead_actor["v"])), lead_distance,
                6.0, 1.2)
            if lead_target < target:
                target = lead_target
                behavior = "FOLLOW"
        if overtaking:
            behavior = "AVOID_NPC"
        if signal is not None:
            signal_distance, signal_id, signal_status, maneuver = signal
            # MORAI bit mask: red=1, yellow=4, green=16, green-left=32.
            green = bool(signal_status & 32) if maneuver == "LEFT" else bool(signal_status & 16)
            # At highway speed, fixed urban braking distance is insufficient.
            # Keep 45 m in town and extend it from current kinetic energy.
            dynamic_signal_slowdown = max(
                self.signal_slowdown_distance,
                ego_speed * ego_speed / (2.0 * 5.5) + ego_speed * .5 +
                self.signal_stop_buffer)
            if not green and signal_distance <= dynamic_signal_slowdown:
                remaining = max(signal_distance - self.signal_stop_buffer, 0.0)
                target = min(target, road_speed * min(
                    remaining / max(dynamic_signal_slowdown - self.signal_stop_buffer, 0.1), 1.0))
                behavior = "SLOW_SIGNAL"
                if remaining <= self.signal_full_stop_lead:
                    behavior, target = "STOP_SIGNAL", 0.0
        if global_path is not None and global_path.poses:
            goal = global_path.poses[-1].pose.position
            ego = odom.pose.pose.position
            first = global_path.poses[0].pose.position
            closed_route = math.hypot(goal.x - first.x, goal.y - first.y) < 1.0
            if not closed_route and math.hypot(goal.x - ego.x, goal.y - ego.y) <= 2.5:
                behavior, target = "MISSION_COMPLETE", 0.0
        if behavior in ("STOP", "EMERGENCY_STOP"): target = 0.0
        stamp = rospy.Time.now(); path = self.make_path(best["xy"], stamp)
        self.path_pub.publish(path); self.speed_pub.publish(Float32(target)); self.behavior_pub.publish(String(behavior)); self.action_pub.publish(String(behavior))
        stop_score = 1.0 if behavior in ("STOP", "STOP_SIGNAL", "EMERGENCY_STOP", "MISSION_COMPLETE") else 0.0
        self.mode_pub.publish(Float32MultiArray(data=[stop_score, 1.0-stop_score])); self.score_pub.publish(Float32MultiArray(data=[float(p["score"]) for p in proposals]))
        label = {"stamp": stamp.to_sec(), "behavior": behavior, "target_speed_mps": target,
                 "lateral_offset_m": best["offset"], "clearance_m": best["meta"]["clearance"],
                 "object_distance_m": best["meta"]["object_distance"], "object_kind": best["meta"]["object_kind"],
                 "object_id": best["meta"]["object_id"],
                 "traffic_light": None if signal is None else {"distance_m": signal[0], "id": signal[1], "status": signal[2], "maneuver": signal[3]},
                 "proposal_index": selected, "proposal_count": len(proposals)}
        self.label_pub.publish(String(json.dumps(label)))
        self.marker_pub.publish(self.markers(proposals, selected, behavior, object_list, target, signal))
        self.last_behavior = behavior; self.last_plan = best
        rospy.loginfo_throttle(1.0, "Privileged expert: %s target=%.1f km/h offset=%.1fm clearance=%.2fm proposals=%d", behavior, target*3.6, best["offset"], best["meta"]["clearance"], len(proposals))


if __name__ == "__main__":
    PrivilegedExpert(); rospy.spin()

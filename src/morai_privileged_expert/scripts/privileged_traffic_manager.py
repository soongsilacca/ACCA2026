#!/usr/bin/env python3
import json
import os
import random
import socket
import struct

import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus, SetTrafficLight
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray


class PrivilegedTrafficManager:
    def __init__(self):
        rospy.init_node("privileged_traffic_manager")
        self.map_dir = rospy.get_param("~map_dir")
        self.local_ip = rospy.get_param("~local_ip", "127.0.0.1")
        self.local_port = int(rospy.get_param("~local_port", 7503))
        self.morai_ip = rospy.get_param("~morai_ip", "127.0.0.1")
        self.morai_port = int(rospy.get_param("~morai_port", 7607))
        self.radius = float(rospy.get_param("~activation_radius_m", 80.0))
        self.signal_group_radius = float(rospy.get_param("~signal_group_radius_m", 25.0))
        self.signal_group_distance_tolerance = float(
            rospy.get_param("~signal_group_distance_tolerance_m", 10.0))
        self.upstream_link_depth = int(rospy.get_param("~upstream_link_depth", 3))
        self.status = None; self.ego_link_id = ""; self.last_light = None
        self.resend_sec = float(rospy.get_param("~resend_sec", 0.5))
        self.min_phase_sec = float(rospy.get_param("~min_phase_sec", 15.0))
        self.max_phase_sec = float(rospy.get_param("~max_phase_sec", 25.0))
        self.red_min_phase_sec = float(rospy.get_param("~red_min_phase_sec", 2.0))
        self.red_max_phase_sec = float(rospy.get_param("~red_max_phase_sec", 3.0))
        self.left_min_phase_sec = float(rospy.get_param("~left_min_phase_sec", 8.0))
        self.left_max_phase_sec = float(rospy.get_param("~left_max_phase_sec", 9.0))
        self.yellow_min_phase_sec = float(rospy.get_param("~yellow_min_phase_sec", 2.0))
        self.yellow_max_phase_sec = float(rospy.get_param("~yellow_max_phase_sec", 2.5))
        # MORAI bit mask: red=1, yellow=4, straight green=16,
        # red + protected-left arrow=33.  Never shuffle these independently:
        # the order below models a Korean protected-left signal cycle.
        self.phase_sequence = list(map(int, rospy.get_param(
            "~phase_sequence", [16, 4, 1, 33, 4, 1])))
        self.last_send = rospy.Time(0)
        self.next_switch = rospy.Time(0); self.current_status = {}
        self.phase_indices = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.local_ip, self.local_port))
        rospy.on_shutdown(self.sock.close)
        control_path = os.path.join(self.map_dir, "traffic_light_control_set.json")
        controls = json.load(open(control_path, encoding="utf-8"))
        nodes = json.load(open(os.path.join(self.map_dir, "node_set.json"), encoding="utf-8"))
        links = json.load(open(os.path.join(self.map_dir, "link_set.json"), encoding="utf-8"))
        node_light = {str(n.get("idx")): str(n.get("traffic_light_id")) for n in nodes if n.get("traffic_light_id") and not str(n.get("traffic_light_id")).upper().startswith("LCS")}
        self.controlled_links = []
        link_by_id = {str(link.get("idx")): link for link in links}
        links_to_node = {}
        for link in links:
            links_to_node.setdefault(str(link.get("to_node_idx")), []).append(link)
        self.light_approach_links = {}
        points = [(light, np.asarray(node["point"][:2], dtype=float))
                  for node in nodes if node.get("point")
                  for light in [node_light.get(str(node.get("idx")))] if light]
        
        try:
            controls = json.load(open(os.path.join(self.map_dir, "traffic_light_control_set.json"), encoding="utf-8"))
            existing = set(l for l, _ in points)
            for c in controls:
                idx = c.get("idx")
                if c.get("type") == "car" and idx and idx not in existing:
                    points.append((idx, np.asarray(c["point"][:2], dtype=float)))
        except Exception: pass
        for link in links:
            light = node_light.get(str(link.get("to_node_idx")))
            p = np.asarray(link.get("points", []), dtype=float)
            if light and len(p):
                self.controlled_links.append((light, p[:, :2]))
                mapped = self.light_approach_links.setdefault(light, set())
                frontier = [link]
                for _ in range(self.upstream_link_depth + 1):
                    next_frontier = []
                    for current in frontier:
                        current_id = str(current.get("idx"))
                        if current_id in mapped:
                            continue
                        mapped.add(current_id)
                        next_frontier.extend(links_to_node.get(
                            str(current.get("from_node_idx")), []))
                    frontier = next_frontier
        self.link_to_lights = {}
        for light, approach_links in self.light_approach_links.items():
            for link_id in approach_links:
                self.link_to_lights.setdefault(link_id, set()).add(light)
        # Prefer the simulator's authoritative signal positions. The link
        # association remains as a fallback for older map exports.
        self.signal_points = []
        for signal in controls:
            light_id, point = str(signal.get("idx", "")), signal.get("point")
            if (signal.get("type") == "car" and light_id and
                    not light_id.upper().startswith("LCS") and
                    isinstance(point, list) and len(point) >= 2):
                self.signal_points.append((light_id, np.asarray(point[:2], dtype=float)))
        self.signal_point_by_id = dict(self.signal_points)
        
        self.global_path_csv = rospy.get_param("~global_path_csv", "")
        if self.global_path_csv and os.path.exists(self.global_path_csv):
            import csv
            from scipy.spatial import KDTree
            global_path = []
            with open(self.global_path_csv, 'r') as f:
                reader = csv.reader(f)
                for row in reader:
                    if not row or row[0] == 'stamp': continue
                    try:
                        global_path.append([float(row[1]), float(row[2])])
                    except (ValueError, IndexError):
                        continue
            if global_path:
                global_path_arr = np.array(global_path)
                self.tree = KDTree(global_path_arr)
                filtered_controlled_links = []
                for light, points in self.controlled_links:
                    dists, indices = self.tree.query(points)
                    min_idx = np.argmin(dists)
                    if dists[min_idx] < 20.0:
                        path_idx = indices[min_idx]
                        # Link direction
                        link_dir = points[-1] - points[max(0, len(points)-5)]
                        norm = np.linalg.norm(link_dir)
                        if norm < 1e-6: continue
                        link_dir = link_dir / norm
                        
                        # Path direction
                        idx_next = min(path_idx + 5, len(global_path_arr) - 1)
                        idx_prev = max(path_idx - 5, 0)
                        if idx_next == idx_prev: continue
                        path_dir = global_path_arr[idx_next] - global_path_arr[idx_prev]
                        p_norm = np.linalg.norm(path_dir)
                        if p_norm < 1e-6: continue
                        path_dir = path_dir / p_norm
                        
                        # Only keep if facing same direction
                        if np.dot(link_dir, path_dir) > 0.0:
                            filtered_controlled_links.append((light, points))
                rospy.loginfo("Filtered controlled links from %d to %d using global path", len(self.controlled_links), len(filtered_controlled_links))
                self.controlled_links = filtered_controlled_links

        self.pub = rospy.Publisher("/privileged_expert/controlled_traffic_light", String, queue_size=1, latch=True)
        self.command_pub = rospy.Publisher("/SetTrafficLight", SetTrafficLight, queue_size=10, latch=True)
        self.marker_pub = rospy.Publisher("/privileged_expert/traffic_light_markers", MarkerArray, queue_size=1, latch=True)
        
        self._publish_markers()

        rospy.Subscriber("/localization/kinematic_state", Odometry, self.status_cb, queue_size=1)
        rospy.Subscriber("/morai/ego_vehicle_status", EgoVehicleStatus,
                         self.ego_status_cb, queue_size=1)
        rospy.Timer(rospy.Duration(.2), self.timer_cb)
        rospy.loginfo("Privileged traffic manager ready: %d signals, %d mapped approaches; UDP local %s:%d -> MORAI %s:%d",
                      len(self.signal_points), len(self.controlled_links), self.local_ip,
                      self.local_port, self.morai_ip, self.morai_port)

    def status_cb(self, msg): self.status = msg
    def ego_status_cb(self, msg): self.ego_link_id = str(msg.link_id)

    def _publish_markers(self):
        msg = MarkerArray()
        target_list = rospy.get_param("~target_light_ids", [])
        self.valid_lights = set(target_list)
                    
        for i, light_id in enumerate(self.valid_lights):
            pos = self.signal_point_by_id.get(light_id)
            if pos is None: continue
            
            # Sphere marker
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "traffic_lights"
            m.id = i * 2
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = pos[0]
            m.pose.position.y = pos[1]
            m.pose.position.z = 2.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 2.0
            m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.8
            msg.markers.append(m)
            
            # Text marker
            t = Marker()
            t.header.frame_id = "map"
            t.ns = "traffic_lights_text"
            t.id = i * 2 + 1
            t.type = Marker.TEXT_VIEW_FACING
            t.action = Marker.ADD
            t.pose.position.x = pos[0]
            t.pose.position.y = pos[1]
            t.pose.position.z = 4.0
            t.pose.orientation.w = 1.0
            t.scale.z = 1.5
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.text = light_id
            msg.markers.append(t)
            
        self.marker_pub.publish(msg)

    @staticmethod
    def packet(light_id, status=48):
        return struct.pack('<14si3i12sh2s', b'#TrafficLight$', 14, 0, 0, 0,
                           light_id.encode('utf-8').ljust(12, b'\0')[:12], int(status), b'\r\n')

    def phase_duration(self, status):
        status = int(status)
        if status & 4:
            return random.uniform(self.yellow_min_phase_sec,
                                  self.yellow_max_phase_sec)
        if status & 32:
            return random.uniform(self.left_min_phase_sec,
                                  self.left_max_phase_sec)
        if status & 1:
            return random.uniform(self.red_min_phase_sec,
                                  self.red_max_phase_sec)
        return random.uniform(self.min_phase_sec, self.max_phase_sec)

    def timer_cb(self, _event):
        if self.status is None: return
        point = np.asarray([self.status.pose.pose.position.x,
                            self.status.pose.pose.position.y], dtype=float)
        q = self.status.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        forward = np.asarray([np.cos(yaw), np.sin(yaw)])
        normal = np.asarray([-forward[1], forward[0]])
        targets = []
        for light_id in getattr(self, 'valid_lights', set()):
            pos = self.signal_point_by_id.get(light_id)
            if pos is None: continue
            relative = pos - point
            longitudinal = float(relative.dot(forward))
            lateral = abs(float(relative.dot(normal)))
            if -25.0 <= longitudinal <= self.radius and lateral <= 15.0:
                targets.append((light_id, max(longitudinal, 0.0)))
        if not targets:
            return
        targets = [(light, distance) for light, distance in targets if distance <= self.radius]
        targets.sort(key=lambda item: item[1])
        if not targets: return
        now = rospy.Time.now(); raw_light, distance = targets[0]
        
        group = set([raw_light])
        for other_light in getattr(self, 'valid_lights', set()):
            other_pos = self.signal_point_by_id.get(other_light)
            if other_pos is None: continue
            relative = other_pos - point
            long = float(relative.dot(forward))
            if abs(long - distance) <= self.signal_group_distance_tolerance:
                group.add(other_light)
        
        group = sorted(list(group))
        light = group[0]  # Canonical light for the group prevents state flip-flopping

        if (light != self.last_light and light not in getattr(self, 'last_group', [])) or now >= self.next_switch:
            if not self.phase_sequence:
                rospy.logerr_throttle(2.0, "Traffic phase_sequence is empty")
                return
            # A newly encountered intersection begins at a random point in a
            # valid cycle.  Afterwards it can only advance in legal order.
            if light not in self.phase_indices:
                phase_index = random.randrange(len(self.phase_sequence))
            else:
                phase_index = (self.phase_indices[light] + 1) % len(self.phase_sequence)
            self.phase_indices[light] = phase_index
            self.current_status[light] = self.phase_sequence[phase_index]
            self.next_switch = now + rospy.Duration(
                self.phase_duration(self.current_status[light]))
        if (now - self.last_send).to_sec() < self.resend_sec: return
        status = self.current_status[light]
        try:
            for group_light in group:
                self.sock.sendto(self.packet(group_light, status),
                                 (self.morai_ip, self.morai_port))
                rospy.sleep(0.05)  # 50ms sleep to prevent MORAI Unity engine from overwriting the first packet
                self.current_status[group_light] = status
            
            self.command_pub.publish(SetTrafficLight(
                trafficLightIndex=light, trafficLightStatus=status))
        except OSError as error:
            rospy.logerr_throttle(2.0, "Traffic-light UDP failed: %s", error)
            return
        self.last_send = now; self.last_light = light; self.last_group = group
        self.pub.publish(String("%s group=%s status=%d distance=%.1fm next=%.1fs" %
                                (light, ",".join(group), status, distance,
                                 max((self.next_switch-now).to_sec(), 0.0))))


if __name__ == "__main__":
    PrivilegedTrafficManager(); rospy.spin()

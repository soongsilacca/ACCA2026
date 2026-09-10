#!/usr/bin/env python3
import json
import os
import threading

import numpy as np
import rospy
import yaml
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String


def inside_polygon(point, polygon):
    if len(polygon) < 3: return False
    x, y = point; inside = False
    for i in range(len(polygon)):
        x1, y1 = polygon[i - 1]; x2, y2 = polygon[i]
        if (y1 > y) != (y2 > y):
            cross = (x2 - x1) * (y - y1) / ((y2 - y1) or 1e-12) + x1
            if x < cross: inside = not inside
    return inside


class PlannerDispatcher:
    def __init__(self):
        rospy.init_node("teacher_planner_dispatcher")
        self.config_path = os.path.expanduser(rospy.get_param("~config"))
        self.link_file = os.path.expanduser(rospy.get_param("~link_file"))
        self.lock = threading.Lock(); self.position = None; self.config_mtime = None
        self.context = {}; self.active_name = None; self.active_since = rospy.Time(0)
        self.load_links(); self.load_config()
        self.planner_pub = rospy.Publisher("/teacher/active_planner", String, queue_size=1, latch=True)
        self.local_planner_pub = rospy.Publisher("/teacher/local_planner", String, queue_size=1, latch=True)
        self.controller_pub = rospy.Publisher("/teacher/controller", String, queue_size=1, latch=True)
        self.behavior_pub = rospy.Publisher("/teacher/behavior", String, queue_size=1, latch=True)
        self.zone_pub = rospy.Publisher("/teacher/active_zone", String, queue_size=1, latch=True)
        self.params_pub = rospy.Publisher("/teacher/planner_params", String, queue_size=1, latch=True)
        self.speed_pub = rospy.Publisher("/teacher/speed_limit_mps", Float32, queue_size=1, latch=True)
        self.transition_pub = rospy.Publisher("/teacher/transition", String, queue_size=1, latch=True)
        self.fallback_pub = rospy.Publisher("/teacher/fallback", String, queue_size=1, latch=True)
        rospy.Subscriber("/localization/kinematic_state", Odometry, self.odom_callback, queue_size=1)
        rospy.Subscriber("/teacher/context", String, self.context_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def load_links(self):
        with open(self.link_file, "r", encoding="utf-8") as stream: raw = json.load(stream)
        self.links = {}
        for index, link in enumerate(raw):
            points = np.asarray(link.get("points", []), dtype=float)
            if len(points): self.links[str(link.get("idx", link.get("id", index)))] = points[:, :2]

    def load_config(self):
        with open(self.config_path, "r", encoding="utf-8") as stream: config = yaml.safe_load(stream) or {}
        self.default = config.get("default_planner", "mgeo")
        self.zones = sorted(config.get("policies", config.get("zones", [])), key=lambda z: -int(z.get("priority", 0)))
        self.config_mtime = os.path.getmtime(self.config_path)

    def odom_callback(self, msg):
        with self.lock: self.position = np.asarray([msg.pose.pose.position.x, msg.pose.pose.position.y])

    def context_callback(self, msg):
        try: self.context = json.loads(msg.data)
        except ValueError: rospy.logwarn_throttle(2.0, "Invalid /teacher/context JSON")

    @staticmethod
    def distance_to_links(point, arrays):
        if not arrays: return float("inf")
        return min(float(np.min(np.linalg.norm(xy - point, axis=1))) for xy in arrays)

    def active_zone(self, point):
        for zone in self.zones:
            if not zone.get("enabled", True): continue
            condition = zone.get("condition", {})
            expression = condition.get("if", "always")
            if expression != "always" and not bool(self.context.get(expression, False)): continue
            polygon = zone.get("polygon", [])
            selected = [self.links[x] for x in map(str, zone.get("link_ids", [])) if x in self.links]
            trigger = zone.get("trigger")
            trigger_radius = float(zone.get("parameters", {}).get("trigger_radius_m", 2.0))
            trigger_active = trigger is not None and np.linalg.norm(point - np.asarray(trigger)) <= trigger_radius
            if inside_polygon(point, polygon) or self.distance_to_links(point, selected) <= 4.0 or trigger_active:
                return zone
        return None

    def timer_callback(self, _event):
        try:
            if os.path.getmtime(self.config_path) != self.config_mtime: self.load_config()
        except (OSError, yaml.YAMLError) as error: rospy.logwarn_throttle(2.0, "Teacher config reload failed: %s", error)
        with self.lock: point = None if self.position is None else self.position.copy()
        if point is None: return
        zone = self.active_zone(point)
        name = "default" if zone is None else zone.get("name", "policy")
        if name != self.active_name:
            self.active_name = name; self.active_since = rospy.Time.now()
        planner = self.default if zone is None else zone.get("global_planner", zone.get("planner", self.default))
        self.planner_pub.publish(String(planner)); self.zone_pub.publish(String(name))
        self.local_planner_pub.publish(String("none" if zone is None else zone.get("local_planner", "none")))
        self.controller_pub.publish(String("mpc" if zone is None else zone.get("controller", "mpc")))
        self.behavior_pub.publish(String("NORMAL" if zone is None else zone.get("behavior", "NORMAL")))
        self.params_pub.publish(String(json.dumps({} if zone is None else zone.get("parameters", {}))))
        speed_value = None if zone is None else zone.get("speed_mps", zone.get("speed_kph"))
        speed = -1.0 if speed_value is None else float(speed_value)
        self.speed_pub.publish(Float32(speed))
        self.transition_pub.publish(String(json.dumps({} if zone is None else zone.get("transition", {}))))
        self.fallback_pub.publish(String(json.dumps({} if zone is None else zone.get("fallback", {}))))


if __name__ == "__main__":
    PlannerDispatcher(); rospy.spin()

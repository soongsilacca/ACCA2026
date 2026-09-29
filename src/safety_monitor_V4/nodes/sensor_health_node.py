#!/usr/bin/env python3
"""6.2 센서 신호 감시 노드.

~signals 목록의 각 신호에 대해 수신 시각을 기록하고, ~rate 주기로 판정해
/safety/sensor_estop (std_msgs/Bool) 과 /safety/sensor_status (std_msgs/String) 를 발행한다.
시각은 time.monotonic()을 쓴다.
"""
import math
import time

import rospy
from std_msgs.msg import Bool, String

from safety_monitor.health.signal_watch import HealthMonitor, SignalWatch


def _finite(*vals):
    return all(math.isfinite(v) for v in vals)


def _valid_imu(m):
    return _finite(m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
                   m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z,
                   m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)


def _valid_cloud(m):
    return m.width * m.height > 0


def _valid_ego(m):
    return _finite(m.velocity.x, m.velocity.y, m.position.x, m.position.y, m.yaw)


def _valid_odometry(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    v = m.twist.twist.linear
    return _finite(p.x, p.y, p.z, q.x, q.y, q.z, q.w, v.x, v.y, v.z)


def _msg_class(kind):
    if kind == "imu":
        from sensor_msgs.msg import Imu
        return Imu, _valid_imu
    if kind == "pointcloud":
        from sensor_msgs.msg import PointCloud2
        return PointCloud2, _valid_cloud
    if kind == "ego_status":
        from morai_msgs.msg import EgoVehicleStatus
        return EgoVehicleStatus, _valid_ego
    if kind == "odometry":
        from nav_msgs.msg import Odometry
        return Odometry, _valid_odometry
    raise ValueError("unknown signal type: %s" % kind)


class SensorHealthNode:
    def __init__(self):
        release = float(rospy.get_param("~release_time", 0.68))
        signals = rospy.get_param("~signals")
        watches = []
        for name, cfg in signals.items():
            w = SignalWatch(name, float(cfg["rate_hz"]), float(cfg["allowed_missing"]), release)
            cls, valid = _msg_class(cfg["type"])
            rospy.Subscriber(cfg["topic"], cls, self._make_cb(w, valid), queue_size=1,
                             buff_size=2 ** 24)
            watches.append(w)
            rospy.loginfo("watch %s: %s timeout=%.3fs", name, cfg["topic"], w.timeout)
        self.monitor = HealthMonitor(watches)
        self.pub_estop = rospy.Publisher("/safety/sensor_estop", Bool, queue_size=1)
        self.pub_status = rospy.Publisher("/safety/sensor_status", String, queue_size=1)
        rate = float(rospy.get_param("~rate", 50.0))
        self.timer = rospy.Timer(rospy.Duration(1.0 / rate), self._tick)

    @staticmethod
    def _make_cb(watch, valid):
        def cb(msg):
            try:
                ok = bool(valid(msg))
            except Exception:
                ok = False
            watch.on_message(time.monotonic(), ok)
        return cb

    def _tick(self, _evt):
        estop, reason, states = self.monitor.evaluate(time.monotonic())
        self.pub_estop.publish(Bool(data=estop))
        self.pub_status.publish(String(data="estop=%d reason=%s %s" % (
            estop, reason, " ".join("%s=%s" % kv for kv in sorted(states.items())))))


def main():
    rospy.init_node("sensor_health_monitor")
    SensorHealthNode()
    rospy.spin()


if __name__ == "__main__":
    main()

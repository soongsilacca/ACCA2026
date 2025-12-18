#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import sqlite3
import sys
from typing import Optional

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
from tf_transformations import quaternion_from_euler


class DB_READER(Node):
    def __init__(self, input_file: str, table: str, frame: str):
        super().__init__("db_reader")
        qos_profile = QoSProfile(depth=10)
        self.pub_path = self.create_publisher(Path, "db_path", qos_profile)
        self.pub_markers = self.create_publisher(MarkerArray, "db_markers", qos_profile)
        self.db_read(input_file, table, frame)

    def db_read(self, file_path: str, table: str, frame: str):
        conn = sqlite3.connect(file_path)
        cur = conn.cursor()

        cur.execute(
            f"""
            SELECT path_id, idx, x, y, yaw, speed
            FROM {table}
            ORDER BY idx ASC
        """
        )
        rows = cur.fetchall()

        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame

        markers = MarkerArray()
        now = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.header.frame_id = frame
        delete_all.header.stamp = now
        delete_all.ns = "db_visual"
        delete_all.id = 0
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        last_path_id: Optional[str] = None
        marker_id = 1

        for i, (path_id, idx, x, y, yaw, speed) in enumerate(rows):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            path.poses.append(pose)

            if last_path_id is None or path_id != last_path_id:
                m = Marker()
                m.header.frame_id = frame
                m.header.stamp = now
                m.ns = "db_visual/path_change"
                m.id = marker_id
                marker_id += 1
                m.type = Marker.SPHERE
                m.action = Marker.ADD
                m.pose.position.x = float(x)
                m.pose.position.y = float(y)
                m.pose.position.z = 0.15
                m.pose.orientation.w = 1.0
                m.scale.x = 0.45
                m.scale.y = 0.45
                m.scale.z = 0.45
                m.color.r = 1.0
                m.color.g = 0.3
                m.color.b = 0.0
                m.color.a = 0.9
                m.lifetime.sec = 0
                markers.markers.append(m)

                t = Marker()
                t.header.frame_id = frame
                t.header.stamp = now
                t.ns = "db_visual/path_change_text"
                t.id = marker_id
                marker_id += 1
                t.type = Marker.TEXT_VIEW_FACING
                t.action = Marker.ADD
                t.pose.position.x = float(x)
                t.pose.position.y = float(y)
                t.pose.position.z = 0.6
                t.pose.orientation.w = 1.0
                t.scale.z = 0.35
                t.color.r = 1.0
                t.color.g = 0.8
                t.color.b = 0.0
                t.color.a = 1.0
                t.text = f"path_id: {path_id}"
                t.lifetime.sec = 0
                markers.markers.append(t)

                last_path_id = path_id

            # speed 텍스트는 모든 포인트마다 찍으면 빽빽하므로 샘플링 적용 (예: 5번째마다)
            if i % 5 == 0:
                spd_val = 0.0 if speed is None else float(speed)
                s = Marker()
                s.header.frame_id = frame
                s.header.stamp = now
                s.ns = "db_visual/speed"
                s.id = marker_id
                marker_id += 1
                s.type = Marker.TEXT_VIEW_FACING
                s.action = Marker.ADD
                s.pose.position.x = float(x) + 0.2
                s.pose.position.y = float(y) + 0.2
                s.pose.position.z = 0.4
                s.pose.orientation.w = 1.0
                s.scale.z = 0.25
                s.color.r = 0.1
                s.color.g = 0.9
                s.color.b = 0.1
                s.color.a = 0.95
                s.text = f"{spd_val:.2f} km/h"
                s.lifetime.sec = 0
                markers.markers.append(s)

        self.pub_path.publish(path)
        self.pub_markers.publish(markers)
        conn.close()


def main(args=sys.argv):
    rclpy.init(args=args)
    if len(args) < 4:
        print("Usage: ros2 run <pkg> db_read_with_markers <DB_FILE> <TABLE_NAME> <frame>")
        return
    input_file = args[1]
    table = args[2]
    frame = args[3]
    node = DB_READER(input_file, table, frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

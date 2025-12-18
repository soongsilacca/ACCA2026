#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default

import numpy as np
import math as m
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline

from geometry_msgs.msg import PoseWithCovarianceStamped

from visualization_msgs.msg import MarkerArray, Marker 

from tf_transformations import *

from DB import DB


class DBWRITE(Node):
    def __init__(self):
        super().__init__("dbwrite")
        
        self.sub_domain = self.create_subscription(
            PoseWithCovarianceStamped, "initialpose", self.callback_initpose, qos_profile_system_default
        )

        self.pub_marker_array = self.create_publisher(
                        MarkerArray, "domain",  qos_profile_system_default
        )

        self.marker_timer = self.create_timer(0.1,self.domain_for_visu)
        
        self.path_x = []
        self.path_y = []
        self.path = []

        self.distance = 0.0
        self.ds = 0.1
        
        self.db =DB('B1.db')

        a = DB("B1_1.db")
        row = a.read_from_idx_to_path(100)
        print(row)
        
        row = np.array(row)
        self.path_x = row[:,0]
        self.path_y = row[:,1]
        self.interpolate_path()

    def callback_initpose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.path_x.append(x)
        self.path_y.append(y)
        if len(self.path_x) >= 3:  # 최소 3개의 포인트 필요
            self.interpolate_path()
    
    def interpolate_path(self):
        x = np.array(self.path_x)
        y = np.array(self.path_y)
        dx_ = np.diff(x)
        dy_ = np.diff(y)
        ds = np.sqrt(dx_**2 + dy_**2)
        s = np.concatenate([[0], np.cumsum(ds)])
        try:
            cs_x = CubicSpline(s, x, bc_type="natural")
            cs_y = CubicSpline(s, y, bc_type="natural")
            
            self.narrow = int(s[-1] / self.ds)

            s_new = np.linspace(s[0], s[-1], self.narrow)
            
            self.get_logger().info(f"path_idx_num: {self.narrow}.")
            x_new = cs_x(s_new)
            y_new = cs_y(s_new)

            dx_new = cs_x(s_new, 1)
            dy_new = cs_y(s_new, 1)

            yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]

            self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))
            
            # plt.figure()
            # plt.plot(x, y, 'o', label='data points')
            # plt.plot(x_new, y_new, '-', label='cubic spline')
            # plt.legend(loc='best')
            # plt.title('Cubic Spline')
            # plt.show()
            
            self.write_db()
        except Exception as e:
            self.get_logger().error(f"An error occurred during spline interpolation: {e}")
            
    def write_db(self):
        self.db.write_db_Path(self.path)

    def domain_for_visu(self):
        marker_array = MarkerArray()
        
        for i ,  point in enumerate(zip(self.path_x,self.path_y)):

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "global_path"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0

            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)
        self.pub_marker_array.publish(marker_array)

def main(args=None):
    
    rclpy.init(args=args)
    node = DBWRITE()
    print(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

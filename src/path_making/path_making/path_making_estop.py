import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,qos_profile_system_default

import numpy as np
import math as m
from scipy.interpolate import CubicSpline

from erp42_msgs.msg import SerialFeedBack, ControlMessage

# import DB as db
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray, Marker

import sys


class Feedback_from_erp():
    def __init__(self,node):
        self.node = node
        
        self.feedback = self.node.create_subscription(
            SerialFeedBack,"erp42_feedback",self.update,qos_profile_system_default
        )
        self.estop = 0
        self.estop_position = []
        self.idx = 1
    def update(self, msg):
        self.estop = msg.estop
        print(self.estop_position)
        
        
        
class Estop_collector(Node):
    def __init__(self):
        super().__init__("Estop_collector")
        
        self.sub_local = self.create_subscription(
            Odometry, "localization/kinematic_state", self.callback_local, qos_profile_system_default
        )

        self.pub_marker_array_ = self.create_publisher(
                    MarkerArray,"estop_point",qos_profile_system_default
                )
          
        self.db = db("0827_ssupark_ys.db")
        self.feedback = Feedback_from_erp(self)
        self.count = 0
        self.marker_id = 0
        
        
    def callback_local(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # a = msg.pose.covariance
        # cov = a[0] + a[7]
        # p1 = (x, y)
        if self.feedback.estop == 1 and  self.count == 0:
                    self.feedback.estop_position.append((self.feedback.idx,x,y))
                    self.feedback.idx += 1
                    self.count = 1
                    
        if self.feedback.estop == 0:
            self.domain_for_visu()
            self.count = 0
        
        # if not self.euclidean_duplicate(p1):
        #     self.path_x.append(p1[0])
        #     self.path_y.append(p1[1])
            # self.path_cov.append(cov)
        # else:
            
        #     euc_dup_list = sorted(self.euclidean_list, key=lambda point: point[2])
        #     low_cov_point = euc_dup_list[0]
            
        #     if low_cov_point == p1:
        #         self.update_path_with_low_cov_point(euc_dup_list)
        #     self.euclidean_list.clear()
            
        # if len(self.path_x) >= 3:
        #     self.interpolate_path()
    


    # def euclidean_duplicate(self, p1):
    #     threshold = 1.0
    #     # a=0
    #     for x, y in zip(self.path_x, self.path_y):
    #         distance = m.sqrt((p1[0] - x) ** 2 + (p1[1] - y) ** 2)

    #         if distance <= threshold:
    #         #     a=1
    #         #     self.euclidean_list.append((x, y, cov))
    #         # if a==1:
    #             # self.euclidean_list.append(p1)    
    #             return True
    #     return False

    # def update_path_with_low_cov_point(self, euc_dup_list):
    #     low_cov_point = euc_dup_list[0]
    #     for x, y, cov in euc_dup_list[1:]:
    #         if x in self.path_x:
    #             self.path_x.insert(self.path_x.index(x),low_cov_point[0])
    #             self.path_x.remove(x)
    #         if y in self.path_y:
    #             self.path_y.insert(self.path_y.index(y),low_cov_point[1])
    #             self.path_y.remove(y)
    #         if cov in self.path_cov:
    #             self.path_cov.insert(self.path_cov.index(cov),low_cov_point[2])
    #             self.path_cov.remove(cov)

    # def interpolate_path(self):
    #     x = np.array(self.path_x)
    #     y = np.array(self.path_y)
    #     dx_ = np.diff(x)
    #     dy_ = np.diff(y)
    #     ds = np.sqrt(dx_**2 + dy_**2)
    #     s = np.concatenate([[0], np.cumsum(ds)])
    #     try:
    #         cs_x = CubicSpline(s, x, bc_type="natural")
    #         cs_y = CubicSpline(s, y, bc_type="natural")
            
    #         self.narrow = int(s[-1] / self.ds)

    #         s_new = np.linspace(s[0], s[-1], self.narrow)
            
    #         self.get_logger().info(f"path_idx_num: {self.narrow}.\n")
            
    #         x_new = cs_x(s_new)
    #         y_new = cs_y(s_new)

    #         dx_new = cs_x(s_new, 1)
    #         dy_new = cs_y(s_new, 1)

    #         yaw_new = [m.atan2(dy, dx) for dy, dx in zip(dy_new, dx_new)]

    #         self.path = list(zip(x_new.tolist(), y_new.tolist(), yaw_new))

    #         self.write_db()
    #     except Exception as e:
    #         self.get_logger().error(f"An error occurred during spline interpolation: {e}")
            
                
    # def write_db(self):
    #     self.db.write_db_Path(self.path)
    def domain_for_visu(self):
        marker_array_ = MarkerArray()
        
        for  point in self.feedback.estop_position:

            marker_ = Marker()
            marker_.header.frame_id = "map"
            marker_.header.stamp = self.get_clock().now().to_msg()
            marker_.ns = "global_path"
            marker_.id = self.marker_id
            marker_.type = Marker.SPHERE
            marker_.action = Marker.ADD

            marker_.pose.position.x = point[1]
            marker_.pose.position.y = point[2]
            marker_.pose.position.z = 0.0

            marker_.pose.orientation.x = 0.0
            marker_.pose.orientation.y = 0.0
            marker_.pose.orientation.z = 0.0
            marker_.pose.orientation.w = 1.0

            marker_.scale.x = 2.
            marker_.scale.y = 2.
            marker_.scale.z = 2.

            marker_.color.r = 0.0
            marker_.color.g = 0.0
            marker_.color.b = 1.0
            marker_.color.a = 1.0
            self.marker_id += 1

            marker_array_.markers.append(marker_)
        self.pub_marker_array_.publish(marker_array_)


def main(args=None):
    rclpy.init(args=args)
    node = Estop_collector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

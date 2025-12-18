from DB import DB
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,qos_profile_system_default

import rclpy
from rclpy import node
from nav_msgs.msg import Path

from tf_transformations import *
class Estop_collector(Node):
    def __init__(self):
        super().__init__("Path_collector")
        
        self.sub_local = self.create_subscription(
            Path, "delivery_path", self.callback_path, qos_profile_system_default
        )
        self.db = DB("B1_test.db")
        self.path = []
    
    def callback_path(self,msg):
        for pose in msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            
            quaternion_x = pose.pose.orientation.x
            quaternion_y = pose.pose.orientation.y
            quaternion_z = pose.pose.orientation.z
            quaternion_w = pose.pose.orientation.w
            _,_,yaw = euler_from_quaternion([quaternion_x,quaternion_y,quaternion_z,quaternion_w])
            self.path.append((x,y,yaw))

        self.write_db()
    def write_db(self):
        self.db.write_db_Path(self.path)


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

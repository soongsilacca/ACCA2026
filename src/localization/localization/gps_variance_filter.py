import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from nav_msgs.msg import Odometry
import numpy as np


class Chage_gpsvar(Node):
    def __init__(self):
        super().__init__("change_gpsvar")

        self.sub = self.create_subscription(
            Odometry, "odometry/gps", self.callback_gps, qos_profile_system_default
        )

        self.pub = self.create_publisher(
            Odometry, "odometry/gps_var", qos_profile_system_default
        )

    def callback_gps(self, msg: Odometry):
        x_cov = msg.pose.covariance[0]
        y_cov = msg.pose.covariance[7]

        # the var in tunnel( 100 mm 오차 0.1m 오차 밖에 안되네 300배 해주자)
        #           covariance:
        #   - 0.012100000000000001
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.012100000000000001
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0
        #   - 0.0

        cov = np.array(msg.pose.covariance).reshape((6, 6))
        if np.linalg.norm(cov) > 0.03:
            cov *= 10.0

        new_msg = msg
        new_msg.pose.covariance = cov.flatten().tolist()

        self.pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Chage_gpsvar()
    rclpy.spin(node)
    rclpy.shutdown()
    node.destroy_node()


if __name__ == "__main__":
    main()

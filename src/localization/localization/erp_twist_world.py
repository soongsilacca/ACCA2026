import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import TwistWithCovarianceStamped, Quaternion
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from tf_transformations import euler_from_quaternion
from rclpy.clock import Clock


class ErpTwist(Node):
    def __init__(self):
        super().__init__("erp_twist")
        qos_profile = QoSProfile(depth=10)

        self.sub_erp = self.create_subscription(
            SerialFeedBack, "erp42_feedback", self.callback_erp, qos_profile
        )
        self.sub_imu = self.create_subscription(
            Imu, "imu/data", self.callback_imu, qos_profile_sensor_data
        )

        self.pub = self.create_publisher(
            TwistWithCovarianceStamped, "erp42/twist/world", qos_profile
        )

        self.yaw = None
        self.time = None
        self.header = Header()

    def callback_erp(self, msg):
        if self.yaw is not None:
            yaw = self.yaw
            header = self.header
            header.stamp = Clock().now().to_msg()
            header.frame_id = "base_link"
            gear = msg.gear
            if gear == 2:
                v = msg.speed
            else:
                v = (-1) * msg.speed
            self.publish_twist(v, yaw, header)

    def callback_imu(self, msg):
        quarternion = msg.orientation
        # self.time =
        _, _, self.yaw = euler_from_quaternion(
            [quarternion.x, quarternion.y, quarternion.z, quarternion.w]
        )
        # self.yaw = 0.0

    def publish_twist(self, v, yaw, header):
        data = TwistWithCovarianceStamped()

        data.header = header

        data.twist.twist.linear.x = v * np.cos(yaw)
        data.twist.twist.linear.y = v * np.sin(yaw)
        data.twist.twist.linear.z = 0.0

        data.twist.twist.angular.x = 0.0
        data.twist.twist.angular.y = 0.0
        data.twist.twist.angular.z = 0.0

        data.twist.covariance = [
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.1,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.pub.publish(data)


def main(args=None):
    rclpy.init(args=args)
    node = ErpTwist()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

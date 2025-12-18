import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_system_default
from nav_msgs.msg import Odometry


class DummyPublisher(Node):
    def __init__(self):
        super().__init__("dummy_publisher")

        self.declare_parameter("gps_jamming_mode", False)

        self.publisher_ = self.create_publisher(
            Odometry,
            "odometry/gps",
            qos_profile_system_default,
        )
        self.timer = self.create_timer(1 / 10, self.publish_dummy_message)

    def publish_dummy_message(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        if self.param_get():
            msg.pose.covariance[0] = 9.99999999999e11
            msg.pose.covariance[7] = 9.99999999999e11
            msg.pose.covariance[14] = 9.99999999999e11
            msg.pose.covariance[21] = 9.99999999999e11
            msg.pose.covariance[28] = 9.99999999999e11
            msg.pose.covariance[35] = 9.99999999999e11

        else:
            msg.pose.covariance[0] = 0.000196
            msg.pose.covariance[7] = 0.000196
            msg.pose.covariance[14] = 0.000196
            msg.pose.covariance[21] = 0.000196
            msg.pose.covariance[28] = 0.000196
            msg.pose.covariance[35] = 0.000196

        self.publisher_.publish(msg)
        self.get_logger().info("Published dummy Odometry message")

    def param_get(self):
        return self.get_parameter("gps_jamming_mode").get_parameter_value().bool_value


def main(args=None):
    rclpy.init(args=args)
    dummy_publisher = DummyPublisher()
    rclpy.spin(dummy_publisher)
    dummy_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

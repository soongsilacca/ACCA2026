import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from geometry_msgs.msg import TwistWithCovarianceStamped

from tf_transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import tf2_ros


class Imu_Encoder(Node):
    def __init__(self):
        super().__init__("imu_encoder_odometry")

        self.declare_parameter("imu_topic", "steer_yaw")
        self.declare_parameter("odom_topic", "/odometry/wheel")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("logging", True)

        imu_topic = self.get_parameter("imu_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.logging = self.get_parameter("logging").value

        # subscriber
        self.imu_sub = self.create_subscription(
            Quaternion,
            imu_topic,
            self.imu_callback,
            qos_profile=qos_profile_sensor_data,
        )
        self.erp_twist_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            "erp42/twist/world",
            self.callback_erp_twist,
            qos_profile=qos_profile_sensor_data,
        )

        # publisher(odometry)
        self.odom_pub = self.create_publisher(
            Odometry, odom_topic, qos_profile=qos_profile_system_default
        )

        # tf
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.x = 1.86846
        self.y = 4.6486
        self.yaw = 0.0
        self.init_yaw = 0.0
        self.v_x = 0.0
        self.v_y = 0.0
        self.last_time = None

    def imu_callback(self, msg):
        q = [msg.x, msg.y, msg.z, msg.w]
        print(q)
        euler = euler_from_quaternion(q)

        # if self.init_yaw == 0.0:
        #     self.init_yaw = euler[2]

        # self.yaw = euler[2] - self.init_yaw
        self.yaw = euler[2]

    def callback_erp_twist(self, msg):
        # print("ongoing")
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.last_time is None:
            self.last_time = current_time

        dt = current_time - self.last_time
        self.last_time = current_time

        self.v_x = msg.twist.twist.linear.x
        self.v_y = msg.twist.twist.linear.y

        self.x = self.x + float(self.v_x) * dt
        self.y = self.y + float(self.v_y) * dt

        if self.logging is True:
            self.get_logger().info(f"position & time: {self.x}    {self.y}    {dt}")
            self.get_logger().info(
                f"velocity & orientation: {self.v_x}  {self.v_y}  {self.yaw}"
            )

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = self.frame_id
        msg.child_frame_id = self.child_frame_id

        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y

        q = quaternion_from_euler(0, 0, self.yaw)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(msg)

        # self.publish_tf(self.x, self.y, q)

    def publish_tf(self, x, y, q):
        tf_msg = tf2_ros.TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()

        tf_msg.header.frame_id = "odom"
        tf_msg.child_frame_id = "base_link"

        tf_msg.transform.translation.x = x
        tf_msg.transform.translation.y = y
        tf_msg.transform.translation.z = 0.0

        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)

    node = Imu_Encoder()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

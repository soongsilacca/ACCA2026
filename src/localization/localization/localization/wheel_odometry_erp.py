import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from erp42_msgs.msg import SerialFeedBack
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster


class LowPassFilter:
    def __init__(self, alpha):
        self.alpha = alpha
        self.prev_val = None

    def filter(self, val):
        if self.prev_val is None:
            self.prev_val = val
            return val
        filtered = self.alpha * val + (1.0 - self.alpha) * self.prev_val
        self.prev_val = filtered
        return filtered


class ERP42Odometry(Node):
    def __init__(self):
        super().__init__("erp42_odometry_node")

        self.declare_parameter("wheelbase", 1.04)
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_tf", False)

        self.declare_parameter("lpf_alpha_speed", 0.7)
        self.declare_parameter("lpf_alpha_steer", 0.7)

        # 단순 선형 매핑 파라미터 (좌/우 gain)
        self.declare_parameter("k_left", 0.50)
        self.declare_parameter("k_right", 0.93)
        
        # Ackermann correction parameter (encoder on front-right wheel)
        self.declare_parameter("track_width", 0.98)  # Distance between left and right wheels

        self.L = self.get_parameter("wheelbase").value
        self.frame_id = self.get_parameter("frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value
        self.publish_tf_flag = self.get_parameter("publish_tf").value

        self.k_left = self.get_parameter("k_left").value
        self.k_right = self.get_parameter("k_right").value
        self.W = self.get_parameter("track_width").value  # Track width

        self.lpf_speed = LowPassFilter(self.get_parameter("lpf_alpha_speed").value)
        self.lpf_steer = LowPassFilter(self.get_parameter("lpf_alpha_steer").value)

        self.sub_feedback = self.create_subscription(
            SerialFeedBack, "/erp42_feedback", self.feedback_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/wheel/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # Always start at 0, EKF will handle offset with IMU

        self.last_time = self.get_clock().now()

        self.get_logger().info(f"ERP42 Odometry Started. Wheelbase={self.L}m, TrackWidth={self.W}m (yaw offset handled by EKF)")

    def feedback_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            return

        raw_speed = msg.speed
        raw_steer = msg.steer

        if msg.gear == 0:
            raw_speed = -abs(raw_speed)
        elif msg.gear == 2:
            raw_speed = abs(raw_speed)
        else:
            raw_speed = 0.0

        v = self.lpf_speed.filter(raw_speed)
        steer_in = self.lpf_steer.filter(raw_steer)

        if steer_in > 0:
            delta = steer_in * self.k_left
        else:
            delta = steer_in * self.k_right



        if abs(v) < 0.01:
            omega = 0.0
            v = 0.0
        else:
            omega = (v / self.L) * math.tan(delta)

        self.yaw += omega * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt

        self.publish_odometry(v, omega, now)
        self.last_time = now

    def publish_odometry(self, v, omega, stamp):
        q = self.euler_to_quaternion(0, 0, self.yaw)

        if self.publish_tf_flag:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.frame_id
            t.child_frame_id = self.child_frame_id
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation = q
            self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        odom.pose.covariance = [
            500.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 500.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.4
        ]

        odom.twist.covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 10.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1e3, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1e3, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1e3, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.5
        ]
        self.pub_odom.publish(odom)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = ERP42Odometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

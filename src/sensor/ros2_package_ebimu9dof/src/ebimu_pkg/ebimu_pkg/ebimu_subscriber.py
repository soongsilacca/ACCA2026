import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import math


def euler_to_quaternion(roll, pitch, yaw):
    """
    Roll, Pitch, Yaw(rad)를 Quaternion(x,y,z,w)으로 변환
    """
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


def data_parser(msg_data):
    words = msg_data.split(",")
    if -1 < words[0].find("*"):
        words[0] = words[0].replace("*", "")
        return list(map(float, words))  # float 리스트 반환


class EbimuSubscriber(Node):

    def __init__(self):
        super().__init__("ebimu_subscriber")
        qos_profile = QoSProfile(depth=10)

        # Subscriber
        self.subscription = self.create_subscription(
            String, "ebimu_data", self.callback, qos_profile
        )

        # Publisher
        self.publisher = self.create_publisher(Imu, "imu/data_raw", qos_profile)

        self.publisher_yaw = self.create_publisher(Float64, "imu/yaw", qos_profile)
    def callback(self, msg):
        imu_data = data_parser(msg.data)
        if imu_data is None:
            return

        # roll, pitch, yaw, w_x, w_y, w_z, a_x, a_y, a_z
        roll, pitch, yaw = imu_data[0:3]
        roll  = -roll
        pitch = -pitch
        yaw   = -yaw
        self.publisher_yaw.publish(Float64(data=yaw))

        # 🔹 x, y 바꿈
        w_y, w_x, w_z = imu_data[3:6]
        a_y, a_x, a_z = imu_data[6:9]

        imu_msg = Imu()

        # Header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Orientation
        imu_msg.orientation = euler_to_quaternion(
            math.radians(roll), math.radians(pitch), math.radians(yaw)
        )

        # Angular velocity
        imu_msg.angular_velocity.x = w_x
        imu_msg.angular_velocity.y = w_y
        imu_msg.angular_velocity.z = w_z

        # Linear acceleration
        imu_msg.linear_acceleration.x = a_x
        imu_msg.linear_acceleration.y = a_y
        imu_msg.linear_acceleration.z = a_z      

        # Covariance 설정 (기존 그대로)
        orientation_stddev= [0.05, 0.05, 0.05]
        angular_velocity_stddev= [0.01, 0.01, 0.01]
        linear_acceleration_stddev= [0.0001, 0.0001, 0.00015]

        imu_msg.orientation_covariance[0] = orientation_stddev[0] ** 2
        imu_msg.orientation_covariance[4] = orientation_stddev[1] ** 2
        imu_msg.orientation_covariance[8] = orientation_stddev[2] ** 2
        
        imu_msg.angular_velocity_covariance[0] = angular_velocity_stddev[0] ** 2
        imu_msg.angular_velocity_covariance[4] = angular_velocity_stddev[1] ** 2
        imu_msg.angular_velocity_covariance[8] = angular_velocity_stddev[2] ** 2

        imu_msg.linear_acceleration_covariance[0] = linear_acceleration_stddev[0] ** 2
        imu_msg.linear_acceleration_covariance[4] = linear_acceleration_stddev[1] ** 2
        imu_msg.linear_acceleration_covariance[8] = linear_acceleration_stddev[2] ** 2

        # Publish
        self.publisher.publish(imu_msg)

        # Debug 출력
        self.get_logger().info(
            f"Published IMU: RPY=({roll:.2f},{pitch:.2f},{yaw:.2f}) "
            f"w=({w_x:.3f},{w_y:.3f},{w_z:.3f}) "
            f"a=({a_x:.3f},{a_y:.3f},{a_z:.3f})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EbimuSubscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

# logger.py
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from erp42_msgs.msg import SerialFeedBack
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion
import csv
import os
from datetime import datetime

class DataLogger(Node):
    def __init__(self):
        super().__init__('bicycle_data_logger')

        # 로그 파일 경로
        now = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_path = os.path.join(os.getcwd(), f'bicycle_log_{now}.csv')
        self.csv_file = open(self.log_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'speed', 'steer', 'yaw'])

        self.yaw = 0.0
        self.create_subscription(
            SerialFeedBack, "erp42_feedback", self.feedback_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Quaternion, "steer_yaw", self.yaw_callback, qos_profile_sensor_data
        )

    def feedback_callback(self, msg):
        t = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([t, msg.speed, msg.steer, self.yaw])

    def yaw_callback(self, msg):
        q = [msg.x, msg.y, msg.z, msg.w]
        _, _, self.yaw = euler_from_quaternion(q)

def main(args=None):
    rclpy.init(args=args)
    node = DataLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.csv_file.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

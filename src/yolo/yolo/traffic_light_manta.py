#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy  # ✅ QoS 관련 import 추가
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import time
from ultralytics import YOLO

class YOLOv8InferenceNode(Node):
    def __init__(self):
        super().__init__('yolov8_manta_node')

        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/traffic_light.pt')
        self.bridge = CvBridge()

        # ✅ RELIABLE QoS 설정
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )

        self.subscription = self.create_subscription(
            Image,
            '/image_color',
            self.listener_callback,
            qos_profile
        )

        cv2.startWindowThread()
        self.get_logger().info("YOLOv8 Inference Node is running (using ROS image topic)...")

    def listener_callback(self, msg):
        print("🟢 callback 진입")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            start = time.perf_counter()
            results = self.model(cv_image)[0]
            latency = (time.perf_counter() - start) * 1000.0
            self.get_logger().info(f"Inference latency: {latency:.2f} ms")

            annotated = results.plot()
            cv2.imshow("YOLOv8 + MantaCam", annotated)
            key = cv2.waitKey(1)
            if key == 27:
                self.get_logger().info("ESC pressed. Shutting down...")
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Callback error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv8InferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

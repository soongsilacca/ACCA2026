#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
from ultralytics import YOLO
import time

from std_msgs.msg import String  # ✅ string 메시지 사용

class YOLOv8InferenceNode(Node):
    def __init__(self):
        super().__init__('yolov8_inference_node')

        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/traffic_light.pt')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            'traffic_light_cam',
            self.listener_callback,
            10
        )

        # ✅ String 퍼블리셔
        self.publisher = self.create_publisher(String, '/yolo/class_name', 10)

        # 무시할 클래스 이름
        self.ignore_classes = {"unknown"}  

        self.get_logger().info("YOLOv8 Inference Node is running...")

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            start = time.perf_counter()
            results = self.model(cv_image)[0]
            end = time.perf_counter()
            latency_ms = (end - start) * 1000.0
            self.get_logger().info(f"Inference time: {latency_ms:.2f} ms")

            annotated = results.plot()
            cv2.imshow("YOLOv8 Detection", annotated)
            cv2.waitKey(1)

            # 후보 중 가장 큰 박스만 선택
            max_area = 0
            selected_class = None

            for box in results.boxes:
                xyxy = box.xyxy.cpu().numpy()[0]  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = map(int, xyxy)
                width = x2 - x1
                height = y2 - y1
                area = width * height

                class_id = int(box.cls.cpu().numpy()[0])
                class_name = self.model.names[class_id]

                # 무시할 클래스 건너뛰기
                if class_name in self.ignore_classes:
                    continue

                if width >= height:
                    continue

                if area > max_area:
                    max_area = area
                    selected_class = class_name

            # ✅ 가장 큰 박스의 class 이름만 발행
            if selected_class:
                msg = String()
                msg.data = selected_class
                self.publisher.publish(msg)
                self.get_logger().info(
                    f"Published class name='{selected_class}', area={max_area}"
                )
            else:
                self.get_logger().info("No valid class to publish.")

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
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

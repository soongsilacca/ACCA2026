#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Header

import cv2
import numpy as np
from ultralytics import YOLO
import time

from yolo_msg.msg import BoundingBox, BoundingBoxArray

##traffic sign
class YOLOv8InferenceNode(Node):
    def __init__(self):
        super().__init__('yolov8_traffic_sign_node')

        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/1.pt')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera2/image_raw',
            self.listener_callback,
            10
        )

        self.publisher = self.create_publisher(BoundingBox, 'yolo/detections', 10)

        self.get_logger().info("YOLOv8 Inference Node is running...")

##traffic light
# class YOLOv8InferenceNode(Node):
#     def __init__(self):
#         super().__init__('yolov8_inference_node')

#         self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/traffic_light.pt')
#         self.bridge = CvBridge()

#         self.subscription = self.create_subscription(
#             Image,
#             '/camera2/image_raw',
#             self.listener_callback,
#             10
#         )

#         self.publisher = self.create_publisher(BoundingBox, 'yolo/detections', 10)

#         self.get_logger().info("YOLOv8 Inference Node is running...")

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

            # 바운딩 박스 추출 및 메시지 발행
            for box in results.boxes:
                xyxy = box.xyxy.cpu().numpy()[0]  # [x1, y1, x2, y2]
                x1, y1, x2, y2 = map(int, xyxy)
                width = x2 - x1
                height = y2 - y1
                center_x = (x1 + x2) / 2.0
                center_y = (y1 + y2) / 2.0

                confidence = float(box.conf.cpu().numpy()[0])  # confidence
                class_id = int(box.cls.cpu().numpy()[0])
                class_name = self.model.names[class_id]

                # 메시지 생성
                msg_out = BoundingBox()
                msg_out.x = x1
                msg_out.y = y1
                msg_out.width = width
                msg_out.height = height
                msg_out.center_x = float(center_x)
                msg_out.center_y = float(center_y)
                msg_out.confidence = confidence
                msg_out.class_name = class_name

                self.publisher.publish(msg_out)

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

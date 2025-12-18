
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

# YOLO 모델 경로
MODEL_PATH = '/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/traffic_light.pt'
model = YOLO(MODEL_PATH)
CLASS_NAMES = model.names  # 예: {0: 'red', 1: 'green', ...}

class TrafficLightDetector7Class(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_class')
        self.subscription = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)
        self.signal_pub = self.create_publisher(String, '/traffic_light_signal', 10)
        self.bridge = CvBridge()
        self.get_logger().info("🚦 YOLO Traffic Light Detector Started")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # YOLO 추론
        results = model(frame)
        detections = results[0].boxes.data.cpu().numpy()

        # ✅ 가로형 신호등 + unknown 제외 + 가장 큰 ROI 찾기
        valid_detections = []
        for det in detections:
            x1, y1, x2, y2, conf, class_id = det
            x1, y1, x2, y2, class_id = map(int, [x1, y1, x2, y2, class_id])

            width = x2 - x1
            height = y2 - y1
            area = width * height

            # 가로형만
            if width <= height:
                continue

            class_name = CLASS_NAMES[class_id]

            # unknown 제외
            if class_name.lower() == "unknown":
                continue

            valid_detections.append((area, class_name))

        # 가장 큰 ROI 결과만 퍼블리시
        if valid_detections:
            largest = max(valid_detections, key=lambda x: x[0])
            _, class_name = largest
            msg = String()
            msg.data = class_name
            self.signal_pub.publish(msg)
            self.get_logger().info(f"Detected Signal: {class_name}")
        else:
            # 아무것도 없으면 "none" 발행
            msg = String()
            msg.data = "none"
            self.signal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrafficLightDetector7Class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

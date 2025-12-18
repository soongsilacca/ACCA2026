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

class YOLOv8InferenceNode(Node):
    def __init__(self):
        super().__init__('yolov8_logitech_node')

        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/traffic_light.pt')
        self.bridge = CvBridge()

        # self.subscription = self.create_subscription(
        #     Image,
        #     '/camera1/image_raw',
        #     self.listener_callback,
        #     10
        # )

        self.subscription = self.create_subscription(
            Image,
            'traffic_light_cam',
            self.listener_callback,
            10
        )

        # 개별 BoundingBox 퍼블리셔 (원 코드 유지)
        self.publisher = self.create_publisher(BoundingBox, 'yolo/detections', 10)

        self.get_logger().info("YOLOv8 Inference Node (Horizontal-only, hide 'unknown') is running...")

    def _get_class_name(self, class_id):
        """YOLO names 매핑을 안전하게 가져오기."""
        names = self.model.names
        if isinstance(names, (list, tuple)):
            if 0 <= class_id < len(names):
                return str(names[class_id])
            return 'unknown'
        # dict일 경우
        return str(names.get(class_id, 'unknown'))

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CVBridge error: {e}")
            return

        # YOLO 추론
        try:
            start = time.perf_counter()
            results = self.model(cv_image)[0]
            end = time.perf_counter()
        except Exception as e:
            self.get_logger().error(f"YOLO inference error: {e}")
            return

        latency_ms = (end - start) * 1000.0

        # 그리기용 이미지
        annotated = cv_image.copy()
        displayed_count = 0
        published_count = 0

        try:
            for box in results.boxes:
                # 좌표 추출: [x1, y1, x2, y2]
                xyxy = box.xyxy.cpu().numpy().astype(int).flatten()
                if xyxy.size != 4:
                    continue
                x1, y1, x2, y2 = xyxy.tolist()

                width = x2 - x1
                height = y2 - y1

                # ✅ 가로형만 처리
                if width <= height:
                    continue

                confidence = float(box.conf.cpu().numpy()[0])
                class_id = int(box.cls.cpu().numpy()[0])
                class_name = self._get_class_name(class_id)

                # 퍼블리시는 그대로 수행(요구사항: 화면 출력만 제외)
                msg_out = BoundingBox()
                msg_out.x = int(x1)
                msg_out.y = int(y1)
                msg_out.width = int(width)
                msg_out.height = int(height)
                msg_out.center_x = float(x1 + width / 2.0)
                msg_out.center_y = float(y1 + height / 2.0)
                msg_out.confidence = float(confidence)
                msg_out.class_name = str(class_name)
                self.publisher.publish(msg_out)
                published_count += 1

                # ❌ 화면 출력 제외: 클래스 이름이 'unknown'이면 그리지 않음
                if class_name.lower() == 'unknown':
                    continue

                # 화면 표시(known 클래스만)
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"{class_name} {confidence:.2f}"
                cv2.putText(annotated, label, (x1, max(y1 - 10, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                displayed_count += 1

        except Exception as e:
            self.get_logger().error(f"Post-processing error: {e}")

        self.get_logger().info(
            f"Inference time: {latency_ms:.2f} ms | published (horizontal): {published_count} | displayed: {displayed_count}"
        )

        # 결과 화면 표시 (unknown은 제외되어 그림)
        try:
            cv2.imshow("YOLOv8 Detection (unknown hidden)", annotated)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().debug(f"imshow skipped: {e}")

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
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass

if __name__ == '__main__':
    main()


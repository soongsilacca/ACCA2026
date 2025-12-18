#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from yolo_msg.msg import BoundingBox, BoundingBoxArray

from ultralytics import YOLO
import cv2
import numpy as np
from cv_bridge import CvBridge
import time


CLASS_COLOR = {
    "yellow": (0, 255, 255),
    "blue": (255, 0, 0)
}


class YoloSegNode(Node):
    def __init__(self):
        super().__init__('yolo_seg_node')
        self.bridge = CvBridge()

        # YOLOv8 세그멘테이션 모델 로드
        self.model = YOLO('/home/acca/acca_ws/src/ACCA_2025/src/yolo/models/cone.pt')  # pt 경로 수정 가능
        self.labels = self.model.names  # {0: 'yellow', 1: 'blue', ...}
        # ROS2 통신
        self.image_sub = self.create_subscription(Image, 'concated_cam', self.image_callback, 10)
        self.yellow_pub = self.create_publisher(PointStamped, 'yellow_cone', 10)
        self.blue_pub = self.create_publisher(PointStamped, 'blue_cone', 10)
        self.pixel_circle_pub = self.create_publisher(PointStamped, 'pixel_circle', 10)
        self.bbox_pub = self.create_publisher(BoundingBoxArray, 'bounding_box', 10)

        cv2.namedWindow("Detections", cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        print(self.labels)
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # print(frame)
        # cv2.imshow("Detections", frame)
        # cv2.waitKey(1)
        start = time.perf_counter()
        # 480 * 1920 320 * 1280
        # results = self.model.predict(frame, imgsz=(480,1920))[0]
        results = self.model.predict(frame, imgsz=1920)[0]
# 
        # results = self.model(frame)[0]
        # print(results.masks)
        end = time.perf_counter()
        latency = (end - start) * 1000
        self.get_logger().info(f"YOLOv8-seg Inference Time: {latency:.2f} ms")

        bbox_array = BoundingBoxArray()
        bbox_array.header.stamp = self.get_clock().now().to_msg()
        bbox_array.header.frame_id = "camera_map"

        if results.masks is None:
            self.get_logger().warn("No masks detected.")
            return

        masks = results.masks.data.cpu().numpy()  # shape: [N, H, W]
        boxes = results.boxes

        for i, box in enumerate(boxes):
            if i >= len(masks):
                self.get_logger().warn(f"No mask for box {i}, skipping.")
                continue

            cls_id = int(box.cls[0].item())
            class_name = self.labels[cls_id]
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy().astype(int)
            width, height = x2 - x1, y2 - y1

            # 마스크 중심점 계산
            mask = masks[i]
            center = self.get_mask_center(mask, i)
            if center is None:
                continue
            cx, cy = center

            # 메시지 생성
            bbox = BoundingBox()
            bbox.x = int(x1)
            bbox.y = int(y1)
            bbox.width = int(width)
            bbox.height = int(height)
            bbox.class_name = class_name
            bbox.confidence = conf
            bbox.center_x = float(cx)
            bbox.center_y = float(cy)
            bbox.pixel_center_x = float(cx)
            bbox.pixel_center_y = float(cy)

            bbox_array.boxes.append(bbox)

            # 시각화
            color = CLASS_COLOR.get(class_name, (0, 255, 0))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{class_name}", (x1, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(frame, (cx, cy), 6, color, -1)

            # 중심점 퍼블리시
            self.publish_center_point(class_name, cx, cy, color)

        self.bbox_pub.publish(bbox_array)

        cv2.imshow("Detections", frame)
        if cv2.waitKey(1) == ord('q'):
            rclpy.shutdown()

    def get_mask_center(self, mask, i):
        ys, xs = np.where(mask > 0.5)
        if len(xs) == 0:
            self.get_logger().warn(f"Mask {i} is empty after thresholding.")
            return None
        return int(xs.mean()), int(ys.mean())

    def publish_center_point(self, class_name, x, y, color):
        pt = PointStamped()
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.header.frame_id = "camera_link"
        pt.point.x = float(x)
        pt.point.y = float(y)
        pt.point.z = 0.0

        if class_name == "yellow":
            self.yellow_pub.publish(pt)
        elif class_name == "blue":
            self.blue_pub.publish(pt)

        self.pixel_circle_pub.publish(pt)


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ImageConcatNode(Node):
    def __init__(self):
        super().__init__('image_concat_node')
        self.bridge = CvBridge()

        # 두 카메라 이미지 구독
        # 위 이미지
        self.sub1 = self.create_subscription(
            Image, 'camera2/image_raw', self.callback1, 10)
        # 밑 이미지
        self.sub2 = self.create_subscription(
            Image, 'camera3/image_raw', self.callback2, 10)

        # 합쳐진 이미지 발행
        self.pub = self.create_publisher(Image, 'traffic_light_cam', 10)

        self.img1 = None
        self.img2 = None

    def callback1(self, msg):
        self.img1 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_concat()

    def callback2(self, msg):
        self.img2 = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.publish_concat()

    def publish_concat(self):
        if self.img1 is not None and self.img2 is not None:
            # 두 이미지를 세로로 이어붙임
            try:
                # 크기 맞추기 (width 같게)
                if self.img1.shape[1] != self.img2.shape[1]:
                    width = min(self.img1.shape[1], self.img2.shape[1])
                    self.img1 = cv2.resize(self.img1, (width, self.img1.shape[0]))
                    self.img2 = cv2.resize(self.img2, (width, self.img2.shape[0]))

                concat_img = np.vstack((self.img1, self.img2))
                img_msg = self.bridge.cv2_to_imgmsg(concat_img, "bgr8")
                self.pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f"Image concat failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConcatNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

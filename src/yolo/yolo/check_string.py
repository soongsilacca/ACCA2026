#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ClassNameSubscriber(Node):
    def __init__(self):
        super().__init__('class_name_subscriber')

        # ✅ /yolo/class_name 구독
        self.subscription = self.create_subscription(
            String,
            '/yolo/class_name',
            self.listener_callback,
            10
        )
        self.get_logger().info("ClassName Subscriber Node is running...")

    def listener_callback(self, msg):
        # 받은 class 이름을 그대로 출력
        self.get_logger().info(f"Received class name: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = ClassNameSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

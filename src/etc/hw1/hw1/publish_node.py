#ROS2
import rclpy
from rclpy.node import Node
from rclpy.qos import  qos_profile_system_default

#msg
from std_msgs.msg import String

file_path = "/home/bum/robot_ws/src/hw1/query.dat" 

class Publish_node(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher_node = self.create_publisher(String,"topic", qos_profile_system_default)
        self.timer = self.create_timer(1,self.publish)
        with open(file_path,'r') as f:
            self.data = f.readlines()
            print(self.data)
        self.count = 0
        

    def publish(self):
        self.count += 1
        str_topic = String()
        str_topic.data = str(self.data[self.count]).strip()
        self.publisher_node.publish(str_topic)
        self.get_logger().info(f"pub : {str_topic.data}")

def main(args=None):
    rclpy.init(args=args)
    node = Publish_node()

    while rclpy.ok():
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info("KeyboardInterrupt (SIGINT)")
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == "__main__":
    main()
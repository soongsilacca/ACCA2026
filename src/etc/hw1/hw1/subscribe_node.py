#ROS2
import rclpy
from rclpy.qos import qos_profile_system_default
from rclpy.node import Node

#msg
from std_msgs.msg import String
from std_msgs.msg import Int16

file_path = "/home/bum/robot_ws/src/hw1/query.dat" 

class Publish_node(Node):
    def __init__(self):
        super().__init__("publish_node")
        self.create_subscription(
            String,
            "topic",
            self.subscribe,
            qos_profile_system_default)
        self.get_set_pub = self.create_publisher(String,'get_set',qos_profile_system_default)
        self.get_count_pub = self.create_publisher(Int16,'get_count',qos_profile_system_default)
        self.set_count_pub = self.create_publisher(Int16,'set_count',qos_profile_system_default)
        self.getcount = 0
        self.setcount = 0
        self.count = 0

    def subscribe(self, msg):
        self.count += 1
        self.get_logger().info(f"sub_count:{self.count}")
        if msg.data[:3] == "get":
            data = msg.data[:3]
            self.getcount += 1
            
            self.get_count_pub.publish(Int16(data=self.getcount))
        elif msg.data[:3] == "set":
            data = msg.data[:3]
            self.setcount += 1
            self.set_count_pub.publish(Int16(data=self.setcount))
        else:
            data = 0

        if data:
            self.get_set_pub.publish(String(data=data))

def main(args = None):
    rclpy.init(args=args)
    node = Publish_node()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
        
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, qos_profile_system_default
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import math as m

class MPCNode(Node):
    def __init__(self):
        super().__init__('mpc_node')
        self.get_logger().info("MPC Node has been started.")
        
        # Define QoS profile for the publisher
        # qos_profile = QoSProfile(
        #     depth=10,
        #     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        # )
        qos_profile = QoSProfile(depth=10)

        
        self.create_subscription(Imu, "imu/data",self.callback, qos_profile)
        # Create a publisher with the defined QoS profile
        self.publisher_ = self.create_publisher(Float32, 'imu/yaw', qos_profile)
        #     SerialFeedBack,
        #     "erp42_feedback",
        #     self.callback_erp_fb,
        #     qos_profile=qos_profile_system_default,
        # )
        # Timer to publish messages periodically
    
    # def callback_erp_fb(self,msg):

    def callback(self, msg):
        _, _, yaw = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.publisher_.publish(Float32(data = m.degrees(yaw)))

def main():
    rclpy.init()
    node = MPCNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

main()
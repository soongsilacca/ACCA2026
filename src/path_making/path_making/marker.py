import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from tf_transformations import euler_from_quaternion
import math as m

class MarkerPublisher(Node):

    def __init__(self):
        super().__init__('marker_publisher')
        
        self.sub_init = self.create_subscription(PoseWithCovarianceStamped,'initialpose',self.callback_init,qos_profile=10)
        self.publisher = self.create_publisher(MarkerArray, 'markerr_array', 10)
        self.timer = self.create_timer(0.5, self.publish_markers)
        self.marker_array = MarkerArray()
        self.points = []
        
        self.i =0

        
    def callback_init(self,msg):
        # Define the points as provided
        x,y = msg.pose.pose.position.x, msg.pose.pose.position.y
        _, _, yaw = euler_from_quaternion([msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
        #Kcity
        # self.points = []
        #school
        self.points.append((x,y,m.degrees(yaw)))
        # Initialize markers
        for point in self.points:

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "markers"
            marker.id = self.i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD

            marker.pose.position.x = point[0]
            marker.pose.position.y = point[1]
            marker.pose.position.z = 0.0

            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = msg.pose.pose.orientation.z
            marker.pose.orientation.w = msg.pose.pose.orientation.w

            marker.scale.x = 0.4
            marker.scale.y = 0.4
            marker.scale.z = 0.1

            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            self.i+=1
            marker.color.a = 1.0

            self.marker_array.markers.append(marker)
        print(len(self.points))
        
        
    def publish_markers(self):
        for marker in self.marker_array.markers:
            marker.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(self.marker_array)
        print(self.points)

def main(args=None):
    rclpy.init(args=args)
    node = MarkerPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

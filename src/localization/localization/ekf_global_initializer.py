#!/usr/bin/env python3
"""
EKF Global Initializer

Reads first GPS position, calculates offset from map datum,
and sets initial pose for EKF Global via set_pose service.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import math


class EKFGlobalInitializer(Node):
    def __init__(self):
        super().__init__('ekf_global_initializer')
        
        # Datum (map origin) from map_anchor.yaml
        self.declare_parameter('datum_latitude', 37.239)
        self.declare_parameter('datum_longitude', 126.773)
        self.declare_parameter('datum_altitude', 0.0)
        
        self.datum_lat = self.get_parameter('datum_latitude').value
        self.datum_lon = self.get_parameter('datum_longitude').value
        self.datum_alt = self.get_parameter('datum_altitude').value
        
        # WGS84 Ellipsoid Constants
        self.a = 6378137.0  # Semi-major axis
        self.f = 1 / 298.257223563  # Flattening
        self.e2 = self.f * (2 - self.f)  # Square of eccentricity

        # Calculate conversion factors at datum latitude
        lat_rad = math.radians(self.datum_lat)
        sin_lat = math.sin(lat_rad)
        
        # Radii of curvature
        # Meridian radius (North-South)
        Rm = self.a * (1 - self.e2) / math.pow(1 - self.e2 * sin_lat**2, 1.5)
        # Prime vertical radius (East-West)
        Rn = self.a / math.sqrt(1 - self.e2 * sin_lat**2)

        self.M_PER_LAT = Rm * (math.pi / 180.0)
        self.M_PER_LON = Rn * math.cos(lat_rad) * (math.pi / 180.0)
        
        self.initialized = False
        
        # ROS communication
        self.sub_gps = self.create_subscription(
            NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        
        self.pub_set_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose/global', 10)
        
        self.get_logger().info(f'EKF Global Initializer Started. Waiting for first GPS...')
    
    def gps_callback(self, msg: NavSatFix):
        if self.initialized or msg.status.status < 0:
            return
        
        # Calculate offset from datum (X=North, Y=West)
        d_lat = msg.latitude - self.datum_lat
        d_lon = msg.longitude - self.datum_lon
        
        x_offset = d_lat * self.M_PER_LAT       # North
        y_offset = -(d_lon * self.M_PER_LON)    # West
        z_offset = 0.0 # Force 2D
        
        # Create and publish initial pose for EKF Global
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = x_offset
        pose_msg.pose.pose.position.y = y_offset
        pose_msg.pose.pose.position.z = z_offset
        pose_msg.pose.pose.orientation.w = 1.0
        
        # Set large covariance to allow EKF to adjust
        for i in range(6):
            pose_msg.pose.covariance[i*7] = 100.0
        
        self.pub_set_pose.publish(pose_msg)
        
        self.get_logger().info(f'Initialized EKF Global at GPS offset: ({x_offset:.2f}, {y_offset:.2f}) m from datum')
        self.get_logger().info(f'GPS: {msg.latitude:.6f}°N, {msg.longitude:.6f}°E')
        self.get_logger().info(f'Datum: {self.datum_lat:.6f}°N, {self.datum_lon:.6f}°E')
        
        self.initialized = True
        
        # Shutdown after initialization
        self.get_logger().info('Initialization complete. Node shutting down.')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = EKFGlobalInitializer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == '__main__':
    main()

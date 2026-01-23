#!/usr/bin/env python3
"""
EKF Global Initializer (UTM-based)

Reads first GPS position, converts to UTM, calculates offset from map datum,
and sets initial pose for EKF Global via set_pose service.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
import utm


class EKFGlobalInitializer(Node):
    def __init__(self):
        super().__init__('ekf_global_initializer')
        
        # Datum (map origin) from map_anchor.yaml
        self.declare_parameter('datum_latitude', 37.495)
        self.declare_parameter('datum_longitude', 126.957)
        self.declare_parameter('datum_altitude', 0.0)
        
        # UTM Zone information
        self.declare_parameter('utm_zone', 52)
        self.declare_parameter('utm_band', 'N')
        
        self.datum_lat = self.get_parameter('datum_latitude').value
        self.datum_lon = self.get_parameter('datum_longitude').value
        self.datum_alt = self.get_parameter('datum_altitude').value
        self.utm_zone = self.get_parameter('utm_zone').value
        self.utm_band = self.get_parameter('utm_band').value
        
        # Convert datum to UTM
        self.datum_easting, self.datum_northing, _, _ = utm.from_latlon(
            self.datum_lat,
            self.datum_lon,
            force_zone_number=self.utm_zone
        )
        
        self.initialized = False
        
        # ROS communication
        self.sub_gps = self.create_subscription(
            NavSatFix, '/ublox_gps_node/fix', self.gps_callback, 10)
        
        self.pub_set_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/set_pose/global', 10)
        
        self.get_logger().info(f'EKF Global Initializer Started (UTM). Waiting for first GPS...')
        self.get_logger().info(f'Datum UTM: E={self.datum_easting:.2f}, N={self.datum_northing:.2f} (Zone {self.utm_zone}{self.utm_band})')
    
    def gps_callback(self, msg: NavSatFix):
        if self.initialized or msg.status.status < 0:
            return
        
        # Convert GPS to UTM
        easting, northing, _, _ = utm.from_latlon(
            msg.latitude,
            msg.longitude,
            force_zone_number=self.utm_zone
        )
        
        # Calculate offset from datum (NWU coordinates)
        # X (North) = northing difference
        x_offset = northing - self.datum_northing
        # Y (West) = -easting difference
        y_offset = -(easting - self.datum_easting)
        z_offset = 0.0  # Force 2D
        
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
        
        self.get_logger().info(f'Initialized EKF Global at UTM offset: ({x_offset:.2f}, {y_offset:.2f}) m from datum')
        self.get_logger().info(f'GPS: {msg.latitude:.6f}°N, {msg.longitude:.6f}°E')
        self.get_logger().info(f'GPS UTM: E={easting:.2f}, N={northing:.2f}')
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

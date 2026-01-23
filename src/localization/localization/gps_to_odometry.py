#!/usr/bin/env python3
"""
GPS to Odometry Converter (UTM-based)

Converts GPS (NavSatFix) to Odometry message using UTM projection
Simpler and more accurate than manual WGS84 conversion
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import utm
import math


class GPSToOdometry(Node):
    def __init__(self):
        super().__init__("gps_to_odometry_node")

        # Datum (map origin) from map_anchor.yaml
        self.declare_parameter("datum_latitude", 37.495)
        self.declare_parameter("datum_longitude", 126.957)
        self.declare_parameter("datum_altitude", 0.0)
        
        # UTM Zone information
        self.declare_parameter("utm_zone", 52)
        self.declare_parameter("utm_band", 'N')

        self.datum_lat = self.get_parameter("datum_latitude").value
        self.datum_lon = self.get_parameter("datum_longitude").value
        self.datum_alt = self.get_parameter("datum_altitude").value
        self.utm_zone = self.get_parameter("utm_zone").value
        self.utm_band = self.get_parameter("utm_band").value

        # Convert datum to UTM coordinates
        self.datum_easting, self.datum_northing, _, _ = utm.from_latlon(
            self.datum_lat, 
            self.datum_lon,
            force_zone_number=self.utm_zone
        )

        # ROS communication
        self.sub_gps = self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix", self.gps_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/gps/odometry_nwu", 10)

        self.get_logger().info(f"GPS to Odometry Converter Started (UTM)")
        self.get_logger().info(f"Datum: {self.datum_lat:.6f}°N, {self.datum_lon:.6f}°E")
        self.get_logger().info(
            f"Datum UTM: E={self.datum_easting:.2f}, N={self.datum_northing:.2f} (Zone {self.utm_zone}{self.utm_band})"
        )
        
        # State for heading calculation
        self.prev_x = None
        self.prev_y = None
        self.min_dist_sq = 0.5**2 # 0.5m movement required to update heading

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # Invalid GPS

        # Convert GPS to UTM coordinates
        easting, northing, _, _ = utm.from_latlon(
            msg.latitude,
            msg.longitude,
            force_zone_number=self.utm_zone
        )

        # Calculate offset from datum (NWU coordinates)
        # X (North) = northing difference
        x_pos = northing - self.datum_northing
        # Y (West) = -easting difference (NWU convention)
        y_pos = -(easting - self.datum_easting)
        z_pos = 0.0  # Force 2D

        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = "map"  # Position in map frame
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = x_pos
        odom.pose.pose.position.y = y_pos
        odom.pose.pose.position.z = z_pos

        # Orientation & Heading Calculation
        # Calculate heading from motion (Course Over Ground)
        if self.prev_x is None:
            self.prev_x = x_pos
            self.prev_y = y_pos
            # Use Identity orientation if no history
            odom.pose.pose.orientation.w = 1.0
            # High covariance for orientation since we don't know it
            odom.pose.covariance[35] = 1000.0
        else:
            dx = x_pos - self.prev_x
            dy = y_pos - self.prev_y
            dist_sq = dx*dx + dy*dy
            
            if dist_sq > self.min_dist_sq:
                # We moved enough to calculate heading
                heading_rad = math.atan2(dy, dx)
                
                # Convert to Quaternion (Yaw only)
                # qz = sin(yaw/2), qw = cos(yaw/2)
                qz = math.sin(heading_rad / 2.0)
                qw = math.cos(heading_rad / 2.0)
                
                odom.pose.pose.orientation.z = qz
                odom.pose.pose.orientation.w = qw
                
                # Update previous position
                self.prev_x = x_pos
                self.prev_y = y_pos
                
                # Trust this heading! (Low covariance)
                # 1e-2 rad variance (~0.5 deg standard deviation)
                odom.pose.covariance[35] = 0.01 
            else:
                # Not moved enough, keep previous orientation (or identity if none)
                # But assume high covariance because we are stationary/noisy
                odom.pose.pose.orientation.w = 1.0
                # Better to just publish position/velocity and HIGH angular covariance
                # so EKF ignores it and relies on IMU/Wheel for stationary yaw hold
                odom.pose.covariance[35] = 1000.0

        # Covariance (Use GPS reported covariance)
        # NavSatFix covariance: [lat, lon, alt] (ENU)
        # Map to NWU: x (North) = lat, y (West) = lon
        odom.pose.covariance[0] = msg.position_covariance[0]  # x (North) from lat variance
        odom.pose.covariance[7] = msg.position_covariance[4]  # y (West) from lon variance  
        odom.pose.covariance[14] = msg.position_covariance[8] # z (Up) from alt variance
        
        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GPSToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

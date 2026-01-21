#!/usr/bin/env python3
"""
GPS to Odometry Converter (Fixed Datum)

Converts GPS (NavSatFix) to Odometry message using fixed datum from map_anchor.yaml
This replaces navsat_transform_node for simpler, predictable behavior
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
import math


class GPSToOdometry(Node):
    def __init__(self):
        super().__init__("gps_to_odometry_node")

        # Datum (map origin) from map_anchor.yaml
        self.declare_parameter("datum_latitude", 38.239)
        self.declare_parameter("datum_longitude", 126.773)
        self.declare_parameter("datum_altitude", 0.0)

        self.datum_lat = self.get_parameter("datum_latitude").value
        self.datum_lon = self.get_parameter("datum_longitude").value
        self.datum_alt = self.get_parameter("datum_altitude").value

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

        # ROS communication
        self.sub_gps = self.create_subscription(
            NavSatFix, "/ublox_gps_node/fix", self.gps_callback, 10
        )
        self.pub_odom = self.create_publisher(Odometry, "/gps/odometry_nwu", 10)

        self.get_logger().info(f"GPS to Odometry Converter Started (WGS84)")
        self.get_logger().info(f"Datum: {self.datum_lat}°N, {self.datum_lon}°E")
        self.get_logger().info(
            f"Conversion: {self.M_PER_LAT:.2f} m/deg (lat), {self.M_PER_LON:.2f} m/deg (lon)"
        )
        
        # State for heading calculation
        self.prev_x = None
        self.prev_y = None
        self.min_dist_sq = 0.5**2 # 0.5m movement required to update heading

    def gps_callback(self, msg: NavSatFix):
        if msg.status.status < 0:
            return  # Invalid GPS

        # Convert GPS to local NWU coordinates (X=North, Y=West, Z=Up)
        # Verify: X (North) = d_lat
        # Verify: Y (West)  = -d_lon (because d_lon is East)
        d_lat = msg.latitude - self.datum_lat
        d_lon = msg.longitude - self.datum_lon

        # NOTE: User requested Map X = North (2D Mode)
        x_pos = d_lat * self.M_PER_LAT  # North
        y_pos = -(d_lon * self.M_PER_LON)  # West (Right Hand Rule with Z Up)
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
                odom.pose.pose.orientation.w = 1.0 # Or keep last known?
                # Better to just publish position/velocity and HIGH angular covariance
                # so EKF ignores it and relies on IMU/Wheel for stationary yaw hold
                odom.pose.covariance[35] = 1000.0

        # Covariance (Force EKF to trust GPS Position)
        # Use extremely small covariance to reduce RMSE against GPS
        odom.pose.covariance[0] = 0.0001 # x variance
        odom.pose.covariance[7] = 0.0001 # y variance
        odom.pose.covariance[14] = 0.0001 # z variance
        
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

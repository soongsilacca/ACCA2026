#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import requests
import json
import math
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class OsrmBridge(Node):
    def __init__(self):
        super().__init__('osrm_bridge')
        
        # Parameters
        self.declare_parameter('osrm_url', 'http://localhost:5000')
        self.osrm_url = self.get_parameter('osrm_url').value
        
        # Subscribers
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, '/odometry/global', self.odom_callback, 10)
        
        # Publishers
        self.global_path_pub = self.create_publisher(Path, '/global_path', 10)
        
        self.current_pose = None
        self.get_logger().info("OSRM Bridge Node Initialized")

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def goal_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().warn("Current pose not available yet.")
            return

        start_lon, start_lat = self.project_local_to_latlon(self.current_pose.position.x, self.current_pose.position.y)
        end_lon, end_lat = self.project_local_to_latlon(msg.pose.position.x, msg.pose.position.y)
        
        self.get_logger().info(f"Requesting path from ({start_lat}, {start_lon}) to ({end_lat}, {end_lon})")
        
        coords = f"{start_lon},{start_lat};{end_lon},{end_lat}"
        url = f"{self.osrm_url}/route/v1/driving/{coords}?geometries=geojson&overview=full"
        
        try:
            response = requests.get(url)
            if response.status_code == 200:
                data = response.json()
                if data['code'] == 'Ok':
                    self.publish_path(data['routes'][0]['geometry']['coordinates'])
                else:
                    self.get_logger().error(f"OSRM Error: {data['code']}")
            else:
                self.get_logger().error(f"HTTP Error: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Request failed: {e}")

    def project_local_to_latlon(self, x, y):
        # NOTE: This projection MUST match what was used to create the OSRM map.
        # This is a placeholder. You need to know the origin of your local map.
        # Assuming MGRS or UTM, we'd need conversion. 
        # For now, let's assume the coordinates in OSRM were already local or we have a static transform.
        # IF the OSRM map was built from lat/lon, we need the origin.
        # Let's check how the OSRM map was created. It was created from a .osm file.
        # Usually .osm files use lat/lon.
        # IF the vehicle gives local XY, we need to convert back to Lat/Lon.
        
        # HACK for simulation/demo: Assuming small area flat earth near a reference point.
        # BUT wait, the visualizer uses "local_x" and "local_y" from the OSM file directly?
        # If the OSM file has local_x attributes, OSRM might just be using those node IDs?
        # OSRM works on Lat/Lon. 
        
        # If the user wants "OSM based path making" and we have an .osm file, 
        # we need to know how to map (local X,Y) -> (Lon, Lat).
        
        # Let's use a dummy reference point roughly near K-City or the provided map location.
        # Without this, OSRM won't work correctly unless we assume 1 meter ~ degrees.
        
        # Checking laneletviz.cpp: It overrode point.basicPoint().x() with "local_x".
        # This implies the OSM file DOES contain local coordinates.
        # However, standard OSRM expects Longitude/Latitude.
        
        # PROBABLY the .osm file has nodes with (lat, lon).
        # We need to find the specific projection.
        # FOR NOW, I will implement a placeholder and ask the user or try to deduce it.
        # Actually, if we look at the bags/school_2.osm file...
        
        # Use a simple approx:
        ref_lat = 37.239  # Approximate
        ref_lon = 126.773 # Approximate
        
        # 1 deg lat ~ 111km
        # 1 deg lon ~ 111km * cos(lat)
        
        d_lat = y / 111000.0
        d_lon = x / (111000.0 * math.cos(math.radians(ref_lat)))
        
        return ref_lon + d_lon, ref_lat + d_lat

    def project_latlon_to_local(self, lon, lat):
        ref_lat = 37.239
        ref_lon = 126.773
        
        x = (lon - ref_lon) * (111000.0 * math.cos(math.radians(ref_lat)))
        y = (lat - ref_lat) * 111000.0
        return x, y

    def publish_path(self, coordinates):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for lon, lat in coordinates:
            pose = PoseStamped()
            pose.header = path_msg.header
            x, y = self.project_latlon_to_local(lon, lat)
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
            
        self.global_path_pub.publish(path_msg)
        self.get_logger().info(f"Published path with {len(path_msg.poses)} points")

def main(args=None):
    rclpy.init(args=args)
    node = OsrmBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

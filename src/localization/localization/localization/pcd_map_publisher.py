#!/usr/bin/env python3
"""
PCD Map Publisher Node

Loads a PCD file and publishes it as a PointCloud2 message when a service is called.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
import numpy as np
import struct
import os


class PCDMapPublisher(Node):
    def __init__(self):
        super().__init__("pcd_map_publisher_node")
        
        # Parameters
        self.declare_parameter("pcd_file", "")
        self.pcd_file = self.get_parameter("pcd_file").value
        
        # Publisher (Transient Local for late joiners)
        qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_map = self.create_publisher(PointCloud2, "/map_cloud", qos_profile)
        
        # Service
        self.srv = self.create_service(Trigger, "/publish_map", self.publish_map_callback)
        
        # Load PCD file
        self.cloud_msg = None
        if os.path.exists(self.pcd_file):
            self.get_logger().info(f"Loading PCD file: {self.pcd_file}")
            self.cloud_msg = self.load_pcd(self.pcd_file)
            if self.cloud_msg:
                self.get_logger().info(f"PCD loaded successfully: {self.cloud_msg.width} points")
            else:
                self.get_logger().error("Failed to load PCD file")
        else:
            self.get_logger().error(f"PCD file not found: {self.pcd_file}")
        
        self.get_logger().info("PCD Map Publisher ready. Call /publish_map service to publish.")
    
    def load_pcd(self, filename):
        """Load PCD file and convert to PointCloud2 message"""
        try:
            with open(filename, 'rb') as f:
                # Read header
                header_lines = []
                while True:
                    line = f.readline().decode('ascii').strip()
                    header_lines.append(line)
                    if line.startswith('DATA'):
                        break
                
                # Parse header
                width = height = 0
                fields_info = []
                point_type = "ascii"
                
                for line in header_lines:
                    if line.startswith('WIDTH'):
                        width = int(line.split()[1])
                    elif line.startswith('HEIGHT'):
                        height = int(line.split()[1])
                    elif line.startswith('FIELDS'):
                        fields_info = line.split()[1:]
                    elif line.startswith('DATA'):
                        point_type = line.split()[1]
                
                # Read point data
                if point_type == 'ascii':
                    points_data = []
                    for line in f:
                        try:
                            values = [float(v) for v in line.decode('ascii').strip().split()]
                            if len(values) >= 3:  # At least x, y, z
                                points_data.append(values[:4] if len(values) >= 4 else values + [0])
                        except:
                            continue
                    points = np.array(points_data, dtype=np.float32)
                
                elif point_type == 'binary':
                    # Assume XYZRGB or XYZI format (common in PCD files)
                    num_points = width * height
                    # Try reading as float32 (4 fields: x, y, z, intensity/rgb)
                    point_data = f.read()
                    points = np.frombuffer(point_data, dtype=np.float32).reshape(-1, 4)
                    if len(points) > num_points:
                        points = points[:num_points]
                
                else:
                    self.get_logger().error(f"Unsupported PCD format: {point_type}")
                    return None
                
                # Create PointCloud2 message
                cloud_msg = PointCloud2()
                cloud_msg.header.frame_id = "map"
                cloud_msg.header.stamp = self.get_clock().now().to_msg()
                
                cloud_msg.height = 1
                cloud_msg.width = len(points)
                
                # Define fields (XYZI)
                cloud_msg.fields = [
                    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
                    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
                    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
                    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
                ]
                
                cloud_msg.is_bigendian = False
                cloud_msg.point_step = 16  # 4 fields * 4 bytes
                cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
                cloud_msg.is_dense = True
                
                # Pack data
                cloud_msg.data = points.tobytes()
                
                return cloud_msg
                
        except Exception as e:
            self.get_logger().error(f"Error loading PCD file: {e}")
            return None
    
    def publish_map_callback(self, request, response):
        """Service callback to publish the map"""
        if self.cloud_msg is None:
            response.success = False
            response.message = "PCD file not loaded"
            self.get_logger().warn("Cannot publish map: PCD not loaded")
            return response
        
        # Update timestamp
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self.pub_map.publish(self.cloud_msg)
        
        response.success = True
        response.message = f"Published map with {self.cloud_msg.width} points"
        self.get_logger().info(f"Map published: {self.cloud_msg.width} points")
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PCDMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

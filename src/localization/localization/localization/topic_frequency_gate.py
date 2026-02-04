#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
import collections

class TopicFrequencyGate(Node):
    def __init__(self):
        super().__init__('topic_frequency_gate')
        
        # Parameters
        self.declare_parameter('min_frequency', 10.0)
        self.declare_parameter('window_size', 10)
        self.declare_parameter('input_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/velodyne_points_stable')

        self.declare_parameter('max_frequency', 12.0)
        self.min_freq = self.get_parameter('min_frequency').value
        self.max_freq = self.get_parameter('max_frequency').value
        self.window_size = self.get_parameter('window_size').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.get_logger().info(f"Target Frequency Range: {self.min_freq} Hz ~ {self.max_freq} Hz")
        self.get_logger().info(f"Window Size: {self.window_size}")
        
        # State
        self.timestamps = collections.deque(maxlen=self.window_size)
        self.lockdown_until = 0.0
        self.min_period = 1.0 / self.max_freq if self.max_freq > 0 else 0.0
        
        self.target_interval = 1.0 / self.max_freq if self.max_freq > 0 else 0.0
        self.last_pub_time = 0.0
        
        # Publishers & Subscribers
        # Use Reliable with Deep Buffer to absorb bursts while Python processes them.
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=50
        )
        
        self.pub = self.create_publisher(PointCloud2, output_topic, 10)
        self.sub = self.create_subscription(
            PointCloud2,
            input_topic,
            self.callback,
            qos_profile=qos 
        )
        
        self.get_logger().info(f"Mode: Smart Downsampler (Python)")
        self.get_logger().info(f"Gating {input_topic} -> {output_topic}")
        self.get_logger().info(f"Max Frequency cap: {self.max_freq:.1f} Hz (Interval > {self.target_interval:.4f}s)")

    def callback(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        
        # Rate Limiting Logic
        dt = now - self.last_pub_time
        
        if dt < self.target_interval:
            # Drop (Throttling)
            return

        self.last_pub_time = now
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TopicFrequencyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

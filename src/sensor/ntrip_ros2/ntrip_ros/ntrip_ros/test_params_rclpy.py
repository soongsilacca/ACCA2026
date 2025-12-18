import rclpy
from rclpy.node import Node

class TestParams(Node):
    def __init__(self):
        super().__init__('test_params_rclpy')

        self.declare_parameter('my_str', "Hello")
        self.declare_parameter('my_int', "32")
        self.declare_parameter('my_double_array', [1.0, 2.0])
        
        param_str = self.get_parameter('my_str')
        param_int = self.get_parameter('my_int')
        param_double_array = self.get_parameter('my_double_array')
        
        self.get_logger().info("str: %s, int: %s, double[]: %s" %
                            (str(param_str.value),
                                str(param_int.value),
                                str(param_double_array.value),))

# The following is just to start the node
def main(args=None):
    rclpy.init(args=args)
    node = TestParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ContactsState
from controller_manager_msgs.srv import SwitchController

class CollisionMonitor(Node):
    def __init__(self):
        super().__init__('collision_monitor')
        
        # Subscribe to contact sensor topic
        self.contact_sub = self.create_subscription(
            ContactsState,
            '/contact_sensor',
            self.contact_callback,
            10
        )
        
        # Service client to switch controllers
        self.switch_controller_client = self.create_client(
            SwitchController,
            '/controller_manager/switch_controller'
        )
        
        self.triggered = False
        self.controllers_to_stop = ['arm_controller', 'gripper_controller']
        
        self.get_logger().info('Collision Monitor Initialized. Monitoring /contact_sensor...')

    def contact_callback(self, msg):
        if self.triggered:
            return
            
        if msg.states:
            # Collision detected
            # We can check the force magnitude if needed, but for now any contact triggers it
            collision_detected = False
            for state in msg.states:
                # Filter out contacts with the ground if necessary, but here we assume any contact is a hit
                # except maybe self-collision if not handled by physics engine (but these usually don't show up if <collision> tags match)
                if state.collision1_name and state.collision2_name:
                   collision_detected = True
                   break
            
            if collision_detected:
                self.get_logger().warn('Collision Detected! Stopping robot controllers...')
                self.stop_robot()
                self.triggered = True

    def stop_robot(self):
        if not self.switch_controller_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error('Controller Manager Switch Controller service not available')
            return

        request = SwitchController.Request()
        request.deactivate_controllers = self.controllers_to_stop
        request.activate_controllers = [] # Activate nothing
        request.strictness = SwitchController.Request.BEST_EFFORT
        request.start_asap = True
        request.timeout.sec = 0
        request.timeout.nanosec = 0
        
        future = self.switch_controller_client.call_async(request)
        future.add_done_callback(self.switch_controller_finished)

    def switch_controller_finished(self, future):
        try:
            response = future.result()
            if response.ok:
                self.get_logger().info('Controllers successfully deactivated. Robot should be limp "knocked down".')
            else:
                self.get_logger().error('Failed to deactivate controllers')
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

def main(args=None):
    rclpy.init(args=args)
    
    collision_monitor = CollisionMonitor()
    
    try:
        rclpy.spin(collision_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        collision_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

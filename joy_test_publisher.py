#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class JoyPublisher(Node):
    def __init__(self):
        super().__init__('joy_publisher')
        self.publisher_ = self.create_publisher(Joy, 'joy', 100)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish at 100Hz

    def timer_callback(self):
        msg = Joy()
        
        # Define the axes values (you can modify these as needed)
        msg.axes = [0.0] * 6  # 6 axes, initialize to 0.0

        # Define the button states (you can modify these as needed)
        msg.buttons = [0] * 21  # 21 buttons, initialize to 0 (unpressed)

        # Example modification to axes/buttons
        msg.axes[1] = 0.1  # Set the first axis to 1.0
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: axes: %s, buttons: %s' % (msg.axes, msg.buttons))

def main(args=None):
    rclpy.init(args=args)
    node = JoyPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node and shut down the client library
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

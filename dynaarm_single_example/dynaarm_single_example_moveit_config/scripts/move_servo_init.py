#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ServoCommandType


class ServoInitNode(Node):
    def __init__(self, command_type):
        super().__init__("move_servo_init")
        self.cli = self.create_client(ServoCommandType, "/servo_node/switch_command_type")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for service /servo_node/switch_command_type...")
        self.req = ServoCommandType.Request()
        self.req.command_type = command_type
        self.future = self.cli.call_async(self.req)
        self.get_logger().info(f"Calling service with command_type: {command_type}")
        self.future.add_done_callback(self.shutdown_node)

    def shutdown_node(self, future):
        self.get_logger().debug("Switch moveit servo command completed.")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    # Default to jog mode (0) if no argument is provided
    command_type = 0
    if len(sys.argv) > 1:
        try:
            command_type = int(sys.argv[1])
        except ValueError:
            print("Invalid argument. Please provide an integer for command_type.")
            sys.exit(1)
    node = ServoInitNode(command_type)
    rclpy.spin(node)


if __name__ == "__main__":
    main()

import sys
import tty
import termios
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher

from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from controller_manager_msgs.srv import SwitchController

abort_loop: bool = False


def read_char() -> int:

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def input_task(node: Node, publisher: Publisher):

    while not abort_loop:
        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        twist.header.frame_id = "wrist_2"

        input = read_char()
        print(ord(input))
        if ord(input) == 3:
            raise KeyboardInterrupt()

        step = 0.2

        if input == "a":
            twist.twist.linear.x = -step

        if input == "d":
            twist.twist.linear.x = +step

        if input == "y":
            twist.twist.linear.y = -step

        if input == "x":
            twist.twist.linear.y = +step

        if input == "w":
            twist.twist.linear.z = +step

        if input == "s":
            twist.twist.linear.z = -step

        if input == "k":
            twist.twist.angular.x = -step * 60
        if input == "ö":
            twist.twist.angular.x = step * 60

        if input == "o":
            twist.twist.angular.z = step * 600
        if input == "l":
            twist.twist.angular.z = -step * 60

        print(twist)
        # Default is an empty twist
        publisher.publish(twist)


def switch_controller(node, stop_controllers, start_controllers):

    # Service Clients
    switch_client = node.create_client(SwitchController, "/controller_manager/switch_controller")
    """Switches between controllers"""
    req = SwitchController.Request()
    req.deactivate_controllers = stop_controllers
    req.activate_controllers = start_controllers
    req.strictness = SwitchController.Request.STRICT
    req.activate_asap = True

    future = switch_client.call_async(req)
    rclpy.spin_until_future_complete(node, future)


def main():
    rclpy.init()

    node = Node("keypose_pose")

    switch_controller(node, ["freeze_controller"], ["joint_trajectory_controller"])

    pose_publisher = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    cmd_type_client = node.create_client(ServoCommandType, "/servo_node/switch_command_type")

    switch_cmd = ServoCommandType.Request()
    switch_cmd.command_type = 1

    input_thread = Thread(target=input_task, args=(node, pose_publisher))
    input_thread.start()

    future = cmd_type_client.call_async(switch_cmd)
    rclpy.spin_until_future_complete(node, future, timeout_sec=1.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # abort_loop = True
        input_thread.join()


if __name__ == "__main__":
    main()

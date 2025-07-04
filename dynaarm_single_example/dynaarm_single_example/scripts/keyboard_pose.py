import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

from geometry_msgs.msg import TwistStamped
from threading import Thread

from moveit_msgs.srv import ServoCommandType

abort_loop: bool = False


def read_char() -> int:
    import sys, tty, termios

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
        input = read_char()
        print(ord(input))
        if ord(input) == 3:
            raise KeyboardInterrupt()

        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        twist.header.frame_id = "wrist_2"

        step = 0.2

        if input == "a":
            twist.twist.linear.x = -step
            publisher.publish(twist)
            print(twist)

        if input == "d":
            twist.twist.linear.x = +step
            publisher.publish(twist)
            print(twist)
        if input == "y":
            twist.twist.linear.y = -step
            publisher.publish(twist)
            print(twist)

        if input == "x":
            twist.twist.linear.y = +step
            publisher.publish(twist)
            print(twist)
        if input == "w":
            twist.twist.linear.z = +step
            publisher.publish(twist)
            print(twist)
        if input == "s":
            twist.twist.linear.z = -step
            publisher.publish(twist)
            print(twist)


def main():
    rclpy.init()

    node = Node("keypose_pose")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

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
        abort_loop = True
        input_thread.join()


if __name__ == "__main__":
    main()

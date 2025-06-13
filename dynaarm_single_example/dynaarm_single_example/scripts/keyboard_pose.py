import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.publisher import Publisher
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
import tf2_geometry_msgs
from tf2_ros import TransformException

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from threading import Thread

abort_loop: bool = False

def read_char()->int:
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

    while(not abort_loop):
        input = read_char()
        print(ord(input))
        if ord(input) == 3:
            raise KeyboardInterrupt()
        
   
        twist = TwistStamped()
        twist.header.stamp = node.get_clock().now().to_msg()
        twist.header.frame_id = "wrist_2"


        if input == 'a':
            twist.twist.linear.x = -0.1
            publisher.publish(twist)
            print(twist)

        if input == 'd':
            twist.twist.linear.x = +0.1
            publisher.publish(twist)
            print(twist)
        if input == 'y':
            twist.twist.linear.y = -0.1
            publisher.publish(twist)
            print(twist)

        if input == 'x':
            twist.twist.linear.y = +0.1
            publisher.publish(twist)
            print(twist)
        if input == 'w':
            twist.twist.linear.z = +0.1
            publisher.publish(twist)
            print(twist)
        if input == 's':
            twist.twist.linear.z = -0.1
            publisher.publish(twist)
            print(twist)

        

        
    

def main():
    rclpy.init()

    node = Node("keypose_pose")

    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)
    
    pose_publisher = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds",10)

    input_thread = Thread(
        target=input_task, args=(node,pose_publisher)
    )
    input_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        abort_loop = True
        input_thread.join()


if __name__ == "__main__":
    main()
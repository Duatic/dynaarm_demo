import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__("twist_to_twiststamped_bridge")
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.callback, 10)
        self.pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)

    def callback(self, msg):
        ts = TwistStamped()
        ts.header.stamp = self.get_clock().now().to_msg()
        ts.header.frame_id = "flange"  # Use your actual end-effector frame name
        ts.twist = msg
        self.pub.publish(ts)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

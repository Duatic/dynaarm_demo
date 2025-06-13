import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog


class JoystickJointJog(Node):
    def __init__(self):
        super().__init__("joystick_joint_jog")
        self.publisher = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.joint_names = [
            "shoulder_rotation",
            "shoulder_flexion",
            "elbow_flexion",
            "forearm_rotation",
            "wrist_flexion",
            "wrist_rotation",
        ]
        self.scale = 1.0
        self.button_scale = 1.0
        self.left_stick_button = 7
        self.right_stick_button = 8
        self.last_nonzero = False

    def joy_callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        velocities = [
            float(axes[0]) * self.scale if len(axes) > 0 else 0.0,
            float(axes[1]) * self.scale if len(axes) > 1 else 0.0,
            float(axes[3]) * self.scale if len(axes) > 3 else 0.0,
            float(axes[2]) * self.scale if len(axes) > 2 else 0.0,
        ]
        wrist_flexion = 0.0
        if len(axes) > 4:
            wrist_flexion -= float(axes[4]) * self.scale
        if len(axes) > 5:
            wrist_flexion += float(axes[5]) * self.scale
        velocities.append(wrist_flexion)

        wrist_rotation = 0.0
        if len(buttons) > self.left_stick_button and buttons[self.left_stick_button]:
            wrist_rotation -= self.button_scale
        if len(buttons) > self.right_stick_button and buttons[self.right_stick_button]:
            wrist_rotation += self.button_scale
        velocities.append(wrist_rotation)

        is_nonzero = any(abs(v) > 1e-4 for v in velocities)

        # Only send zero-velocity command once when releasing the joystick
        if is_nonzero:
            joint_jog = JointJog()
            joint_jog.header.stamp = self.get_clock().now().to_msg()
            joint_jog.header.frame_id = ""
            joint_jog.joint_names = self.joint_names
            joint_jog.velocities = velocities
            joint_jog.displacements = []
            joint_jog.duration = 0.05
            self.publisher.publish(joint_jog)
        elif self.last_nonzero:
            # Send a single zero-velocity command
            joint_jog = JointJog()
            joint_jog.header.stamp = self.get_clock().now().to_msg()
            joint_jog.header.frame_id = ""
            joint_jog.joint_names = self.joint_names
            joint_jog.velocities = [0.0] * len(self.joint_names)
            joint_jog.displacements = []
            joint_jog.duration = 0.05
            self.publisher.publish(joint_jog)

        self.last_nonzero = is_nonzero


def main(args=None):
    rclpy.init(args=args)
    node = JoystickJointJog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

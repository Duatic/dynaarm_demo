import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from control_msgs.msg import JointJog
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType


class JoystickJointJog(Node):
    def __init__(self):
        super().__init__("joystick_joint_jog")

        # Publishers
        self.joint_publisher = self.create_publisher(JointJog, "/servo_node/delta_joint_cmds", 10)
        self.twist_publisher = self.create_publisher(
            TwistStamped, "/servo_node/delta_twist_cmds", 10
        )

        # Subscription
        self.subscription = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # Service client for switching command type
        self.cmd_type_client = self.create_client(
            ServoCommandType, "/servo_node/switch_command_type"
        )

        # Joint names for joint jog
        self.joint_names = [
            "shoulder_rotation",
            "shoulder_flexion",
            "elbow_flexion",
            "forearm_rotation",
            "wrist_flexion",
            "wrist_rotation",
        ]

        # Parameters
        self.scale = 1.0
        self.button_scale = 1.0
        self.left_stick_button = 7
        self.right_stick_button = 8

        # Mode control
        self.servo_active = True
        self.is_joint_mode = True  # True = Joint Jog, False = Twist Jog

        self.switch_button = 4  # Button to switch between modes (usually SELECT button)
        self.last_switch_button_state = False
        self.switch_button_pressed = False

        self.toggle_start_stop_button = 6
        self.toggle_start_stop_button_pressed = False
        self.last_start_stop_button_state = False

        # Frame for twist commands
        self.twist_frame = "wrist_2"

        # Timer for processing commands and service calls at 100Hz
        self.timer = self.create_timer(0.01, self.timer_callback)  # 100Hz = 0.01s

        # Service call state
        self.pending_switch_future = None

        self.switch_servo_mode_async(0)
        self.get_logger().info("JoystickJointJog initialized")
        self.get_logger().info("Timer running at 100Hz for service calls")

    def timer_callback(self):
        """Timer callback running at 100Hz to handle service calls"""
        # Handle pending switch command

        if self.toggle_start_stop_button_pressed:
            self.toggle_start_stop_button_pressed = False
            if self.servo_active:
                self.switch_servo_mode_async(2)
                self.get_logger().info("Servo stopped")
            else:
                self.is_joint_mode = True
                self.switch_servo_mode_async(0)
                self.get_logger().info("Servo activated")
            self.servo_active = not self.servo_active

        if self.switch_button_pressed:
            self.switch_button_pressed = False
            if self.pending_switch_future is None:
                self.is_joint_mode = not self.is_joint_mode
                self.switch_servo_mode_async(0 if self.is_joint_mode else 1)
                self.get_logger().info("Switching servo mode...")

        # Check if pending switch is complete
        if self.pending_switch_future is not None:
            if self.pending_switch_future.done():
                try:
                    if self.pending_switch_future.result():
                        if self.servo_active:
                            mode_name = "Joint Jog" if self.is_joint_mode else "Twist Jog"
                            self.get_logger().info(f"Switched to {mode_name} mode")
                    else:
                        self.get_logger().warn("Failed to switch servo mode")
                except Exception as e:
                    self.get_logger().error(f"Error switching servo mode: {e}")
                finally:
                    self.pending_switch_future = None

    def switch_servo_mode_async(self, command_type: int):
        """Switch between Joint Jog (0), Twist Jog (1) and Stop (2) modes asynchronously"""
        if not self.cmd_type_client.wait_for_service(timeout_sec=0.1):
            self.get_logger().warn("Servo command type service not available")
            return

        switch_cmd = ServoCommandType.Request()
        switch_cmd.command_type = command_type

        self.pending_switch_future = self.cmd_type_client.call_async(switch_cmd)

    def joy_callback(self, msg):
        axes = msg.axes
        buttons = msg.buttons

        if len(buttons) > self.toggle_start_stop_button:
            current_stop_state = buttons[self.toggle_start_stop_button]
            if current_stop_state and not self.last_start_stop_button_state:
                # Stop button pressed (rising edge) - set flag for timer to handle
                self.toggle_start_stop_button_pressed = True
            self.last_start_stop_button_state = current_stop_state

        if not self.servo_active:
            return

        # Handle mode switching - just set the flag, don't call service directly
        if len(buttons) > self.switch_button:
            current_switch_state = buttons[self.switch_button]
            if current_switch_state and not self.last_switch_button_state:
                # Button pressed (rising edge) - set flag for timer to handle
                self.switch_button_pressed = True
            self.last_switch_button_state = current_switch_state

        # Process commands based on current mode
        if self.is_joint_mode:  # Joint Jog mode
            self.handle_joint_jog(axes, buttons)
        else:
            self.handle_twist_jog(axes, buttons)

    def handle_joint_jog(self, axes, buttons):
        """Handle joystick input for joint jog mode"""
        velocities = [
            float(axes[0]) * self.scale if len(axes) > 0 else 0.0,
            float(axes[1]) * self.scale if len(axes) > 1 else 0.0,
            float(axes[3]) * self.scale if len(axes) > 3 else 0.0,
            float(axes[2]) * self.scale if len(axes) > 2 else 0.0,
        ]

        # Wrist flexion (using triggers)
        wrist_flexion = 0.0
        if len(axes) > 4:
            wrist_flexion -= float(axes[4]) * self.scale
        if len(axes) > 5:
            wrist_flexion += float(axes[5]) * self.scale
        velocities.append(wrist_flexion)

        # Wrist rotation (using stick buttons)
        wrist_rotation = 0.0
        if len(buttons) > self.left_stick_button and buttons[self.left_stick_button]:
            wrist_rotation -= self.button_scale
        if len(buttons) > self.right_stick_button and buttons[self.right_stick_button]:
            wrist_rotation += self.button_scale
        velocities.append(wrist_rotation)

        # Create and publish joint jog message
        joint_jog = JointJog()
        joint_jog.header.stamp = self.get_clock().now().to_msg()
        joint_jog.header.frame_id = ""
        joint_jog.joint_names = self.joint_names
        joint_jog.velocities = velocities
        joint_jog.displacements = []
        joint_jog.duration = 0.05
        self.joint_publisher.publish(joint_jog)

    def handle_twist_jog(self, axes, buttons):
        """Handle joystick input for twist jog mode"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.twist_frame

        # Linear movement (left stick for X/Y, right stick Y for Z)
        if len(axes) > 0:
            twist.twist.linear.x = float(axes[0]) * self.scale
        if len(axes) > 1:
            twist.twist.linear.y = float(axes[1]) * self.scale
        if len(axes) > 3:
            twist.twist.linear.z = float(axes[3]) * self.scale

        # Angular movement (right stick X for rotation around Z)
        if len(axes) > 2:
            twist.twist.angular.z = float(axes[2]) * self.scale * 100

        # Additional rotations using triggers
        if len(axes) > 4:
            twist.twist.angular.x -= float(axes[4]) * self.scale * 100
        if len(axes) > 5:
            twist.twist.angular.x += float(axes[5]) * self.scale * 100

        # Additional rotations using stick buttons
        if len(buttons) > self.left_stick_button and buttons[self.left_stick_button]:
            twist.twist.angular.y -= self.button_scale
        if len(buttons) > self.right_stick_button and buttons[self.right_stick_button]:
            twist.twist.angular.y += self.button_scale

        # Publish twist command
        self.twist_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickJointJog()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

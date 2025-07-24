import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
import numpy as np
import random
from rclpy.duration import Duration
from builtin_interfaces.msg import Time as TimeMsg
from scipy.spatial.transform import Rotation as R

class WorkspaceExplorer(Node):
    def __init__(self):
        super().__init__('workspace_explorer')

        # Parameters
        self.declare_parameter('num_samples', 50)
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.position_tolerance = 0.01  # meters
        self.timeout_duration = Duration(seconds=4.0)

        # State
        self.sample_count = 0
        self.sampled_poses = []
        self.current_pose = None
        self.target_pose = None
        self.state = 'IDLE'
        self.command_sent_time = None

        # Publishers and Subscribers
        self.pose_pub = self.create_publisher(PoseStamped, '/cartesian_motion_controller/target_frame', 10)
        self.marker_pub = self.create_publisher(Marker, '/workspace_points', 10)
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',
            self.current_pose_callback,
            10
        )

        # Main loop timer
        self.timer = self.create_timer(0.1, self.loop)

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose


    def random_quaternion(self, max_angle_deg=60.0):
        # Limit in degrees → radians
        max_angle_rad = np.radians(max_angle_deg)

        # Sample small rotations around each axis
        roll = np.random.uniform(-max_angle_rad, max_angle_rad)
        pitch = np.random.uniform(-max_angle_rad, max_angle_rad)
        yaw = np.random.uniform(-max_angle_rad, max_angle_rad)

        r = R.from_euler('xyz', [roll, pitch, yaw])
        qx, qy, qz, qw = r.as_quat()
        return qx, qy, qz, qw

    def loop(self):
        if self.sample_count >= self.num_samples:
            if self.state != 'DONE':
                self.get_logger().info('✅ Done sampling workspace.')
                self.state = 'DONE'
            return

        # FSM Logic
        if self.state == 'IDLE':
            # Generate and send pose
            pose = PoseStamped()
            pose.header.frame_id = 'base'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = random.uniform(-0.6, 0.6)
            pose.pose.position.y = random.uniform(-0.6, 0.6)
            pose.pose.position.z = random.uniform(0.2, 0.8)
            qx, qy, qz, qw = self.random_quaternion()
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.target_pose = pose
            self.pose_pub.publish(pose)
            self.command_sent_time = self.get_clock().now()
            self.get_logger().info(f'[{self.sample_count + 1}] Sent target pose: {pose.pose.position}')
            self.state = 'MOVING'

        elif self.state == 'MOVING':
            # Wait and check if robot reached target
            if self.current_pose is None:
                return

            time_elapsed = self.get_clock().now() - self.command_sent_time
            dx = self.target_pose.pose.position.x - self.current_pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.position.y
            dz = self.target_pose.pose.position.z - self.current_pose.position.z
            dist = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if dist < self.position_tolerance:
                self.get_logger().info(f'[{self.sample_count + 1}] ✅ Reached target. Error: {dist:.3f} m')
                self.sampled_poses.append((self.target_pose.pose, 'green'))
                self.sample_count += 1
                self.state = 'IDLE'
                self.publish_marker()
            elif time_elapsed > self.timeout_duration:
                self.get_logger().warn(f'[{self.sample_count + 1}] ❌ Target NOT reached in time. Error: {dist:.3f} m')
                self.sampled_poses.append((self.target_pose.pose, 'red'))
                self.sample_count += 1
                self.state = 'IDLE'
                self.publish_marker()

    def publish_marker(self):
        marker = Marker()
        marker.header = Header(frame_id='base')
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.colors.clear()
        marker.points.clear()

        for pose, color in self.sampled_poses:
            marker.points.append(pose.position)
            if color == 'green':
                marker.colors.append(ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0))
            else:
                marker.colors.append(ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0))

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = WorkspaceExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

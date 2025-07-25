import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from rclpy.duration import Duration
import numpy as np
from scipy.spatial.transform import Rotation as R


def quaternion_look_at(direction, up=np.array([0, 0, 1])):
    """Return a quaternion such that the z-axis points in the given direction."""
    z = direction / np.linalg.norm(direction)
    x = np.cross(up, z)
    if np.linalg.norm(x) < 1e-6:
        x = np.array([1.0, 0.0, 0.0])
    else:
        x = x / np.linalg.norm(x)
    y = np.cross(z, x)
    rot_matrix = np.column_stack((x, y, z))
    r = R.from_matrix(rot_matrix)
    return r.as_quat()


class WorkspaceExplorer(Node):
    def __init__(self):
        super().__init__('workspace_grid_explorer')

        # Parameters
        self.position_tolerance = 0.01  # meters
        self.timeout_duration = Duration(seconds=2.0)

        # Grid definition
        self.x_range = np.linspace(-0.6, 0.6, 6)
        self.y_range = np.linspace(-0.6, 0.6, 6)
        self.z_range = np.linspace(0.0, 1.0, 5)

        self.grid_positions = [
            (x, y, z)
            for x in self.x_range
            for y in self.y_range
            for z in self.z_range
        ]
        self.grid_index = 0

        # State
        self.sampled_poses = []
        self.current_pose = None
        self.target_pose = None
        self.state = 'IDLE'
        self.command_sent_time = None
        self.last_pose_position = None

        # ROS Interfaces
        self.pose_pub = self.create_publisher(PoseStamped, '/cartesian_motion_controller/target_frame', 10)
        self.marker_pub = self.create_publisher(Marker, '/workspace_points', 10)
        self.current_pose_sub = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',
            self.current_pose_callback,
            10
        )

        # Main loop
        self.timer = self.create_timer(0.1, self.loop)

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose

    def loop(self):
        if self.state == 'DONE':
            return

        if self.state == 'IDLE':
            if self.grid_index >= len(self.grid_positions):
                self.get_logger().info('✅ Finished scanning workspace grid.')
                self.state = 'DONE'
                return

            x, y, z = self.grid_positions[self.grid_index]
            self.grid_index += 1

            pose = PoseStamped()
            pose.header.frame_id = 'base'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            direction = np.array([x, y, z])
            qx, qy, qz, qw = quaternion_look_at(direction)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw

            self.target_pose = pose
            self.pose_pub.publish(pose)
            self.command_sent_time = self.get_clock().now()
            self.get_logger().info(f'[{self.grid_index}/{len(self.grid_positions)}] Sent target pose: {pose.pose.position}')
            self.state = 'MOVING'

        elif self.state == 'MOVING':
            if self.current_pose is None:
                return

            time_elapsed = self.get_clock().now() - self.command_sent_time
            dx = self.target_pose.pose.position.x - self.current_pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.position.y
            dz = self.target_pose.pose.position.z - self.current_pose.position.z
            dist = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

            if dist < self.position_tolerance:
                self.get_logger().info(f'✅ Reached pose. Error: {dist:.3f} m')
                self.sampled_poses.append((self.target_pose.pose, 'green'))
                self.last_pose_position = np.array([
                    self.target_pose.pose.position.x,
                    self.target_pose.pose.position.y,
                    self.target_pose.pose.position.z
                ])
                self.publish_marker()
                self.state = 'IDLE'

            elif time_elapsed > self.timeout_duration:
                self.get_logger().warn(f'❌ Failed to reach pose in time. Error: {dist:.3f} m')
                self.sampled_poses.append((self.target_pose.pose, 'red'))
                self.last_pose_position = np.array([
                    self.target_pose.pose.position.x,
                    self.target_pose.pose.position.y,
                    self.target_pose.pose.position.z
                ])
                self.publish_marker()
                self.state = 'IDLE'

    def publish_marker(self):
        marker = Marker()
        marker.header = Header(frame_id='base')
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01

        marker.points.clear()
        marker.colors.clear()

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

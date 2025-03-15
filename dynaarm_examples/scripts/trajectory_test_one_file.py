import os
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import time
import math
import pandas as pd
import subprocess
from datetime import datetime  # For timestamping rosbag files


class DynaArm(Node):
    def __init__(self):
        super().__init__("dyna_arm")

        # ROS 2 package path
        package_path = get_package_share_directory("dynaarm_examples")
        
        # Trajectory file (for single test)
        self.csv_path = os.path.join(package_path, "scripts", "trajectory_data", "trajectory_data_config3_vel2.5.csv")

        # Ensure rosbag recordings directory exists
        self.rosbag_dir = os.path.join(os.getcwd(), "rosbag_recordings")
        os.makedirs(self.rosbag_dir, exist_ok=True)

        # Publishers
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.create_subscription(JointState, "/joint_states", self.joint_state_callback, 10)
        self.current_joint_positions = {}

        # Service Clients
        self.switch_client = self.create_client(SwitchController, '/controller_manager/switch_controller')

        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for switch_controller service...")

        # Joint limits
        self.joint_limits = {
            "shoulder_rotation": (-3 * math.pi / 2, 3 * math.pi / 2),
            "shoulder_flexion": (-math.pi / 2 - 0.3, math.pi / 2 + 0.3),
            "elbow_flexion": (-0.05, math.pi),
            "forearm_rotation": (-3 * math.pi / 2, 3 * math.pi / 2),
            "wrist_flexion": (-math.pi / 2, math.pi / 2),
            "wrist_rotation": (-3 * math.pi / 2, 3 * math.pi / 2)
        }

        # Initial start position
        self.start_position = [0.0, math.radians(20), math.radians(90), math.radians(-90), math.radians(70), 0.0]

    def joint_state_callback(self, msg):
        """Update current joint positions from /joint_states topic."""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def get_current_joint_positions(self):
        """Returns the latest known joint positions as a list."""
        joint_names = list(self.joint_limits.keys())  # Ensure correct order
        return [self.current_joint_positions.get(joint, 0.0) for joint in joint_names]

    def switch_controller(self, stop_controllers, start_controllers):
        """Switches between controllers."""
        req = SwitchController.Request()
        req.deactivate_controllers = stop_controllers
        req.activate_controllers = start_controllers
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True

        future = self.switch_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info("Controller switch successful.")
        else:
            self.get_logger().error("Controller switch failed.")

    def move_to_start(self):
        """Move the arm smoothly to the initial start position if not already there."""
        current_positions = self.get_current_joint_positions()

        if all(abs(curr - target) < 0.05 for curr, target in zip(current_positions, self.start_position)):
            self.get_logger().info("Already at start position, skipping movement.")
            return

        self.get_logger().info("Moving to start position smoothly...")

        transition_traj = JointTrajectory()
        transition_traj.joint_names = list(self.joint_limits.keys())

        point = JointTrajectoryPoint()
        point.positions = self.start_position
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = Duration(seconds=3.0).to_msg()  # Slow transition

        transition_traj.points.append(point)
        self.pub.publish(transition_traj)
        time.sleep(3.5)  # Ensure completion

    def record_rosbag(self, csv_filename):
        """Starts recording a rosbag with a timestamped filename inside rosbag_recordings/."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_filename = f"{csv_filename.replace('.csv', '')}_{timestamp}"
        bag_path = os.path.join(self.rosbag_dir, bag_filename)

        self.get_logger().info(f"Starting rosbag recording: {bag_path}")
        return subprocess.Popen([
            "ros2", "bag", "record",
            "/joint_states",
            "/joint_trajectory_controller/controller_state",
            "/dynaarm_status_broadcaster/state",
            "/tf",
            "-o", bag_path
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def move_joints(self, csv_path=None):
        """Loads a trajectory from CSV, records a rosbag, and executes it safely."""
        if csv_path is None:
            csv_path = self.csv_path  # Default to the predefined CSV file

        # Load CSV data
        df = pd.read_csv(csv_path)
        time_stamps = df["time"].values
        positions = df[[f"pos{i+1}" for i in range(6)]].values
        velocities = df[[f"vel{i+1}" for i in range(6)]].values
        accelerations = df[[f"acc{i+1}" for i in range(6)]].values
        bool_throw = df["bool_throw"].values

        # Ensure last velocity and acceleration are zero
        velocities[-1] = [0.0] * 6
        accelerations[-1] = [0.0] * 6

        # Move to start position before executing
        self.move_to_start()

        # Start rosbag recording
        csv_filename = os.path.basename(csv_path)
        rosbag_process = self.record_rosbag(csv_filename)

        # Send trajectory
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_limits.keys())

        release_index = None

        for i, (pos, vel, acc, t, throw) in enumerate(zip(positions, velocities, accelerations, time_stamps, bool_throw)):
            point = JointTrajectoryPoint()
            point.positions = pos.tolist()
            point.velocities = vel.tolist()
            point.accelerations = acc.tolist()
            point.time_from_start = Duration(seconds=float(t)).to_msg()
            traj.points.append(point)

            if throw == 1 and release_index is None:
                release_index = i

        self.pub.publish(traj)
        self.get_logger().info(f"Trajectory '{csv_filename}' sent.")

        time.sleep(time_stamps[-1])

        if release_index is not None:
            time.sleep(0.05)

        self.get_logger().info("Stopping rosbag recording.")
        rosbag_process.terminate()
        rosbag_process.wait()
        self.get_logger().info(f"Rosbag saved: {csv_filename}")

def main():
    rclpy.init()
    arm = DynaArm()

    arm.switch_controller(['freeze_controller'], ['joint_trajectory_controller'])
    arm.move_joints()  # Runs the default trajectory file
    arm.switch_controller(['joint_trajectory_controller'], ['freeze_controller'])

    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

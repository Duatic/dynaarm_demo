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

class DynaArm(Node):
    def __init__(self):
        super().__init__("dyna_arm")

        # ROS 2 Paketpfad ermitteln
        package_path = get_package_share_directory("dynaarm_examples")
        self.csv_path = os.path.join(package_path, "scripts", "trajectory_data_config3_vel0.5.csv")

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

        # Initial joint position `q0` as defined in the email
        self.q0 = [0.0, math.radians(20), math.radians(90), math.radians(-90), math.radians(70), 0.0]

    def joint_state_callback(self, msg):
        """Callback function to update current joint positions from /joint_states topic."""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def get_current_joint_positions(self):
        """Returns the latest known joint positions as a list."""
        joint_names = list(self.joint_limits.keys())  # Order must match expected joint order
        positions = [self.current_joint_positions.get(joint, 0.0) for joint in joint_names]
        return positions

    def switch_controller(self, stop_controllers, start_controllers):
        """ Switches between controllers """
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

    def move_to_q0(self):
        """Move the arm smoothly to the initial position q0 if not already there."""
        current_positions = self.get_current_joint_positions()

        if all(abs(curr - target) < 0.05 for curr, target in zip(current_positions, self.q0)):
            self.get_logger().info("Already at q0, skipping movement.")
            return

        self.get_logger().info("Moving to q0 smoothly...")

        transition_traj = JointTrajectory()
        transition_traj.joint_names = list(self.joint_limits.keys())

        point = JointTrajectoryPoint()
        point.positions = self.q0
        point.velocities = [0.0] * 6
        point.accelerations = [0.0] * 6
        point.time_from_start = Duration(seconds=3.0).to_msg()  # Slow transition

        transition_traj.points.append(point)
        self.pub.publish(transition_traj)
        time.sleep(3.5)  # Ensure completion

    def move_joints(self):
        """Loads trajectory from CSV and executes it safely."""

        # Load CSV data
        df = pd.read_csv(self.csv_path)
        time_stamps = df["time"].values
        positions = df[[f"pos{i+1}" for i in range(6)]].values
        velocities = df[[f"vel{i+1}" for i in range(6)]].values
        accelerations = df[[f"acc{i+1}" for i in range(6)]].values
        bool_throw = df["bool_throw"].values  # Identify release instant

        # Ensure last velocity and acceleration are zero
        velocities[-1] = [0.0] * 6
        accelerations[-1] = [0.0] * 6

        # Move to q0 before executing
        self.move_to_q0()

        # Send trajectory
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_limits.keys())

        release_index = None  # Store the row index of the release instant

        for i, (pos, vel, acc, t, throw) in enumerate(zip(positions, velocities, accelerations, time_stamps, bool_throw)):
            point = JointTrajectoryPoint()
            point.positions = pos.tolist()
            point.velocities = vel.tolist()
            point.accelerations = acc.tolist()
            point.time_from_start = Duration(seconds=float(t)).to_msg()

            traj.points.append(point)

            # Check if this is the throwing moment
            if throw == 1 and release_index is None:
                release_index = i

        self.pub.publish(traj)
        self.get_logger().info("Trajectory sent.")

        # Wait for execution
        time.sleep(time_stamps[-1])

        if release_index is not None:
            self.get_logger().info(f"Release instant at row {release_index}, time {time_stamps[release_index]}s.")

def main():
    rclpy.init()
    arm = DynaArm()

    arm.switch_controller(['freeze_controller'], ['joint_trajectory_controller'])
    arm.move_joints()
    arm.switch_controller(['joint_trajectory_controller'], ['freeze_controller'])

    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

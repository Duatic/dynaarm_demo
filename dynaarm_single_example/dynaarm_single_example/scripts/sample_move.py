import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from controller_manager_msgs.srv import SwitchController
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import time
import math
import numpy as np


class DynaArm(Node):
    def __init__(self):
        super().__init__("dyna_arm")

        # Publishers
        self.pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        # Service Clients
        self.switch_client = self.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )

        # Wait for services
        while not self.switch_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for switch_controller service...")

        # Joint limits
        self.joint_limits = {
            "shoulder_rotation": (-3 * math.pi / 2, 3 * math.pi / 2),
            "shoulder_flexion": (-math.pi / 2 - 0.3, math.pi / 2 + 0.3),
            "elbow_flexion": (-0.05, math.pi),
            "forearm_rotation": (-3 * math.pi / 2, 3 * math.pi / 2),
            "wrist_flexion": (-math.pi / 2, math.pi / 2),
            "wrist_rotation": (-3 * math.pi / 2, 3 * math.pi / 2),
        }

    # Generate synusoidal trajectory for the arm to follow, this should smoothen the motion of the arm
    def piecewise_sinusoidal_trajectory(
        self,
        positions_start,
        positions_intermediate,
        positions_end,
        duration1,
        duration2,
        steps1,
        steps2,
    ):
        """
        Generates a piecewise sinusoidal trajectory for robotic arm joints.

        Args:
            positions_start: Initial joint positions (6 elements).
            positions_intermediate: Intermediate joint positions (6 elements).
            positions_end: Final joint positions (6 elements).
            duration1: Duration of the first part of the trajectory (start to intermediate).
            duration2: Duration of the second part of the trajectory (intermediate to end).
            steps1: Number of steps for the first part.
            steps2: Number of steps for the second part.

        Returns:
            A numpy array of shape (steps1 + steps2, 6) containing the joint positions.
        """

        positions_start = np.array(positions_start)
        positions_intermediate = np.array(positions_intermediate)
        positions_end = np.array(positions_end)

        new_step1 = steps1 - 1
        trajectory = np.zeros((new_step1 + steps2, 6))

        # First part of the trajectory (start to intermediate)

        new_stop = duration1 - (duration1 / new_step1)
        time1 = np.linspace(0, new_stop, new_step1)
        delta_positions1 = positions_intermediate - positions_start
        for i in range(6):
            trajectory[:(new_step1), i] = positions_start[i] + 0.5 * delta_positions1[i] * (
                1 - np.cos(np.pi * time1 / duration1)
            )

        # Second part of the trajectory (intermediate to end)
        time2 = np.linspace(0, duration2, steps2)
        step_size = time1[1]
        start_time2 = step_size + time1[-1]
        new_time2 = np.linspace(start_time2, duration2 + start_time2, steps2)
        delta_positions2 = positions_end - positions_intermediate
        for i in range(6):
            trajectory[(new_step1):, i] = positions_intermediate[i] + 0.5 * delta_positions2[i] * (
                1 - np.cos(np.pi * time2 / duration2)
            )

        return trajectory, np.concatenate((time1, new_time2))

    def switch_controller(self, stop_controllers, start_controllers):
        """Switches between controllers"""
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

    def move_to_zero(self, speed=0.1):
        """Moves the robot to zero position smoothly if not already at zero"""
        # current_positions = self.get_current_joint_positions()

        # if all(abs(pos) < 0.001 for pos in current_positions):
        #     self.get_logger().info("Robot is already in zero position. No movement needed.")
        #     return

        self.get_logger().info("Moving to zero position...")
        self.switch_controller(["freeze_controller"], ["joint_trajectory_controller"])

        duration = 1.0 / speed
        zero_positions = [0.0] * len(self.joint_limits)

        self.send_trajectory(zero_positions, duration)

        self.get_logger().info("Zero position reached.")
        self.switch_controller(["joint_trajectory_controller"], ["freeze_controller"])

    def move_joints(self, duration=1):

        # Send Home
        home_positions = [0.0] * 6
        self.send_trajectory(home_positions, 2)
        time.sleep(0.5)

        # Define start, intermediate, and end positions
        positions_start = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        positions_intermediate = [-1.0, -1.5, 0.5, -1.5, 1.5, -2.0]
        positions_end = [1.5, 1.0, 2.0, 1.5, -1.5, 2.0]

        # Durations
        duration1 = 1.2
        duration2 = duration1
        steps1 = 1000
        steps2 = steps1

        # Generate trajectory points
        trajectory_points, time_vector = self.piecewise_sinusoidal_trajectory(
            positions_start,
            positions_intermediate,
            positions_end,
            duration1,
            duration2,
            steps1,
            steps2,
        )

        # Initialize JointTrajectory message
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_limits.keys())

        # Create JointTrajectoryPoint for each position with the correct timestamp
        for i, (pos, my_time) in enumerate(zip(trajectory_points, time_vector)):
            point = JointTrajectoryPoint()
            point.positions = pos
            point.time_from_start = Duration(
                seconds=float(my_time)
            ).to_msg()  # Convert time to ROS2 message format
            traj.points.append(point)

        self.pub_trajectory(traj)
        time.sleep(duration1 + duration2)

        # Send Home
        self.send_trajectory(home_positions, 3)

    def pub_trajectory(self, traj):
        self.pub.publish(traj)

    def send_trajectory(self, positions, duration=2.0):
        """Sends a joint trajectory command"""
        traj = JointTrajectory()
        traj.joint_names = list(self.joint_limits.keys())
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(seconds=duration).to_msg()
        traj.points.append(point)

        self.pub.publish(traj)
        # self.get_logger().info("Trajectory sent.")
        if duration <= 0.05:
            duration = duration + 0.05
        time.sleep(duration - 0.05)  # Ensure execution completion


def main():
    rclpy.init()
    arm = DynaArm()
    # arm.move_to_zero(speed=0.1)

    arm.switch_controller(["freeze_controller"], ["joint_trajectory_controller"])
    arm.move_joints()
    arm.switch_controller(["joint_trajectory_controller"], ["freeze_controller"])

    # arm.move_to_zero(speed=0.1)
    arm.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

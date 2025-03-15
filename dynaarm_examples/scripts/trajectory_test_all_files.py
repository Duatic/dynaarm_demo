import os
import rclpy
from trajectory_test_one_file import DynaArm  # Import the existing class
from ament_index_python.packages import get_package_share_directory
import glob
import time

def main():
    rclpy.init()
    arm = DynaArm()  # Reuse the existing class

    # Define the directory containing the trajectory files
    package_path = get_package_share_directory("dynaarm_examples")
    trajectory_dir = os.path.join(package_path, "scripts", "trajectory_data")

    # Find all CSV trajectory files
    trajectory_files = sorted(glob.glob(os.path.join(trajectory_dir, "trajectory_data_config*.csv")))

    if not trajectory_files:
        arm.get_logger().error("No trajectory files found in trajectory_data/!")
        return

    arm.get_logger().info(f"Executing {len(trajectory_files)} trajectory files.")

    # Enable trajectory controller
    arm.switch_controller(['freeze_controller'], ['joint_trajectory_controller'])

    for i, trajectory_file in enumerate(trajectory_files):
        arm.get_logger().info(f"Executing trajectory {i+1}/{len(trajectory_files)}: {trajectory_file}")

        # Move to start position before executing the trajectory
        arm.move_to_start()

        # Execute the trajectory using the existing move_joints() function
        arm.move_joints(trajectory_file)

        # Small delay before next trajectory
        arm.get_logger().info("Waiting 2 seconds before next trajectory...")
        time.sleep(2)

    # Disable trajectory controller after execution
    arm.switch_controller(['joint_trajectory_controller'], ['freeze_controller'])

    arm.get_logger().info("All trajectories executed successfully!")
    arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

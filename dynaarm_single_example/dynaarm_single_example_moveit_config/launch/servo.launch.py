import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
   
    moveit_config = (
        MoveItConfigsBuilder("dynaarm", package_name="dynaarm_single_example_moveit_config")
        .robot_description()
        .joint_limits()
        .to_moveit_configs()
    )
    
    # Get parameters for the Servo node
    package_name = "dynaarm_single_example_moveit_config"
    package_path = get_package_share_directory(package_name)
    servo_yaml_path = os.path.join(package_path, "config", "servo.yaml")

    servo_params = ParameterBuilder(package_name) \
        .yaml(servo_yaml_path, parameter_namespace="") \
        .to_dict()

    # This sets the update rate and planning group name for the acceleration limiting filter.
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "dynaarm"}
    
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen"
    )

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo")
        + "/config/demo_rviz_config_ros.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )    

    return launch.LaunchDescription(
        [
            rviz_node,
            servo_node,
        ]
    )
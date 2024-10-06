import os
import yaml

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from rclpy.impl import rcutils_logger

def _load_controllers_from_yaml(file_path: str) -> list[ExecuteProcess]:
    # a real ros logger for configuration errors and debug during launch
    logger = rcutils_logger.RcutilsLogger(name='stack_launcher')

    # require a valid yaml file with optional controller config section
    initial_controller_states = {}
    try:
        with open(file_path, 'r') as file:
            data = yaml.safe_load(file)
            if 'stack_launcher' in data:
                initial_controller_states = data['stack_launcher']['ros__parameters']['ros2controllers']
            else:
                print(
                    f'File: {file_path} does not contain any controller to activate')
    except KeyError as e:
        # warn in the log just in case the developer has made a typo in the yaml
        raise e

    # configure the desired controllers to launch in their configured initial states
    controllers = []
    for controller_name, initial_state in initial_controller_states.items():
        logger.info(
            f'{controller_name} initial state configured as {initial_state}')
        controllers.append(ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller',
                 '--set-state', initial_state, controller_name],
            output='screen')
        )
    return controllers


def _robot_state_publisher_for_joint_states(urdf_cmd: Command) -> Node:
    return Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf_cmd}],
        remappings=[('/joint_states', '/dynaarm_state_broadcaster/joint_states')],
    )


def _controller_manager(controller_files: list[str], urdf_cmd: Command) -> Node:
    return Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=controller_files + [{'robot_description': urdf_cmd}],
        output='screen',
        emulate_tty=True,
    )


def _configuration_files(driver_config: str) -> list[str]:
    return [
        os.path.join(driver_config, 'ros2control', 'controllers.yaml'),
        os.path.join(driver_config, 'ros2control', 'common.yaml'),
    ]

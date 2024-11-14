# Copyright 2024 Duatic AG
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions, and
#    the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions, and
#    the following disclaimer in the documentation and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
#    promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from moveit_configs_utils import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):
        
    ethercat_bus = LaunchConfiguration("ethercat_bus")
    start_rviz = LaunchConfiguration("start_rviz")
    use_fake = LaunchConfiguration("use_fake")

    ethercat_bus_value = ethercat_bus.perform(context)    
    use_fake_value = use_fake.perform(context)

    launch_arguments = {
        "use_fake": use_fake_value,        
        'ethercat_bus': ethercat_bus_value
    }

    dynaarm_moveit_pkg = f"dynaarm_moveit_config"
    dynaarm_driver_pkg = f"dynaarm_driver"

    dynaarm_driver_pkg_share = FindPackageShare('dynaarm_description').find('dynaarm_description')    
    joint_limits_file_path = os.path.join(dynaarm_driver_pkg_share, 'config', 'joint_limits.yaml')    

    moveit_config = (
        MoveItConfigsBuilder("dynaarm", package_name=dynaarm_moveit_pkg)
        .robot_description(mappings=launch_arguments)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True)        
        .joint_limits(joint_limits_file_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )    

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(dynaarm_driver_pkg),
            "config",
            f"controllers.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, robot_controllers],        
        output={
            'stdout': 'screen',
            'stderr': 'screen',
        },
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    dynaarm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dynaarm_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
   
    # RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(dynaarm_moveit_pkg), "config", "moveit.rviz"]
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
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits
        ],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(start_rviz),
    )
    
    nodes_to_start = [        
        control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        dynaarm_controller_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        move_group_node,
    ]

    return nodes_to_start

def generate_launch_description():
    
    declared_arguments = []  
    declared_arguments.append(
        DeclareLaunchArgument(
            "ethercat_bus",
            default_value='enp86s0',
            description='The ethercat bus id or name.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake",
            default_value='true',
            description='Use fake hardware.',
        )
    )   
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 automatically with this launch file.',
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
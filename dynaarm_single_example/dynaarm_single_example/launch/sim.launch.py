# Copyright 2025 Duatic AG
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
from ament_index_python import get_package_share_directory
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    dof = LaunchConfiguration("dof")
    covers = LaunchConfiguration("covers")
    version = LaunchConfiguration("version")
    gui = LaunchConfiguration("gui")

    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
    version_value = version.perform(context)

    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    pkg_share_description = FindPackageShare(package="dynaarm_single_example_description").find(
        "dynaarm_single_example_description"
    )

    doc = xacro.parse(
        open(os.path.join(pkg_share_description, "urdf/dynaarm_single_example.urdf.xacro"))
    )
    xacro.process_doc(
        doc,
        mappings={
            "dof": dof_value,
            "covers": covers_value,
            "version": version_value,
            "mode": "sim",
        },
    )
    robot_description = {"robot_description": doc.toxml()}

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # Launch RViz
    pkg_share_description_base = FindPackageShare(package="dynaarm_description").find(
        "dynaarm_description"
    )
    rviz_config_file = PathJoinSubstitution([pkg_share_description_base, "config/config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(gui),
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    world_file = os.path.join(pkg_share_description, "worlds", "custom.sdf")
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r -v 2 " + world_file], "on_exit_shutdown": "true"}.items(),
    )

    # Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string",
            doc.toxml(),
            "-name",
            "dynaarm",
        ],
        output="both",
    )

    # Bridge for Gazebo topics
    bridge_params = os.path.join(
        get_package_share_directory("dynaarm_single_example"), "config", "gz_bridge.yaml"
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_ros_spawner_cmd,
            on_exit=[joint_state_broadcaster_spawner_node],
        )
    )

    joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    cartesian_motion_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "--inactive"],
    )

    # freedrive_controller_node = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["freedrive_controller", "--inactive"],
    # )

    # The controller to start variable
    delay_startup_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[
                joint_trajectory_controller_node,
                cartesian_motion_controller_node,
                # freedrive_controller_node,
            ],
        )
    )

    # Start the joy node for gamepad input
    joy_node = Node(
        package="joy",
        executable="game_controller_node",
        output="screen",
        parameters=[{"autorepeat_rate": 100.0}],  # Set autorepeat to 100 Hz
    )

    move_to_predefined_position_node = Node(
        package="dynaarm_extensions",
        executable="move_to_predefined_position_node",
        name="move_to_predefined_position_node",
        output="screen",
        parameters=[{"robot_configuration": "dynaarm"}],
    )

    nodes_to_start = [
        joy_node,
        start_gazebo_cmd,
        gz_bridge,
        robot_state_pub_node,
        rviz_node,
        start_gazebo_ros_spawner_cmd,
        delay_joint_state_broadcaster,
        delay_startup_controller,
        move_to_predefined_position_node,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="dof",
            choices=["1", "2", "3", "4", "5", "6"],
            default_value="6",
            description="Select the desired degrees of freedom (dof)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="covers",
            default_value="False",
            description="Show or hide the covers of the robot",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="version",
            default_value="baracuda12",
            choices=["arowana4", "baracuda12"],
            description="Select the desired version of robot ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="False",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

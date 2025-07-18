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
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    gui = LaunchConfiguration("gui")
    covers = LaunchConfiguration("covers")
    version_1 = LaunchConfiguration("version_1")
    version_2 = LaunchConfiguration("version_2")

    covers_value = covers.perform(context)
    version_1_value = version_1.perform(context)
    version_2_value = version_2.perform(context)

    # Load the robot description
    pkg_share_description = FindPackageShare(package="dynaarm_dual_example_description").find(
        "dynaarm_dual_example_description"
    )
    doc = xacro.parse(
        open(os.path.join(pkg_share_description, "urdf/dynaarm_dual_example.urdf.xacro"))
    )
    xacro.process_doc(
        doc,
        mappings={
            "covers": covers_value,
            "version_1": version_1_value,
            "version_2": version_2_value,
            "mode": "mock",
        },
    )
    robot_description = {"robot_description": doc.toxml()}

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dynaarm_dual_example"),
            "config",
            "controllers.yaml",
        ]
    )

    joy_node = Node(
        package="joy",
        executable="game_controller_node",
        output="screen",
        parameters=[{"autorepeat_rate": 100.0}],  # Set autorepeat to 100 Hz
    )

    e_stop_node = Node(
        package="dynaarm_extensions",
        executable="e_stop_node",
        name="e_stop_node",
        output="screen",
        parameters=[{"emergency_stop_button": 9}],  # Change button index here
    )

    move_to_predefined_position_node = Node(
        package="dynaarm_extensions",
        executable="move_to_predefined_position_node",
        name="move_to_predefined_position_node",
        output="screen",
        parameters=[{"robot_configuration": "dynaarm_dual"}],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    status_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dynaarm_status_broadcaster_arm_1", "dynaarm_status_broadcaster_arm_2"],
    )

    freeze_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["freeze_controller_arm_1", "freeze_controller_arm_2"],
    )

    gravity_compensation_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gravity_compensation_controller_arm_1",
            "gravity_compensation_controller_arm_2",
        ],
    )

    freedrive_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["freedrive_controller_arm_1", "freedrive_controller_arm_2", "--inactive"],
    )

    joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller_arm_1",
            "joint_trajectory_controller_arm_2",
            "--inactive",
        ],
    )

    delay_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[
                rviz_node,
                status_broadcaster_node,
                freeze_controller_node,
                gravity_compensation_controller_node,
                joint_trajectory_controller_node,
                freedrive_controller_node,
            ],
        )
    )

    nodes_to_start = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner_node,
        delay_after_joint_state_broadcaster_spawner,
        joy_node,
        e_stop_node,
        move_to_predefined_position_node,
    ]

    return nodes_to_start


def generate_launch_description():

    # Declare the launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="gui",
            default_value="True",
            choices=["True", "False"],
            description="Flag to enable joint_state_publisher_gui",
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
            name="version_1",
            default_value="baracuda12",
            choices=["arowana4", "baracuda12"],
            description="Select the desired version of robot 1 ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            name="version_2",
            default_value="baracuda12",
            choices=["arowana4", "baracuda12"],
            description="Select the desired version of robot 2",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

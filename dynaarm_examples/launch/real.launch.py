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
import xacro
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    ethercat_bus = LaunchConfiguration("ethercat_bus")
    dof = LaunchConfiguration("dof")
    covers = LaunchConfiguration("covers")
    version = LaunchConfiguration("version")
    start_rviz = LaunchConfiguration("start_rviz")

    ethercat_bus_value = ethercat_bus.perform(context)
    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
    version_value = version.perform(context)

    # Load the robot description
    pkg_share_description = FindPackageShare(package="dynaarm_description").find(
        "dynaarm_description"
    )
    doc = xacro.parse(
        open(os.path.join(pkg_share_description, "urdf/dynaarm_standalone.urdf.xacro"))
    )
    xacro.process_doc(
        doc,
        mappings={
            "ethercat_bus": ethercat_bus_value,
            "dof": dof_value,
            "covers": covers_value,
            "version": version_value,
            "mode": "real",
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
    rviz_config_file = PathJoinSubstitution([pkg_share_description, "config", "config.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(start_rviz),
    )

    joint_state_broadcaster_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("dynaarm_examples"),
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
        arguments=["dynaarm_status_broadcaster"],
    )

    freeze_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["freeze_controller"],
    )

    gravity_compensation_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gravity_compensation_controller"],
    )

    freedrive_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["freedrive_controller", "--inactive"],
    )

    pid_tuner_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["pid_tuner", "--inactive"],
    )

    joint_trajectory_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "--inactive"],
    )

    cartesian_motion_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_motion_controller", "--inactive"],
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
                cartesian_motion_controller_node,
                freedrive_controller_node,
                pid_tuner_node,
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
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="ethercat_bus",
            default_value="enp86s0",
            description="The ethercat bus id or name.",
        )
    )
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
            "start_rviz",
            default_value="True",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

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
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Package Directories
    pkg_duatic_visualization = FindPackageShare("duatic_visualization")
    pkg_dynaarm_bringup = FindPackageShare("dynaarm_bringup")
    pkg_dynaarm_description = FindPackageShare("dynaarm_description")
    pkg_dynaarm_dual_example_description = FindPackageShare("dynaarm_dual_example_description")

    # Robot State Publisher
    doc = xacro.parse(
        open(
            os.path.join(
                pkg_dynaarm_dual_example_description, "urdf/dynaarm_dual_example.urdf.xacro"
            )
        )
    )
    xacro.process_doc(
        doc,
        mappings={
            "ethercat_bus_1": LaunchConfiguration("ethercat_bus_1").perform(context),
            "ethercat_bus_2": LaunchConfiguration("ethercat_bus_2").perform(context),
            "covers": LaunchConfiguration("covers").perform(context),
            "version_1": LaunchConfiguration("version").perform(context),
            "version_2": LaunchConfiguration("version").perform(context),
            "mode": "real",
        },
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": doc.toxml()}],
        namespace=LaunchConfiguration("namespace"),
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Dynaarm Left Bringup
    dynaarm_left_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dynaarm_bringup, "launch", "real.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "start_as_subcomponent": "true",
            "tf_prefix": "arm_left",
        }.items(),
    )

    # Dynaarm Right Bringup
    dynaarm_right_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dynaarm_bringup, "launch", "real.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "start_as_subcomponent": "true",
            "tf_prefix": "arm_right",
        }.items(),
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=LaunchConfiguration("namespace"),
        parameters=[{"update_rate": 1000}],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Show RVIZ
    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_duatic_visualization, "launch", "rviz.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "use_sim_time": "true",
            "rviz_config": PathJoinSubstitution([pkg_dynaarm_description, "config", "config.rviz"]),
        }.items(),
    )

    # Gamepad input
    joy_node = Node(
        package="joy",
        namespace=LaunchConfiguration("namespace"),
        executable="game_controller_node",
        output="screen",
        parameters=[{"autorepeat_rate": 100.0}],
    )

    # Emergency Stop
    e_stop_node = Node(
        package="dynaarm_extensions",
        executable="e_stop_node",
        name="e_stop_node",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        parameters=[{"emergency_stop_button": 9}],  # Change button index here
    )

    # Move Arms to Start Position
    # TODO: Find a better way to delay this node start until controllers are ready
    move_to_predefined_position_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="dynaarm_extensions",
                executable="move_to_predefined_position_node",
                namespace=LaunchConfiguration("namespace"),
                name="move_to_predefined_position_node",
                output="screen",
                parameters=[{"robot_configuration": "dynaarm_dual"}],
            )
        ],
    )

    nodes_to_start = [
        robot_state_publisher,
        dynaarm_left_bringup,
        dynaarm_right_bringup,
        controller_manager,
        rviz,
        joy_node,
        e_stop_node,
        move_to_predefined_position_node,
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            name="ethercat_bus_1",
            default_value="enp86s0",
            description="The ethercat bus id or name of the first robot.",
        ),
        DeclareLaunchArgument(
            name="ethercat_bus_2",
            default_value="enp86s1",
            description="The ethercat bus id or name of the second robot.",
        ),
        DeclareLaunchArgument(
            name="covers",
            default_value="False",
            description="Show or hide the covers of the robot",
        ),
        DeclareLaunchArgument(
            name="version",
            default_value="baracuda12",
            choices=["arowana4", "baracuda12"],
            description="Select the desired version of robot 1",
        ),
        DeclareLaunchArgument(
            "start_rviz",
            default_value="True",
            description="Start RViz2 automatically with this launch file.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

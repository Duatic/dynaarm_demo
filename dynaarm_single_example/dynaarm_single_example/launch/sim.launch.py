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

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    # Package Directories
    pkg_duatic_simulation = FindPackageShare("duatic_simulation")
    pkg_dynaarm_bringup = FindPackageShare("dynaarm_bringup")
    pkg_dynaarm_description = FindPackageShare("dynaarm_description")

    # Dynaarm Bringup
    dynaarm_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_dynaarm_bringup, "launch", "sim.launch.py"])
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace"),
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    # Show RVIZ
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=LaunchConfiguration("namespace"),
        arguments=["-d", PathJoinSubstitution([pkg_dynaarm_description, "config", "config.rviz"])],
        output={"both": "log"},
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Gazebo Simulation
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_duatic_simulation, "launch", "gazebo.launch.py"])
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
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
                parameters=[{"robot_configuration": "dynaarm"}],
            )
        ],
    )

    nodes_to_start = [simulation, dynaarm_bringup, rviz, joy_node, move_to_predefined_position_node]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = [
        DeclareLaunchArgument(
            name="namespace",
            default_value="",
        ),
        DeclareLaunchArgument(name="world", default_value="duatic_empty", description="World name"),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

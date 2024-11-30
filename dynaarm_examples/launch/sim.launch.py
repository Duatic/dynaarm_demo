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
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")

    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
    version_value = version.perform(context)

    pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
    pkg_share_description = FindPackageShare(package="dynaarm_description").find(
        "dynaarm_description"
    )

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
        parameters=[{"use_sim_time": use_sim_time}, robot_description],
    )

    # Launch RViz
    rviz_config_file = PathJoinSubstitution([pkg_share_description, "rviz", "config.rviz"])
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

    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments=[
            ("gz_args", [" -r -v 4 empty.sdf"]),
        ],
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

    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_gazebo_ros_spawner_cmd,
            on_exit=[joint_state_broadcaster_spawner_node],
        )
    )

    startup_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    # The controller to start variable
    delay_startup_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner_node,
            on_exit=[startup_controller_node],
        )
    )

    nodes_to_start = [
        start_gazebo_cmd,
        robot_state_pub_node,
        rviz_node,
        start_gazebo_ros_spawner_cmd,
        delay_joint_state_broadcaster,
        delay_startup_controller,
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
            default_value="v2",
            choices=["v1", "v2"],
            description="Select the desired version of robot ",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use simulated time.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="False",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

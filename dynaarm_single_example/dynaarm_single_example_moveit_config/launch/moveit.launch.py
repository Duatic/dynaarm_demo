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
    RegisterEventHandler,
    OpaqueFunction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python import get_package_share_directory
import os
import xacro


def launch_setup(context, *args, **kwargs):

    start_rviz = LaunchConfiguration("start_rviz")
    dof = LaunchConfiguration("dof")
    covers = LaunchConfiguration("covers")
    version = LaunchConfiguration("version")
    ethercat_bus = LaunchConfiguration("ethercat_bus")
    mode = LaunchConfiguration("mode")

    ethercat_bus_value = ethercat_bus.perform(context)
    mode_value = mode.perform(context)
    dof_value = dof.perform(context)
    covers_value = covers.perform(context)
    version_value = version.perform(context)

    launch_arguments = {
        "dof": dof_value,
        "mode": mode_value,
        "ethercat_bus": ethercat_bus_value,
        "covers": covers_value,
        "version": version_value,
    }

    dynaarm_moveit_pkg = "dynaarm_single_example_moveit_config"
    dynaarm_examples_pkg = "dynaarm_single_example"

    moveit_config = (
        MoveItConfigsBuilder("dynaarm", package_name=dynaarm_moveit_pkg)
        .robot_description(mappings=launch_arguments)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare(dynaarm_examples_pkg),
            "config",
            "controllers.yaml",
        ]
    )

    nodes_to_start = []

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    if mode_value in ("mock", "real"):

        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[moveit_config.robot_description, robot_controllers],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        )

        nodes_to_start.append(control_node)

        nodes_to_start.append(joint_state_broadcaster_spawner)
        nodes_to_start.append(joint_trajectory_controller_spawner)
    else:
        pkg_share_description = FindPackageShare(package="dynaarm_single_example_description").find(
            "dynaarm_single_example_description"
        )
        pkg_ros_gz_sim = FindPackageShare(package="ros_gz_sim").find("ros_gz_sim")
        world_file = os.path.join(pkg_share_description, "worlds", "custom.sdf")
        start_gazebo_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
            ),
            launch_arguments={
                "gz_args": ["-r -v 2 " + world_file],
                "on_exit_shutdown": "true",
            }.items(),
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
        nodes_to_start.extend([start_gazebo_cmd, start_gazebo_ros_spawner_cmd, gz_bridge])

        delay_joint_state_broadcaster = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_gazebo_ros_spawner_cmd,
                on_exit=[joint_state_broadcaster_spawner],
            )
        )

        nodes_to_start.append(delay_joint_state_broadcaster)

        # The controller to start variable
        delay_startup_controller = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    joint_trajectory_controller_spawner,
                    # freedrive_controller_node,
                ],
            )
        )
        nodes_to_start.append(delay_startup_controller)

    moveit_config_dict = moveit_config.to_dict()
    moveit_config_dict["use_sim_time"] = mode_value == "sim"
    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config_dict],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": mode_value == "sim"}, moveit_config.robot_description],
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

    nodes_to_start.extend(
        [
            robot_state_publisher,
            delay_rviz_after_joint_state_broadcaster_spawner,
            move_group_node,
        ]
    )

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            name="start_rviz",
            default_value="True",
            description="Start RViz2 automatically with this launch file.",
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
            "ethercat_bus",
            default_value="enp0s31f6",
            description="The ethercat bus id or name.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="mock",
            choices=["real", "mock", "sim"],
            description="Set the desired hardware mode.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

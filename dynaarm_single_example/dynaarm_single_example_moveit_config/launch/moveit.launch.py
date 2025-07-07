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
)
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from moveit_configs_utils import MoveItConfigsBuilder
from launch_param_builder import ParameterBuilder
import os


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

    # Depending on the mode we simply start one of the other launch files
    nodes_to_start = []

    if mode_value == "mock":
        pkg_demo_mock = FindPackageShare(package="dynaarm_single_example").find(
            "dynaarm_single_example"
        )

        start_demo_mock = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_demo_mock, "launch", "mock.launch.py")),
            launch_arguments={
                "dof": dof_value,
                "gui": "False",
                "covers": covers_value,
                "version": version_value,
            }.items(),
        )

        nodes_to_start.append(start_demo_mock)

    elif mode_value == "real":
        pkg_demo_real = FindPackageShare(package="dynaarm_single_example").find(
            "dynaarm_single_example"
        )

        start_demo_real = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_demo_real, "launch", "real.launch.py")),
            launch_arguments={
                "dof": dof_value,
                "gui": "False",
                "covers": covers_value,
                "version": version_value,
                "ethercat_bus": ethercat_bus_value,
            }.items(),
        )

        nodes_to_start.append(start_demo_real)
    elif mode_value == "sim":
        pkg_demo_sim = FindPackageShare(package="dynaarm_single_example").find(
            "dynaarm_single_example"
        )

        start_demo_sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_demo_sim, "launch", "sim.launch.py")),
            launch_arguments={
                "dof": dof_value,
                "gui": "False",
                "covers": covers_value,
                "version": version_value,
            }.items(),
        )
        nodes_to_start.append(start_demo_sim)

    # After starting the backend we start a movegroupd node and start the corresponding rviz ui
    moveit_config = (
        MoveItConfigsBuilder("dynaarm", package_name=dynaarm_moveit_pkg)
        .robot_description(mappings=launch_arguments)
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

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
    acceleration_filter_update_period = {"update_period": 0.01}
    planning_group_name = {"planning_group_name": "dynaarm"}

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("dynaarm_single_example_moveit_config")
        .yaml("config/dynaarm_servo_config.yaml")
        .to_dict()
    }

    moveit_servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        output="screen",
        parameters=[
            servo_params,
            acceleration_filter_update_period,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": mode_value == "sim"},
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
        ],
        condition=IfCondition(start_rviz),
    )

    nodes_to_start.extend([rviz_node, move_group_node, moveit_servo_node])

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
            default_value="enx70886b8adda2",
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

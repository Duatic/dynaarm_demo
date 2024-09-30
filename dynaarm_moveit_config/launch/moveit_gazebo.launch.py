import os
import xacro
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
    
    start_rviz = LaunchConfiguration("start_rviz")        


    pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
    dynaarm_driver_pkg_share = FindPackageShare('dynaarm_description').find('dynaarm_description')    
    joint_limits_file_path = os.path.join(dynaarm_driver_pkg_share, 'config', 'joint_limits.yaml')  

    dynaarm_moveit_pkg = f"dynaarm_moveit_config"
    
    launch_arguments = {
        "use_gazebo": 'true',
        "use_fake": 'true',        
        'ethercat_bus': "not-needed"
    }

    moveit_config = (
        MoveItConfigsBuilder("dynaarm", package_name=dynaarm_moveit_pkg)
        .robot_description(mappings=launch_arguments)
        .planning_scene_monitor(
            publish_robot_description=True, 
            publish_robot_description_semantic=True)  
        .joint_limits(joint_limits_file_path)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )    

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},],
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
            moveit_config.joint_limits,            
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

    start_gazebo_cmd = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments=[('gz_args', [' -r -v 4 empty.sdf']),])

	# Spawn the robot
    start_gazebo_ros_spawner_cmd = Node(
		package='ros_gz_sim',
		executable='create',    
		arguments=[
			'-topic', 'robot_description',
			'-name', 'dynaarm',      
		],
		output='both')
    
    nodes_to_start = [           
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        move_group_node,
        dynaarm_controller_spawner,        
        start_gazebo_cmd,                
        start_gazebo_ros_spawner_cmd
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
            "use_gazebo",
            default_value='false',
            description='Use gazebo hardware.',
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
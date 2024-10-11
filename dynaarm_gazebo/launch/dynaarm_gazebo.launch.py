# Author: Timo Schwarzer
# Date: December 22, 2023
# Description: Launch a Mitsubishi RV-2FR robot URDF file using Rviz.
# http://www.robolynk.de

import os
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, OpaqueFunction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
		
	ethercat_bus = LaunchConfiguration("ethercat_bus")
	start_rviz = LaunchConfiguration("start_rviz")	
	use_sim_time = LaunchConfiguration("use_sim_time")
	use_fake = LaunchConfiguration("use_fake")

	ethercat_bus_value = ethercat_bus.perform(context)
	use_fake_value = use_fake.perform(context)

	pkg_ros_gz_sim = FindPackageShare(package='ros_gz_sim').find('ros_gz_sim')  
	pkg_share_description = FindPackageShare(package='dynaarm_description').find('dynaarm_description')
	
	rviz_config_file = PathJoinSubstitution([FindPackageShare("dynaarm_description"), "rviz", "config.rviz"])	

	doc = xacro.parse(open(os.path.join(pkg_share_description, 'urdf/dynaarm.xacro')))
	xacro.process_doc(doc, mappings={'use_fake': use_fake_value,
								  	 'ethercat_bus': ethercat_bus_value,
									 'use_gazebo': 'true'})	
	robot_description = {'robot_description': doc.toxml()}    

	# Subscribe to the joint states of the robot, and publish the 3D pose of each link.
	robot_state_pub_node = Node(
		  package='robot_state_publisher',
		  executable='robot_state_publisher',
		  output='both',
		  parameters=[
			  {'use_sim_time': use_sim_time},
			  robot_description
		]
	)

	# Launch RViz
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
		PythonLaunchDescriptionSource(
			os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
		launch_arguments=[('gz_args', [' -r -v 4 empty.sdf']),])

	# Spawn the robot
	start_gazebo_ros_spawner_cmd = Node(
		package='ros_gz_sim',
		executable='create',    
		arguments=[
			'-string', doc.toxml(),
			'-name', 'dynaarm',      
		],
		output='both')
	
	delay_joint_state_broadcaster = RegisterEventHandler(
		event_handler=OnProcessExit(
			target_action=start_gazebo_ros_spawner_cmd,
			on_exit=[joint_state_broadcaster_spawner_node],
		)
	)

	#startup_controller_name = 'position_controller'
	startup_controller_name = 'joint_trajectory_controller'    
    #startup_controller_name = 'gravity_compensation_controller'

	startup_controller_node = Node(
		package="controller_manager",
        executable="spawner",
        arguments=[startup_controller_name, "-c", "/controller_manager"],
    )

	# The controller to start variable    
	delay_startup_controller = RegisterEventHandler(
			event_handler=OnProcessExit(
				target_action=joint_state_broadcaster_spawner_node,
				on_exit=[startup_controller_node]))

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
			"ethercat_bus",
			default_value='enp86s0',
			description='The ethercat bus id or name.',
		)
	)	
	declared_arguments.append(
		DeclareLaunchArgument(
			"use_fake",
			default_value='true',
			description='Use gazebo simulation.',
		)
	)
	declared_arguments.append(
		DeclareLaunchArgument(
			"use_sim_time",
			default_value='true',
			description='Use simulated time.',
		)
	)
	declared_arguments.append(
		DeclareLaunchArgument(
			'start_rviz',
			default_value='false',
			description='Start RViz2 automatically with this launch file.',
		)
	)

	return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
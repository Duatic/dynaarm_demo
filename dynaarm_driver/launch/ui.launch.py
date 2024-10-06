import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import Shutdown, DeclareLaunchArgument, OpaqueFunction

from launch_ros.actions import Node

DRIVER_PACKAGE = os.path.join(get_package_share_directory('dynaarm_driver'))
DRIVER_PACKAGE_CONFIG = os.path.join(DRIVER_PACKAGE, 'config')


def generate_launch_description():
    # Declare Lists that will be added to the LaunchDescription
    nodes = []

    rqt = Node(
        package='rqt_gui',
        executable='rqt_gui',
        output='screen',
        arguments=[
            '--force-discover',
            '--perspective-file',
            os.path.join(DRIVER_PACKAGE_CONFIG, 'ui', 'rqt.perspective'),
        ],
        on_exit=Shutdown(),
        sigterm_timeout='0',
        sigkill_timeout='0',
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(DRIVER_PACKAGE_CONFIG, 'ui/config.rviz'),
        ],
        output='screen',
        sigterm_timeout='0',
        sigkill_timeout='0',
    )

    joy_node = Node(
        package='joy',
        executable='game_controller_node',
        name='joy',
        output='screen',
        parameters=[{'autorepeat_rate': 0.0, 'coalesce_interval_ms': 5}],
    )

    # Add all nodes
    nodes.append(rqt)
    nodes.append(rviz2)
    nodes.append(joy_node)

    # Add all opaque functions

    return LaunchDescription(nodes)

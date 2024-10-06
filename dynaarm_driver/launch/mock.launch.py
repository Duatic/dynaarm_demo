import os
from launch import LaunchDescription
from launch.actions import OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command

from ament_index_python.packages import get_package_share_directory

from dynaarm_driver import common

HW_INTERFACE_TYPE = 'mock'
DYNAARM_CONFIGURATION = '6dof'

DRIVER_PACKAGE = os.path.join(get_package_share_directory('dynaarm_driver'))
DRIVER_PACKAGE_CONFIG = os.path.join(DRIVER_PACKAGE, 'config')
URDF_DESCRIPTION_FILE = os.path.join(get_package_share_directory('dynaarm_standalone_description'), 'resource', 'xacro', 'dynaarm_standalone.urdf.xacro')
UI_LAUNCH_FILE = os.path.join(DRIVER_PACKAGE, 'launch', 'ui.launch.py')

def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=load_nodes_and_processes)])


def load_nodes_and_processes(context):
    urdf_cmd = Command(
        [f'xacro {URDF_DESCRIPTION_FILE} hardware_interface_type_arg:={HW_INTERFACE_TYPE} dynaarm_dof_configuration_arg:={DYNAARM_CONFIGURATION}'])
    files = common._configuration_files(DRIVER_PACKAGE_CONFIG)
    nodes_and_processes = [
        common._robot_state_publisher_for_joint_states(urdf_cmd),
        common._controller_manager(files, urdf_cmd)
    ]
    for f in files:
        nodes_and_processes.extend(common._load_controllers_from_yaml(f))

    nodes_and_processes.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource([UI_LAUNCH_FILE])
    )
)
    return nodes_and_processes

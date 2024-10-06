from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import Command

from cs_launch_utils import cs_launch_utils

from dynaarm_driver.common import (
    ROBOT_NAME,
    URDF_DESCRIPTION_FILE,
    DRIVER_PACKAGE_CONFIG,
    UI_LAUNCH_FILE,
    generate_urdfs,
)

HW_INTERFACE_TYPE = 'sim'


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=load_nodes_and_processes),
    ])


def load_nodes_and_processes(context):
    urdf_cmd = Command(
        [f'xacro {URDF_DESCRIPTION_FILE} hardware_interface_type:={HW_INTERFACE_TYPE} umv_configuration:={ROBOT_NAME}'])
    processes = cs_launch_utils.nodes_and_processes(
        driver_config=DRIVER_PACKAGE_CONFIG,
        urdf_cmd=urdf_cmd,
        cm_config_file=HW_INTERFACE_TYPE,
        cm_type=cs_launch_utils.ControllerManagerType.SIMULATION,
    )
    processes.append(
        cs_launch_utils.create_ui_launch_description(UI_LAUNCH_FILE))
    return generate_urdfs() + processes

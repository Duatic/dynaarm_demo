from launch import LaunchDescription
from launch.actions import (
    OpaqueFunction,
    DeclareLaunchArgument,
)
from launch.substitutions import Command

from cs_launch_utils import cs_launch_utils

from dynaarm_driver.common import (
    ROBOT_NAME,
    URDF_DESCRIPTION_FILE,
    DRIVER_PACKAGE_CONFIG,
    generate_urdfs,
)

HW_INTERFACE_TYPE = 'real'
ODRIVE_COMMAND_MODE = 'position_velocity_torque' # position_velocity_torque or torque
# The friction compensation is experimental. Do not turn this on if you're not testing it
APPLY_FRICTION_COMPENSATION = '0' # 0 (false) or 1 (true)

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware_serial_number'),
        OpaqueFunction(function=load_nodes_and_processes),
    ])


def load_nodes_and_processes(context):
    hardware_serial_number = context.launch_configurations['hardware_serial_number']
    if not hardware_serial_number:
        raise RuntimeError(
            'hardware_serial_number argument is required to launch!')

    urdf_cmd = Command([
        f'xacro {URDF_DESCRIPTION_FILE} umv_configuration:={ROBOT_NAME} hardware_interface_type:={HW_INTERFACE_TYPE} hardware_serial_number:={hardware_serial_number} odrive_command_mode:={ODRIVE_COMMAND_MODE} apply_static_friction_compensation:={APPLY_FRICTION_COMPENSATION} '])

    return generate_urdfs() + cs_launch_utils.nodes_and_processes(
        driver_config=DRIVER_PACKAGE_CONFIG,
        urdf_cmd=urdf_cmd,
        cm_config_file=HW_INTERFACE_TYPE,
    )

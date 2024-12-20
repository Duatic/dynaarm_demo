# dynaarm_demo

This repository contains some simple demo application the [ros2_control](https://control.ros.org/) based driver for the [Duatic DynaArm](https://duatic.com/robotic-arm/)

# License

The contents are licensed under the BSD-3-Clause  [license](LICENSE).\
Images in this repository are to be licensed separately if you want to use them for any other usecase than forking this repository. Please open an issue in order to get in touch with us.

# Dependencies

All dependencies with their corresponding version are listed in the [repos.list](./repos.list).

| Name | Description | License
| ---  | --- | --- |
| [dynaarm_driver](https://github.com/Duatic/dynaarm_driver/) | ros2control based driver for the DynaArm | BSD-3-Clause |
| [dynaarm_description](https://github.com/Duatic/dynaarm_description) | URDF description of the DynaArm | BSD-3-Clause |
| [ethercat_sdk_master](https://github.com/Duatic/ethercat_sdk_master) | Object oriented wrapper around the soem_interface | BSD-3-Clause |
| [rsl_drive_sdk](https://github.com/leggedrobotics/rsl_drive_sdk) | Basic drive sdk for the DynaDrives | BSD-3-Clause |
| [soem_interface](https://github.com/Duatic/soem_interface) | Ethercat wrapper library around SOME | GPL v3 |
| [message_logger](https://github.com/leggedrobotics/message_logger) | Logging library which allows logging with and without ROS | BSD-3-Clause |

# Usage

Please see the DynaArm documentation for a [getting started guide](https://docs.duatic.com)

# Contributing

Please see the [Contributing guide](./CONTRIBUTING.md)

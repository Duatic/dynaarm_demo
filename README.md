# dynaarm_ros2

ROS 2 driver for the dynaarm

# License

The driver is licensed under the BSD-3-Clause [license](LICENSE)

# Usage

# Tooling

## pre-commit

The [pre-commit](https://pre-commit.com/) tool is used for running certain checks and formatters everytime before a commit is done. 
Please use it on any pull requests for this repository. 

__Installation:__

Ubuntu 24.04: `apt install pipx && pipx install pre-commit`. Open a new terminal afterwards.

__Usage:__ 
In the repository folder run: `pre-commit run --all`. This can be automated with `pre-commit install`, so all checks are run everytime a commit is done. 

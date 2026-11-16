#!/bin/bash
# Build the workspace and launch the simulation with a single command.
# Usage (from WSL):
#   ./run_sim.sh             build + launch
#   ./run_sim.sh --clean     delete build/ install/ log/ first, then build + launch
#   ./run_sim.sh rviz:=false any other argument is forwarded to ros2 launch

set -e  # stop at the first failing command (e.g. a compilation error)

# move to the folder containing this script (the repository root = workspace root)
cd "$(dirname "$(readlink -f "$0")")"

# optional clean build, useful when old build artifacts break colcon
if [ "$1" == "--clean" ]; then
    rm -rf build install log
    shift  # remove --clean from the arguments forwarded to ros2 launch
fi

# ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# let Gazebo find the instructor's aruco_box_1..4 models (~/.bashrc is not read
# by scripts); the group's custom models in assignment/models are added by
# sim.launch.py
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/gazebo_models"

# compile all packages (assignment_interfaces, erl1, assignment)
colcon build --symlink-install

# overlay the freshly built packages and start Gazebo + aruco_detector + FSM
source install/setup.bash
ros2 launch assignment sim.launch.py "$@"
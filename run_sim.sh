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

# kill every process left over by a previous simulation.
# On WSL, Ctrl+C often leaves the Gazebo server ("gz sim") and the bridges
# running in the background: a second server then answers the spawn request
# (the robot appears in RViz but not in the visible Gazebo) and two physics
# engines publish /clock and /odom at the same time (the robot "glitches").
cleanup() {
    for proc in "gz sim" parameter_bridge image_bridge robot_state_publisher \
                ekf_node rviz2 aruco_detector fsm_node; do
        pkill -f "$proc" 2>/dev/null || true   # || true: no match is not an error
    done
    sleep 1  # give the processes time to exit
}
cleanup
# also clean up when the launch ends (Ctrl+C included)
trap cleanup EXIT

# ROS 2 Jazzy environment
source /opt/ros/jazzy/setup.bash

# let Gazebo find the instructor's aruco_box_1..4 models in ~/gazebo_models
# (~/.bashrc is not read by scripts)
export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$HOME/gazebo_models"

# compile all packages (assignment_interfaces, erl1, assignment)
colcon build --symlink-install

# overlay the freshly built packages and start Gazebo + aruco_detector + FSM
source install/setup.bash
ros2 launch assignment sim.launch.py "$@"

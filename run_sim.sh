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
    # take_photo_service included: a stale server left alive by a previous run
    # would answer (or block) the FSM requests in place of the new one
    for proc in "gz sim" parameter_bridge image_bridge robot_state_publisher \
                ekf_node rviz2 aruco_detector fsm_node take_photo_service; do
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

# sim.launch.py starts take_photo_service with prefix='xterm -e'.
# If xterm is not installed, ros2 launch cannot start that process at all:
# the "take_photo" service never appears and the FSM waits for it forever
# at the first marker. When xterm is missing we put a minimal "xterm"
# wrapper on PATH that drops "-e" and runs the node in this terminal.
if ! command -v xterm >/dev/null 2>&1; then
    echo "[run_sim] xterm not found (install it with: sudo apt install xterm)."
    echo "[run_sim] take_photo_service will run in this terminal instead."
    shim_dir="$(mktemp -d)"
    cat > "$shim_dir/xterm" << 'EOF'
#!/bin/bash
# fallback xterm: skip the "-e" option and execute the command directly
[ "$1" == "-e" ] && shift
exec "$@"
EOF
    chmod +x "$shim_dir/xterm"
    export PATH="$shim_dir:$PATH"
fi

# compile all packages (assignment_interfaces, erl1, assignment)
colcon build --symlink-install

# overlay the freshly built packages
source install/setup.bash

# ---------------------------------------------------------------------------
# Start Gazebo + aruco_detector + FSM and stop EVERYTHING cleanly when:
#   - Ctrl+C is pressed (SIGINT)
#   - Ctrl+Z is pressed (SIGTSTP: by default it only suspends the processes,
#     leaving Gazebo and the nodes alive but frozen)
#   - the terminal is closed (SIGHUP) or the script is killed (SIGTERM)
#   - the Gazebo window is closed
# ---------------------------------------------------------------------------
set +e  # from here on a non-zero exit status must not abort the monitor loop

# "set -m" starts the launch in its OWN process group: the terminal keys
# (Ctrl+C / Ctrl+Z) then reach only this script, which decides how to stop it.
# stdin from /dev/null: a background group must never read the terminal.
set -m
ros2 launch assignment sim.launch.py "$@" < /dev/null &
launch_pid=$!   # PID of ros2 launch = ID of its process group
set +m

# clean shutdown of the whole simulation
stop_all() {
    trap '' INT TSTP TERM HUP  # ignore further keys while shutting down
    echo
    echo "[run_sim] $1: stopping all simulation processes..."
    # stop the pause of the monitor loop (wait + 2>/dev/null hide "Killed")
    { kill -KILL "$sleep_pid"; wait "$sleep_pid"; } 2>/dev/null
    # 1) graceful: SIGINT to the whole launch group (ros2 launch + nodes),
    #    SIGCONT in case some process was already suspended
    kill -INT  -- -"$launch_pid" 2>/dev/null
    kill -CONT -- -"$launch_pid" 2>/dev/null
    # 2) wait up to 10 s for ros2 launch to terminate its nodes
    for _ in $(seq 1 10); do
        kill -0 "$launch_pid" 2>/dev/null || break
        sleep 1
    done
    # 3) force-kill anything still alive in the group (e.g. the xterm window)
    kill -KILL -- -"$launch_pid" 2>/dev/null
    exit 0   # the EXIT trap runs cleanup() for processes outside the group
}
trap 'stop_all "Ctrl+C"'  INT
trap 'stop_all "Ctrl+Z"'  TSTP
trap 'stop_all "SIGTERM"' TERM
trap 'stop_all "SIGHUP"'  HUP

# monitor loop: runs while ros2 launch is alive
gz_seen=false
while kill -0 "$launch_pid" 2>/dev/null; do
    if pgrep -f "gz sim" > /dev/null; then
        gz_seen=true                         # Gazebo is up
    elif [ "$gz_seen" == true ]; then
        stop_all "Gazebo closed"             # Gazebo was up and is now gone
    fi
    # interruptible 1 s pause: "wait" returns as soon as a key is trapped
    sleep 1 &
    sleep_pid=$!
    wait "$sleep_pid"
done

# ros2 launch ended by itself (e.g. Gazebo closed -> on_exit_shutdown):
# kill whatever is still alive in its group, then the EXIT trap runs cleanup()
echo "[run_sim] launch terminated: cleaning up..."
kill -KILL -- -"$launch_pid" 2>/dev/null
exit 0

# ER_Assignment1 – ArUco Treasure Hunt

ROS 2 project for the Experimental Robotics Laboratory course's first assignment. A differential drive robot finds four ArUco markers one after the other and carries out the action linked to each one:

| Marker | Action |
|--------|--------|
| 1 | Turn right 90° and move forward |
| 2 | Rotate 180° and move forward |
| 3 | Turn left 90° and move forward |
| 4 | Stop: treasure found |

## Requirements

- Ubuntu 24.04 (native or WSL 2)
- ROS 2 Jazzy/Humble and Gazebo Harmonic (`ros-jazzy-ros-gz`)
- `robot_localization`, `cv_bridge`, `python3-opencv`
- Cyclone DDS middleware: `sudo apt install ros-jazzy-rmw-cyclonedds-cpp`
- Optional: `xterm` (`sudo apt install xterm`) to open the photo service in its own window

## Packages

- `assignment`: the ArUco detector node (Python), the finite state machine (C++), the config and the launch file
- `assignment_interfaces`: the custom messages (`Marker`, `MarkerArray`), the `TakePhoto` service and its server node
- `erl1`: the models' package (Gazebo world, robot model, bridge, EKF)

## Node graph

Rounded boxes are nodes, arrows are topics (solid) and services (dashed). Everything started by `erl1/launch/spawn_assignment.launch.py` (Gazebo, bridges, robot state publisher, EKF, RViz) is shown as a single `erl1` node.

```mermaid
flowchart LR
    erl1(["erl1<br/>Gazebo + bridges + EKF"])
    detector(["aruco_detector<br/>(Python)"])
    fsm(["fsm_node<br/>(C++)"])
    photo(["take_photo_service<br/>(C++)"])

    erl1 -- "/camera/image/compressed" --> detector
    erl1 -- "/camera/image/compressed" --> photo
    erl1 -- "/odometry/filtered" --> fsm
    detector -- "/marker_array" --> fsm
    fsm -- "/cmd_vel" --> erl1
    fsm -. "take_photo (TakePhoto.srv)" .-> photo
    photo -- "/marker_image" --> out[["/marker_image<br/>(rqt_image_view)"]]
```

- `aruco_detector` finds the markers in every camera frame and publishes their id, center and side on `/marker_array`.
- `fsm_node` reads the markers and the filtered odometry, drives the robot through `/cmd_vel` and calls the `take_photo` service when it reaches a marker.
- `take_photo_service` draws a circle and the "id N" label on the current frame and publishes it once on `/marker_image`.

Both camera consumers subscribe **only to the compressed (JPEG) stream** (`sensor_msgs/CompressedImage`); the raw image is never transferred. The full topic name is the `image_topic` parameter in `config/sim.yaml` (`/camera/image/compressed`) and `config/rosbot.yaml` (`/camera/rgb/image_raw/compressed`).

## DDS middleware: Cyclone DDS

All nodes use **Cyclone DDS** (`rmw_cyclonedds_cpp`). Both launch files set `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` for the processes they start, and `run_sim.sh` exports it as well. For the `ros2` commands typed in other terminals (`ros2 topic echo`, `rqt_image_view`, ...) add it once to `~/.bashrc`:

```bash
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc
source ~/.bashrc
ros2 daemon stop    # restart the CLI daemon with the new middleware
```

With the real ROSbot, also export the robot's `ROS_DOMAIN_ID` (10, 11 or 12) and make sure the robot side uses Cyclone DDS too.

## 1. One-time setup: Gazebo models

The simulation world needs the `aruco_box_1..4` models in `~/gazebo_models` from the repository inside [CarmineD8/erl1](https://github.com/CarmineD8/erl1). Copy them once, then make the folder visible to Gazebo:

```bash
echo 'export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/gazebo_models' >> ~/.bashrc
source ~/.bashrc
```

## 2. Build and run (simulation)

### Scripted build and run

The repository folder is also the colcon workspace. From that folder in WSL/Ubuntu, run:

```bash
./run_sim.sh
```

This single command:
1. closes any simulation processes left over from a previous run
2. sources ROS 2 Jazzy and makes the Gazebo models visible
3. builds all packages (`colcon build --symlink-install`)
4. launches Gazebo, RViz, the ArUco detector, the FSM and the photo service (`sim.launch.py`)

Options:

| Command | Effect |
|---------|--------|
| `./run_sim.sh --clean` | Deletes `build/`, `install/` and `log/` before building (use it if the build fails because of old files) |
| `./run_sim.sh rviz:=false` | Launches without RViz (any `<arg>:=<value>` is passed to `ros2 launch`) |
| `./run_sim.sh rosbot` | Builds and launches `rosbot.launch.py` (detector, FSM, photo service for the real ROSbot, no Gazebo/RViz). Without this flag the Gazebo simulation (`sim.launch.py`) is launched |
| `./run_sim.sh --clean rosbot` | Flags can be combined, in any order |

To stop everything, press **Ctrl+C** or close the Gazebo window. The script then shuts down every node.

### Manual build and run (without the script)

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch assignment sim.launch.py
```

## 3. Check that it works

Open a second terminal and source the workspace (`source install/setup.bash`), then try these:

```bash
ros2 topic echo /marker_array        # detected markers: id, center (u, v), side in px
ros2 run rqt_image_view rqt_image_view /marker_image   # annotated photo of each marker
```

Expected behaviour: the robot drives towards marker 1, centres it and approaches it, takes the photo, performs the marker action, and repeats this for markers 2 and 3. At marker 4 it stops and prints a completion message.

## Development history

See commits.

## Authors

This project was developed as a collaborative group project for the Experimental Robotics Laboratory course in Robotics Engineering Master's Degree at the University of Genoa.
<div align="center">

  <a href="https://github.com/Greg-0-0"><img src="https://github.com/Greg-0-0.png" width="60px"/></a>
  &nbsp;&nbsp;&nbsp;&nbsp;
   <a href="https://github.com/ErToto00"><img src="https://github.com/ErToto00.png" width="60px"/></a>
  &nbsp;&nbsp;&nbsp;&nbsp;
   <a href="https://github.com/Jamaxo03"><img src="https://github.com/Jamaxo03.png" width="60px"/></a>
  &nbsp;&nbsp;&nbsp;&nbsp;
   <a href="https://github.com/AriBardoni"><img src="https://github.com/AriBardoni.png" width="60px"/></a>
  &nbsp;&nbsp;&nbsp;&nbsp;
   <a href="https://github.com/Chiara-magn"><img src="https://github.com/Chiara-magn.png" width="60px"/></a>
</div>


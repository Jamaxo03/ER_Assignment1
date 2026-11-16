import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_erl1 = get_package_share_directory('erl1')
    pkg_assignment = get_package_share_directory('assignment')
    config = os.path.join(pkg_assignment, 'config', 'sim.yaml')

    # let Gazebo resolve the group's custom models (model://<name>): they are
    # installed in share/assignment/models, which is appended to
    # GZ_SIM_RESOURCE_PATH before erl1's launch file (and so Gazebo) is started.
    # The world and the instructor's aruco_box_1..4 models are not touched: the
    # world comes from erl1, the boxes from ~/gazebo_models
    models_path = os.path.join(pkg_assignment, 'models')
    gz_models = AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path)

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_erl1, 'launch', 'spawn_assignment.launch.py')))

    aruco_detector = Node(
        package='assignment',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    fsm_node = Node(
        package='assignment',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    # gz_models must come first: actions are executed in order
    return LaunchDescription([gz_models, simulation, aruco_detector, fsm_node])

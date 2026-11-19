import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_erl1 = get_package_share_directory('erl1')
    pkg_assignment = get_package_share_directory('assignment')
    config = os.path.join(pkg_assignment, 'config', 'sim.yaml')

    # DDS middleware: Cyclone DDS for every process started by this launch file
    # (it must be the first action, so that Gazebo, bridges and nodes inherit it)
    rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

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

    take_photo_srv = Node(
        package='assignment_interfaces',
        executable='take_photo_service',
        name='take_photo_service',
        output='screen',
        parameters=[config, {'use_sim_time': True}],
    )

    return LaunchDescription([
        rmw, simulation, aruco_detector, fsm_node, take_photo_srv
    ])

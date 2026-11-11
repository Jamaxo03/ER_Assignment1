import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_erl1 = get_package_share_directory('erl1')
    pkg_assignment = get_package_share_directory('assignment')
    config = os.path.join(pkg_assignment, 'config', 'sim.yaml')

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

    return LaunchDescription([simulation, aruco_detector])
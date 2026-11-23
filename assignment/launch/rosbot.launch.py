import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_assignment = get_package_share_directory('assignment')

    # Cyclone DDS for every node started by this launch file
    # ROS_DOMAIN_ID for reaching the robot 
    rmw = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value='rosbot.yaml',
        description='Parameter file in assignment/config')

    config = PathJoinSubstitution(
        [pkg_assignment, 'config', LaunchConfiguration('config')])

    aruco_detector = Node(
        package='assignment',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen',
        parameters=[config, {'use_sim_time': False}],
    )

    fsm_node = Node(
        package='assignment',
        executable='fsm_node',
        name='fsm_node',
        output='screen',
        parameters=[config, {'use_sim_time': False}],
    )

    take_photo_srv = Node(
        package='assignment_interfaces',
        executable='take_photo_service',
        name='take_photo_service',
        output='screen',
        parameters=[config, {'use_sim_time': False}],
    )

    return LaunchDescription([
        rmw, config_arg, aruco_detector, fsm_node, take_photo_srv
    ])
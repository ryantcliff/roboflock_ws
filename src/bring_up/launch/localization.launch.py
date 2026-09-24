"""Local odometry plus optional GPS localization, with exclusive TF ownership."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('bring_up'),
                          'config', 'ekf_navsat_params.yaml')
    clock = {'use_sim_time': ParameterValue(
        LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gps_localization', default_value='false',
                              choices=['true', 'false'],
                              description='GPS EKF owns map->odom; do not run with SLAM TF'),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_odom', output='screen', parameters=[config, clock],
             remappings=[('/odometry/filtered', '/odometry/local')]),
        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node_map', output='screen', parameters=[config, clock],
             condition=IfCondition(LaunchConfiguration('gps_localization')),
             remappings=[('/odometry/filtered', '/odometry/global')]),
        Node(package='robot_localization', executable='navsat_transform_node',
             name='navsat_transform_node', output='screen', parameters=[config, clock],
             condition=IfCondition(LaunchConfiguration('gps_localization')),
             remappings=[('/gps/fix', '/gps/robot/fix'),
                         ('/imu', '/imu/data'),
                         ('/odometry/filtered', '/odometry/global')]),
    ])

#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bring_up_share = get_package_share_directory('bring_up')
    foxglove_share = get_package_share_directory('foxglove_bridge')

    robot_state_publisher_launch = os.path.join(
        bring_up_share, 'launch', 'robot_state_publisher.launch.py'
    )
    rplidar_launch = os.path.join(
        bring_up_share, 'launch', 'rplidar_a1_launch.py'
    )
    slam_params = os.path.join(
        bring_up_share, 'config', 'slam_params.yaml'
    )
    foxglove_launch = os.path.join(
        foxglove_share, 'launch', 'foxglove_bridge_launch.xml'
    )

    foxglove_port = LaunchConfiguration('foxglove_port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'foxglove_port',
            default_value='8765',
            description='TCP port used by Foxglove Bridge',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_publisher_launch)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch)
        ),

        # Temporary LiDAR-only odometry for mapping without wheel odometry.
        # Do not run RF2O or another odom -> base_link publisher with this launch.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mapping_static_odom_to_base_link',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'odom',
                '--child-frame-id', 'base_link',
            ],
        ),

        # These overrides are the LiDAR-only settings verified during testing.
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[
                slam_params,
                {
                    'minimum_travel_distance': 0.0,
                    'minimum_travel_heading': 0.0,
                    'throttle_scans': 1,
                    'minimum_time_interval': 0.1,
                    'map_update_interval': 1.0,
                    'min_laser_range': 0.2,
                    'use_sim_time': False,
                },
            ],
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(foxglove_launch),
            launch_arguments={
                'port': foxglove_port,
                'address': '0.0.0.0',
                'use_sim_time': 'false',
            }.items(),
        ),
    ])


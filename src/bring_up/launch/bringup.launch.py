"""Bring up hardware, mapping (or a saved map), and navigation separately."""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(),
    )


def map_source(context):
    # Evaluate only the selected branch, so navigation-only debugging does not
    # require SLAM or a saved map installed/configured.
    if LaunchConfiguration('enable_mapping').perform(context).lower() != 'true':
        return []
    if LaunchConfiguration('slam').perform(context).lower() == 'true':
        return [include('bring_up', 'slam.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        })]
    map_file = LaunchConfiguration('map').perform(context)
    if not os.path.isfile(map_file):
        raise RuntimeError('slam:=false requires map:=/absolute/path/to/map.yaml')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return [
        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[{'yaml_filename': map_file, 'use_sim_time': clock}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_map', output='screen', parameters=[{
                 'use_sim_time': clock,
                 'autostart': ParameterValue(
                     LaunchConfiguration('autostart'), value_type=bool),
                 'node_names': ['map_server'],
             }]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('hardware', default_value='true', choices=['true', 'false'],
                              description='Start sensors, GPS and localization'),
        DeclareLaunchArgument('enable_mapping', default_value='true', choices=['true', 'false'],
                              description='Start SLAM or the saved-map server'),
        DeclareLaunchArgument('slam', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('map', default_value='', description='Saved map YAML when slam=false'),
        DeclareLaunchArgument('params_file', default_value=PathJoinSubstitution([
            FindPackageShare('bring_up'), 'config', 'nav2_params.yaml'])),
        GroupAction(condition=IfCondition(LaunchConfiguration('hardware')), actions=[
            include('bring_up', 'robot_state_publisher.launch.py'),
            include('beacon_pkg', 'beacon_receiver.launch.py'),
            include('bring_up', 'robot_gps.launch.py'),
            include('bring_up', 'mpu9250.launch.py'),
            include('rplidar_ros', 'rplidar_a1_launch.py'),
            include('ultrasonic_pkg', 'ultrasonic_publisher.launch.py'),
            include('bring_up', 'localization.launch.py', {
                'gps_localization': PythonExpression([
                    "'false' if '", LaunchConfiguration('slam'), "' == 'true' else 'true'"]),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }),
            include('bring_up', 'rf2o_laser_odometry.launch.py'),
            # Beacon GPS goals need a shared map datum (priority 3). Keep
            # automatic goal publication off until that conversion is fixed.
        ]),
        OpaqueFunction(function=map_source),
        include('bring_up', 'nav2.launch.py', {
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
        }),
        # Motor startup remains deliberately absent until stop/watchdog handling
        # is implemented. No launch argument enables the current motor driver.
    ])

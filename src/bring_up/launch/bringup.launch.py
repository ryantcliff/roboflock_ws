"""Bring up hardware, mapping (or a saved map), navigation and, optionally, beacon following."""

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
from nav2_common.launch import RewrittenYaml


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(),
    )


def enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() == 'true'


def map_source(context):
    # Evaluate only the selected branch, so navigation-only debugging does not
    # require SLAM or a saved map installed/configured.
    if not enabled(context, 'enable_mapping'):
        return []
    if LaunchConfiguration('slam').perform(context).lower() == 'true':
        return [include('bring_up', 'slam.launch.py', {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        })]
    map_file = LaunchConfiguration('map').perform(context)
    if enabled(context, 'follow') and not map_file:
        return []  # Outdoor following plans on the live lidar costmap only.
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


def navigation(context):
    params_file = LaunchConfiguration('params_file')
    if enabled(context, 'follow'):
        if enabled(context, 'slam'):
            raise RuntimeError(
                'follow:=true requires slam:=false: beacon goals need the GPS-aligned map frame')
        params_file = RewrittenYaml(
            source_file=params_file, convert_types=True, param_rewrites={
                # No prior map outdoors: plan on the rolling lidar costmap.
                'global_costmap.global_costmap.ros__parameters.static_layer.enabled': 'False',
                # The truncated follow path ends in an arbitrary heading; do not
                # turn in place to match it.
                'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance': '3.14',
            })
    return [include('bring_up', 'nav2.launch.py', {
        'autostart': LaunchConfiguration('autostart'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'params_file': params_file,
    })]


def sim_nodes(context):
    if not enabled(context, 'sim'):
        return []
    if enabled(context, 'hardware'):
        raise RuntimeError('sim:=true replaces the drivers; also set hardware:=false')
    # Same localization and robot model as hardware; fake sensors and motors.
    nodes = [
        include('bring_up', 'robot_state_publisher.launch.py'),
        include('bring_up', 'localization.launch.py', {
            'gps_localization': 'false' if enabled(context, 'slam') else 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),
        Node(package='bring_up', executable='fake_robot', name='fake_robot', output='screen',
             parameters=[{'cmd_vel_topic':
                          'cmd_vel_mux' if enabled(context, 'follow') else 'cmd_vel'}]),
    ]
    if enabled(context, 'follow') and enabled(context, 'fake_beacon'):
        nodes.append(Node(package='bring_up', executable='fake_beacon', name='fake_beacon',
                          output='screen'))
    return nodes


def follow_nodes(context):
    if not enabled(context, 'follow'):
        return []
    twist_mux_params = PathJoinSubstitution([
        FindPackageShare('bring_up'), 'config', 'twist_mux.yaml'])
    nodes = [
        Node(package='beacon_pkg', executable='beacon_goalpose', name='beacon_goalpose',
             output='screen'),
        Node(package='bring_up', executable='follow_manager', name='follow_manager',
             output='screen', parameters=[{'enabled': ParameterValue(
                 LaunchConfiguration('follow_enabled'), value_type=bool)}]),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[twist_mux_params], remappings=[('cmd_vel_out', 'cmd_vel_mux')]),
    ]
    if enabled(context, 'hardware'):
        # Motors only start in follow mode, and stay idle until e_stop is armed.
        nodes += [
            Node(package='joy', executable='joy_node', name='joy_node', output='screen'),
            Node(package='bring_up', executable='e_stop', name='e_stop', output='screen'),
            Node(package='bring_up', executable='ps4_teleop', name='ps4_teleop',
                 output='screen', parameters=[{'cmd_vel_topic': 'cmd_vel_joy'}]),
            Node(package='bring_up', executable='ultrasonic_estop', name='ultrasonic_estop',
                 output='screen'),
            Node(package='bring_up', executable='diff_drive_controller',
                 name='diff_drive_controller', output='screen',
                 parameters=[{'cmd_vel_topic': 'cmd_vel_mux'}]),
        ]
    return nodes


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
        DeclareLaunchArgument('follow', default_value='false', choices=['true', 'false'],
                              description='Follow the GPS beacon (requires slam:=false)'),
        DeclareLaunchArgument('sim', default_value='false', choices=['true', 'false'],
                              description='Fake robot and sensors (with hardware:=false)'),
        DeclareLaunchArgument('fake_beacon', default_value='true', choices=['true', 'false'],
                              description='With sim and follow: walk a fake beacon'),
        DeclareLaunchArgument('follow_enabled', default_value='true', choices=['true', 'false'],
                              description='Start following at launch; toggle with /follow/enable'),
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
        ]),
        OpaqueFunction(function=map_source),
        OpaqueFunction(function=navigation),
        OpaqueFunction(function=sim_nodes),
        OpaqueFunction(function=follow_nodes),
    ])

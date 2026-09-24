"""Launch navigation servers; mapping and localization are owned by bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('bring_up'), 'config', 'nav2_params.yaml']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'autostart': LaunchConfiguration('autostart'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Humble evaluates this value inside PythonExpression('not ...').
                'use_composition': 'False',
            }.items(),
        ),
    ])

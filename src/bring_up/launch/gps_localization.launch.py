"""GPS localization only: sensors, both EKFs and navsat_transform. No Nav2, no motors."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def include(package, filename, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare(package), 'launch', filename])),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('imu', default_value='mpu9250', choices=['mpu9250', 'bno085'],
                              description='IMU driver (mpu9250 has no real heading)'),
        include('bring_up', 'robot_state_publisher.launch.py'),
        include('bring_up', 'robot_gps.launch.py'),
        include('bring_up', 'imu.launch.py', {'imu': LaunchConfiguration('imu')}),
        include('rplidar_ros', 'rplidar_a1_launch.py'),
        include('bring_up', 'rf2o_laser_odometry.launch.py'),
        include('bring_up', 'localization.launch.py', {
            'gps_localization': 'true',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }),
        Node(package='bring_up', executable='gps_monitor', name='gps_monitor',
             output='screen', parameters=[{'topics': ['/gps/robot/fix']}]),
    ])

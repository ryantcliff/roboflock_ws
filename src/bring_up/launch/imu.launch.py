"""IMU driver selected by the imu argument: mpu9250 (current board) or bno085."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # The board on the robot is an MPU-6050 (no magnetometer), so mpu9250
        # gives no real heading. Switch to bno085 once one is fitted.
        DeclareLaunchArgument('imu', default_value='mpu9250', choices=['mpu9250', 'bno085'],
                              description='IMU driver'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('bring_up'), 'launch', 'mpu9250.launch.py'])),
            condition=LaunchConfigurationEquals('imu', 'mpu9250')),
        Node(package='bring_up', executable='bno085_imu', name='bno085_imu', output='screen',
             parameters=[PathJoinSubstitution([
                 FindPackageShare('bring_up'), 'config', 'bno085.yaml'])],
             condition=LaunchConfigurationEquals('imu', 'bno085')),
    ])

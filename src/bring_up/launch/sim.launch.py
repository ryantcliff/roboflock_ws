import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    # -----------------------------
    # Robot description
    # -----------------------------

    urdf_file = os.path.join(
        get_package_share_directory('urdf_description'),
        'urdf',
        'URDF.xacro'
    )

    robot_description = xacro.process_file(urdf_file).toxml()

    # -----------------------------
    # Robot State Publisher
    # -----------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    # -----------------------------
    # Gazebo Sim
    # -----------------------------

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    # -----------------------------
    # Spawn RoboFlock
    # -----------------------------

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'roboflock',
            '-topic', 'robot_description',
            '-z', '0.6'
        ],
        output='screen'
    )

    # -----------------------------
    # ROS <-> Gazebo bridge
    # -----------------------------

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,

        TimerAction(
            period=2.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=4.0,
            actions=[bridge]
        ),
    ])

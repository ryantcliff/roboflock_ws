import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from ament_index_python.packages import get_package_share_directory

import xacro


def generate_launch_description():

    bring_up_share = get_package_share_directory('bring_up')
    world_file = os.path.join(bring_up_share, 'worlds', 'basic_world.sdf')
    rviz_config = os.path.join(
        bring_up_share,
        'config',
        'roboflock_sim.rviz'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Gazebo world file'
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz'
    )
    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Start SLAM Toolbox'
    )
    declare_nav2 = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Start Nav2'
    )
    declare_follow_beacon = DeclareLaunchArgument(
        'follow_beacon',
        default_value='true',
        description='Publish simulated GPS and follow the beacon'
    )

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
            'gz_args': ['-r ', LaunchConfiguration('world')]
        }.items()
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                bring_up_share,
                'launch',
                'slam.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('slam'))
    )

    navigation = GroupAction(
        actions=[
            SetRemap(src='/cmd_vel', dst='/cmd_vel/nav'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bring_up_share, 'launch', 'nav2.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'true',
                    'autostart': 'true',
                }.items(),
            ),
        ],
        condition=IfCondition(LaunchConfiguration('nav2')),
    )

    tracking = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('beacon_pkg'),
                'launch',
                'beacon_goalpose.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items(),
        condition=IfCondition(LaunchConfiguration('follow_beacon')),
    )

    simulated_gps = Node(
        package='bring_up',
        executable='simulated_gps',
        name='simulated_gps',
        output='screen',
        parameters=[
            os.path.join(bring_up_share, 'config', 'simulated_gps.yaml'),
            {'use_sim_time': True},
        ],
        condition=IfCondition(LaunchConfiguration('follow_beacon')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
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
        remappings=[('/cmd_vel', '/cmd_vel/safe')],
        output='screen'
    )
    velocity_safety = Node(
        package='bring_up',
        executable='velocity_safety',
        name='velocity_safety',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'require_ultrasonic': False,
        }],
    )
    odom_tf_broadcaster = Node(
        package='bring_up',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    lidar_frame_bridge = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_frame_bridge',
    arguments=[
        '--x', '0',
        '--y', '0',
        '--z', '0',
        '--roll', '0',
        '--pitch', '0',
        '--yaw', '0',
        '--frame-id', 'lidar_link',
        '--child-frame-id', 'roboflock/base_link/lidar',
    ],
    output='screen'
)
    return LaunchDescription([
        declare_world,
        declare_rviz,
        declare_slam,
        declare_nav2,
        declare_follow_beacon,
        gazebo,
        robot_state_publisher,
        odom_tf_broadcaster,
        lidar_frame_bridge,
        velocity_safety,
        simulated_gps,
        rviz,

        TimerAction(
            period=2.0,
            actions=[spawn_robot]
        ),

        TimerAction(
            period=4.0,
            actions=[bridge]
        ),

        TimerAction(
            period=6.0,
            actions=[slam]
        ),

        TimerAction(
            period=8.0,
            actions=[navigation]
        ),

        TimerAction(
            period=10.0,
            actions=[tracking]
        ),
    ])

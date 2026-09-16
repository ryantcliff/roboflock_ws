import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
<<<<<<< HEAD
from launch.substitutions import LaunchConfiguration

=======
from ament_index_python.packages import get_package_share_directory
>>>>>>> upstream/dev

def generate_launch_description():

    bringup_dir = get_package_share_directory('bring_up')
<<<<<<< HEAD
    nav2_params_file = os.path.join(
        bringup_dir, 'config', 'nav2_params.yaml'
    )
    navigation = IncludeLaunchDescription(
=======
    nav2_params_file = os.path.join(bringup_dir, 'config', 'nav2_params.yaml')

    nav2_launch = IncludeLaunchDescription(
>>>>>>> upstream/dev
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
<<<<<<< HEAD
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
=======
            'params_file': nav2_params_file,
            'autostart': LaunchConfiguration('autostart'),
            'use_sim_time': 'false'
>>>>>>> upstream/dev
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
<<<<<<< HEAD
            'params_file',
            default_value=nav2_params_file,
            description='Nav2 parameter file'
        ),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        navigation,
=======
            'autostart',
            default_value='true',
            description='Automatically start Nav2'
        ),
        nav2_launch
>>>>>>> upstream/dev
    ])

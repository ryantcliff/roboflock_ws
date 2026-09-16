import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    bring_up_share = get_package_share_directory('bring_up')
    mode = LaunchConfiguration('mode')
    is_simulation = PythonExpression(["'", mode, "' == 'simulation'"])
    is_physical = PythonExpression(["'", mode, "' != 'simulation'"])
    is_autonomous = PythonExpression(["'", mode, "' == 'autonomous'"])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bring_up_share, 'launch', 'sim.launch.py')
        ),
        condition=IfCondition(is_simulation),
    )
    physical = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bring_up_share, 'launch', 'bringup.launch.py')
        ),
        launch_arguments={
            'slam': 'false',
            'nav2': is_autonomous,
            'tracking': is_autonomous,
        }.items(),
        condition=IfCondition(is_physical),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='simulation',
            choices=['simulation', 'teleop', 'autonomous'],
            description='RoboFlock operating mode',
        ),
        simulation,
        physical,
    ])

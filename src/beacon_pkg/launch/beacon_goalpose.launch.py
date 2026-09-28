from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='beacon_pkg',
            executable='beacon_goalpose',
            name='beacon_goalpose',
            output='screen'
        )
    ])

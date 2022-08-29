from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'number', default_value='0.4'
        ),
        DeclareLaunchArgument(
            'boolInput', default_value='true'
        ),
        Node(
            package='package_Name',
            name='exe_name',
            executable='exe_name',
            parameters=[
                {'number': LaunchConfiguration('number')},
                {'boolInput': LaunchConfiguration('boolInput')}
            ]
        ),
    ])

    
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='safety',
            namespace='safety',
            executable='e_brake'
        ),
        DeclareLaunchArgument(
            'offset', default_value='0.4'
        ),
        DeclareLaunchArgument(
            'verbose', default_value='true'
        ),
        Node(
            package='utilities',
            namespace='utilities',
            executable='subscriber_gen',
            parameters=[
                {'offset': LaunchConfiguration('offset')},
                {'verbose': LaunchConfiguration('verbose')}
            ]
        )
    ])

    
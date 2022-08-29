from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='safety',
            namespace='safety',
            executable='e_brake'
        ),
        Node(
            package='utilities',
            namespace='utilities',
            executable='subscriber_gen'
        )
    ])

    
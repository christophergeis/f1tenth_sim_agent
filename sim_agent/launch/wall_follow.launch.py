from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution

from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument(
            'ttcThresh', default_value='0.35'
        ),
        Node(
            package='sim_agent',
            executable='e_brake',
            name='e_brake',
            parameters=[
                {'ttcThresh': LaunchConfiguration('ttcThresh')}
            ]
         ),
        # Node(
        #     package='sim_agent',
        #     executable='wall_follow_scan_tf2',
        #     name='wall_follow_scan_tf2'
        # ),
        DeclareLaunchArgument(
            'kp', default_value='0.4'
        ),
        DeclareLaunchArgument(
            'ki', default_value='0.0'
        ),
        DeclareLaunchArgument(
            'kd', default_value='0.0004'
        ),
        DeclareLaunchArgument(
            'L', default_value='0.8'
        ),
        DeclareLaunchArgument(
            'wallDist', default_value='0.6'
        ),
        DeclareLaunchArgument(
            'angleA', default_value='55.0'
        ),
        DeclareLaunchArgument(
            'angleB', default_value='100.0'
        ),
        Node(
            package='sim_agent',
            executable='wall_follow',
            name='wall_follow',
            parameters=[
                {'kp': LaunchConfiguration('kp')},
                {'ki': LaunchConfiguration('ki')},
                {'kd': LaunchConfiguration('kd')},
                {'L': LaunchConfiguration('L')},
                {'wallDist': LaunchConfiguration('wallDist')},
                {'angleA': LaunchConfiguration('angleA')},
                {'angleB': LaunchConfiguration('angleB')}
            ]
        ),
    ])
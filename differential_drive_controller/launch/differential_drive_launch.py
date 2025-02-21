import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='differential_drive_controller',
            executable='differential_drive_controller',
            name='differential_drive_controller',
            output='screen',
            parameters=[{
                'wheelbase': 0.5,
                'wheel_radius': 0.1,
                'max_rpm': 300.0
            }]
        )
    ])

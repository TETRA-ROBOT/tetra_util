#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tetra_service_node = Node(
        package='tetra_service',
        executable='tetra_service',
        output='screen',
        parameters=[{
            'robot_model': 's',
            'm_dHome_ID': 0
        }]
    )

    return LaunchDescription([
        tetra_service_node
    ])

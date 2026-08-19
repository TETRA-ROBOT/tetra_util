from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='velocity_integrator',
            executable='velocity_integrator_node',
            name='velocity_integrator_node',
            output='screen',
            parameters=[],
            remappings=[
                # ('/cmd_vel', '/my_robot/cmd_vel')
            ]
        )
    ])


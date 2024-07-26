from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='server_trajectory',
            executable='server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='bp001_control',
            executable='balance_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

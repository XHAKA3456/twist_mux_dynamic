from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux_dynamic',
            executable='twist_mux_dynamic',
            name='twist_mux_dynamic',
            output='screen',
            parameters=['config/twist_mux_params.yaml']
        )
    ])

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_gpu_demo',
            executable='gpu_matrix_multiply_node',
            name='gpu_matrix_multiply_node',
            output='screen',
        ),
    ])


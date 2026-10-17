from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='torq_r1_navigation',
            executable='arrow_teleop_node',
            name='arrow_teleop_node',
            output='screen',
            emulate_tty=True
        )
    ])

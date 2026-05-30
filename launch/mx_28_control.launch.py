from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('mx_28_control'), 'config', 'mx_28_control.yaml']
    )

    return LaunchDescription([
        Node(
            package='mx_28_control',
            executable='mx_28_control',
            name='mx_28_control',
            output='screen',
            parameters=[config_file],
        )
    ])

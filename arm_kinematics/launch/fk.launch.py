"""Arranca fk_node con los parámetros de config/params.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('arm_kinematics'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='arm_kinematics',
            executable='fk_node',
            name='fk_node',
            parameters=[params],
            output='screen',
        ),
    ])

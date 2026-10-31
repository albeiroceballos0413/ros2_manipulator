"""Muestra el brazo RRR en RViz con deslizadores para mover las articulaciones."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('arm_description')
    with open(os.path.join(share, 'urdf', 'rrr_arm.urdf')) as f:
        robot_description = f.read()
    rviz_cfg = os.path.join(share, 'rviz', 'display.rviz')
    fk_params = os.path.join(
        get_package_share_directory('arm_kinematics'), 'config', 'params.yaml')

    return LaunchDescription([
        # URDF + /joint_states -> transformaciones (TF) de cada eslabón
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': robot_description}]),
        # Ventana con deslizadores que publica /joint_states
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        # Tu nodo: calcula la pose de la punta con DH y la publica
        Node(package='arm_kinematics', executable='fk_node', name='fk_node',
             parameters=[fk_params], output='screen'),
        # Visualización
        Node(package='rviz2', executable='rviz2', arguments=['-d', rviz_cfg]),
    ])

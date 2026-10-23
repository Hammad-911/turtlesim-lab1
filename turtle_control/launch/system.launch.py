import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('turtle_control'), 'config', 'params.yaml')

    return LaunchDescription([
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='turtle_control', executable='go_to_pose_server',
             name='go_to_pose_server', parameters=[params], output='screen'),
        Node(package='turtle_control', executable='reset_service',
             name='reset_service', output='screen'),
        Node(package='turtle_control', executable='square_server',
             name='square_server', output='screen'),
    ])

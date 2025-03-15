import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():


    return LaunchDescription([
        Node(
            package='hnurm_radar',
            executable='lidar_node',
            output='screen',
        ),
        Node(
            package='hnurm_radar',
            executable='detector_node',
            output='screen',
        ),
        Node(
            package='hnurm_radar',
            executable='radar_node',
            output='screen',
            parameters=[{
                'param_name': 'param_value'
            }]
        ),
        Node(
            package='hnurm_radar',
            executable='radar_node',
            output='screen',
        ),
    ])
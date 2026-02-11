import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ROS 2 Launch File: 用于启动项目所需的多个节点
def generate_launch_description():
    camera_dir = get_package_share_directory('hnurm_camera')
    params_file = LaunchConfiguration('params_file')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='Full path to the ROS2 parameters file to use for the camera'
        ),

        # 启动雷达相关的节点
        Node(
            package='hnurm_radar',
            executable='lidar_node',
            output='screen',
        ),
        # 启动一个节点，用于播放视频文件作为模拟摄像头输入
        Node(
            package='hnurm_radar',
            executable='publish_video',
            output='screen',
            parameters=[{
                'video_file': '/data/projects/radar/video.avi'
            }]
        ),
        # 启动核心的检测节点
        Node(
            package='hnurm_radar',
            executable='detector_node',
            output='screen',
        ),
        # 启动雷达节点
        Node(
            package='hnurm_radar',
            executable='radar_node',
            output='screen',
            parameters=[{
                'param_name': 'param_value'
            }]
        ),
    ])

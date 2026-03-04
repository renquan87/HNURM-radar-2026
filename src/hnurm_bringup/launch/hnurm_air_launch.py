"""
hnurm_air_launch.py — 空中机器人识别启动文件（方案三 air_scheme）

功能：
  启动空中机器人检测相关节点（纯激光雷达点云聚类定位）。
  不包含地面机器人识别，如需地面方案请使用 hnurm_radar_launch.py。

启动的节点：
  1. lidar_node      — 激光雷达数据接收与点云预处理（空中方案也需要点云）
  2. air_target_node — 空中机器人检测（点云聚类 + 卡尔曼跟踪）
  3. display_panel   — 小地图可视化面板
  4. ekf_node        — 扩展卡尔曼滤波

使用方式：
  ros2 launch hnurm_bringup hnurm_air_launch.py

另见：
  hnurm_radar_launch.py       — 地面机器人识别（激光雷达 + 相机）
  hnurm_radar_video_launch.py — 视频离线调试版本
"""

from launch import LaunchDescription
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
            executable='air_target_node',
            output='screen',
        ),
        Node(
            package='hnurm_radar',
            executable='display_panel',
            output='screen',
        ),
        Node(
            package='ekf',
            executable='ekf_node',
            output='screen',
        ),
    ])

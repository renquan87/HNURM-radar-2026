"""
hnurm_radar_launch.py — 雷达站主启动文件（方案二 lidar_scheme）

功能：
  使用 ROS2 Launch 一键启动雷达站全部节点，适用于接入真实硬件
  （海康相机 + Livox 激光雷达）的比赛/调试场景。

启动的节点（按启动顺序）：
  1. lidar_node      — 激光雷达数据接收与点云预处理
  2. detector_node   — YOLO 三阶段目标检测（装甲板检测 → ROI 裁剪 → 数字识别）
  3. radar_node      — 点云-图像融合定位主节点（坐标解算核心）
  4. display_panel   — 小地图可视化面板
  5. judge_messager  — 裁判系统串口通信
  6. ekf_node        — 扩展卡尔曼滤波（位置平滑）

使用方式：
  ros2 launch hnurm_bringup hnurm_radar_launch.py

另见：
  hnurm_radar_video_launch.py — 使用视频文件替代相机的离线调试版本
"""

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
            executable='display_panel',
            output='screen',
        ),
        Node(
            package='hnurm_radar',
            executable='judge_messager',
            output='screen',
        ),
        Node(
            package='ekf',
            executable='ekf_node',
            output='screen',
        )
    ])

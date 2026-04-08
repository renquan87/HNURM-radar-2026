"""
hnurm_rosbag_launch.py — Rosbag 回放模式启动文件（方案二 lidar_scheme 离线测试）

功能：
  使用录制的 rosbag 数据回放，替代实际硬件设备进行离线测试。
  自动从 main_config.yaml 读取 rosbag_path，并启动 ros2 bag play
  + 所有方案二节点 + rviz + foxglove_bridge。

  与 hnurm_radar_launch.py 的区别：
    - detector_node 从 /compressed_image topic 订阅图像，而非海康相机硬件
    - lidar_node 从 rosbag 发布的 /livox/lidar 获取点云数据
    - 自动启动 ros2 bag play 播放 rosbag
    - 包含 foxglove_bridge 用于 Foxglove Studio 可视化

  与实时版（bringup.sh）的对应关系：
    实时版                              rosbag 版
    ──────────────────────────────      ──────────────────────────────
    livox_ros_driver2 (硬件)            ros2 bag play (回放)
    hnurm_radar_launch.py               本文件（主节点组 + 可视化）
    registration.launch.py              单独终端启动 registration.launch.py

前置条件：
  1. configs/main_config.yaml 中 camera.mode 设为 "rosbag"
  2. configs/main_config.yaml 中 camera.rosbag_path 填写 rosbag 目录路径
  3. rosbag 目录中包含 metadata.yaml（如无，可用 scripts/generate_rosbag_metadata.py 生成）

启动的节点/进程：
  1. ros2 bag play       — 播放 rosbag 数据（ExecuteProcess）
  2. lidar_node          — 激光雷达数据接收与点云预处理
  3. detector_node       — YOLO 三阶段目标检测（rosbag 模式：订阅压缩图像）
  4. radar_node          — 点云-图像融合定位
  5. display_panel       — 小地图可视化面板
  6. ekf_node            — 扩展卡尔曼滤波
  7. rviz2               — 点云可视化
  8. foxglove_bridge     — Foxglove Studio WebSocket 桥接

使用方式：
  # 1. 先修改 configs/main_config.yaml:
  #    camera:
  #      mode: "rosbag"
  #      rosbag_path: "/path/to/your/rosbag/directory"
  #
  # 2. 构建
  #    colcon build --packages-select hnurm_bringup hnurm_radar registration
  #
  # 3. 启动（推荐使用 bringup_rosbag.sh 一键启动）
  #    ./bringup_rosbag.sh
  #    或手动分两个终端：
  #    终端1: ros2 launch hnurm_bringup hnurm_rosbag_launch.py
  #    终端2: ros2 launch registration registration.launch.py
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def _get_project_root():
    """通过 ament 包安装路径向上推导 colcon workspace 根目录

    colcon install 布局:
      <ws>/install/<pkg>/share/<pkg>/launch/this_file.py
    get_package_share_directory 返回:
      <ws>/install/<pkg>/share/<pkg>
    向上 4 级 → <ws>
    """
    share_dir = get_package_share_directory('hnurm_bringup')
    return os.path.normpath(
        os.path.join(share_dir, '..', '..', '..', '..')
    )


def _load_rosbag_config():
    """从 main_config.yaml 读取 rosbag 相关配置

    返回 dict: {rosbag_path}
    """
    result = {
        'rosbag_path': None,
    }
    try:
        from ruamel.yaml import YAML
        project_root = _get_project_root()
        config_path = os.path.join(project_root, 'configs', 'main_config.yaml')

        with open(config_path, encoding='utf-8') as f:
            cfg = YAML().load(f)

        camera_cfg = cfg.get('camera', {})
        result['rosbag_path'] = camera_cfg.get('rosbag_path', '')

        if not result['rosbag_path']:
            print('[WARN] main_config.yaml 中未配置 camera.rosbag_path，'
                  '请设置后重新启动', file=sys.stderr)
            result['rosbag_path'] = None
    except Exception as e:
        print(f'[ERROR] 读取 rosbag 配置失败: {e}', file=sys.stderr)

    return result


def generate_launch_description():
    rosbag_cfg = _load_rosbag_config()
    rosbag_path = rosbag_cfg['rosbag_path']

    project_root = _get_project_root()
    print(f'[rosbag_launch] project_root = {project_root}', file=sys.stderr)

    rviz_config = os.path.join(project_root, 'configs', 'rosbag_pointcloud.rviz')

    actions = []

    # ── rosbag 播放 ──
    if rosbag_path:
        bag_play = ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'play',
                rosbag_path,
                '--topics', '/livox/lidar', '/compressed_image',
                '--rate', '1.0',
                '--delay', '5.0',  # 等待所有节点（含 registration）启动
            ],
            output='screen',
            name='rosbag_play',
        )
        actions.append(
            LogInfo(msg=f'[rosbag] 将播放: {rosbag_path}')
        )
        actions.append(bag_play)
    else:
        actions.append(
            LogInfo(msg='[rosbag] ⚠ 未配置 rosbag_path，请手动执行 ros2 bag play')
        )

    # ── RViz2（点云可视化） ──
    if os.path.isfile(rviz_config):
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config]
            )
        )
    else:
        print(f'[WARN] RViz 配置文件不存在: {rviz_config}，跳过 rviz2 启动',
              file=sys.stderr)

    # ── Foxglove Bridge（Foxglove Studio 可视化） ──
    actions.append(
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='screen',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0',
                'send_buffer_limit': 10000000,
            }]
        )
    )

    # ── 方案二核心节点 ──
    actions.extend([
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
            package='ekf',
            executable='ekf_node',
            output='screen',
        ),
    ])

    return LaunchDescription(actions)

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get_project_root():
    """通过 ament 包安装路径向上推导 colcon workspace 根目录"""
    share_dir = get_package_share_directory('registration')
    return os.path.normpath(
        os.path.join(share_dir, '..', '..', '..', '..')
    )


def _load_scene_pcd_paths(project_root):
    """从 main_config.yaml 读取当前场景的 PCD 路径。

    返回 (pcd_file, downsampled_pcd_file) 的绝对路径元组。
    读取失败时回退到默认的 data/pcds*.pcd。
    """
    pcd_rel = "data/pcds.pcd"
    down_rel = "data/pcds_downsampled.pcd"
    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        config_path = os.path.join(project_root, "configs", "main_config.yaml")
        with open(config_path, encoding="utf-8") as f:
            cfg = yaml.load(f)
        scene_name = cfg.get("global", {}).get("scene", "competition")
        scenes = cfg.get("scenes", {})
        scene_cfg = scenes.get(scene_name, {})
        if scene_cfg:
            pcd_rel = scene_cfg.get("pcd_file", pcd_rel)
            down_rel = scene_cfg.get("downsampled_pcd", down_rel)
        print(f"[registration.launch] scene={scene_name}, "
              f"pcd={pcd_rel}, downsampled={down_rel}")
    except Exception as e:
        print(f"[registration.launch] Failed to load scene config: {e}, using defaults")

    return (
        os.path.join(project_root, pcd_rel),
        os.path.join(project_root, down_rel),
    )


def generate_launch_description():
    project_root = _get_project_root()
    camera_dir = get_package_share_directory('registration')
    params_file = LaunchConfiguration('params_file')
    rviz_config = os.path.join(project_root, 'configs', 'icp.rviz')

    # 从场景配置动态获取 PCD 路径
    pcd_file, downsampled_pcd_file = _load_scene_pcd_paths(project_root)

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='small gicp params'
        ),
        Node(
            package='registration',
            executable='registration_node',
            output='screen',
            parameters=[
                params_file,
                {'pcd_file': pcd_file},
                {'downsampled_pcd_file': downsampled_pcd_file},
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )

    ])

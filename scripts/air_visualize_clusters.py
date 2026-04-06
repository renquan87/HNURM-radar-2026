#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
air_visualize_clusters.py — 空中目标检测可视化工具

功能：
  - 读取 PCD 文件
  - 执行完整的预处理 + 背景减除 + 聚类流程
  - 用 Open3D 可视化：原始点云（灰色）+ 各聚类（随机色）+ 包围盒
  - 在终端打印每个目标的置信度和尺寸信息
  - 支持模拟多帧跟踪序列（--frames 参数）

用法：
  python scripts/air_visualize_clusters.py --pcd data/pointclouds/registration/lab_pcds.pcd
  python scripts/air_visualize_clusters.py --pcd data/pointclouds/registration/lab_pcds.pcd --show-raw
  python scripts/air_visualize_clusters.py --pcd data/frame0.pcd --frames frame0.pcd frame1.pcd ...
"""

import argparse
import os
import sys
import numpy as np
import open3d as o3d
from ruamel.yaml import YAML

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src", "hnurm_radar")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from hnurm_radar.air_scheme.air_config import load_air_target_config
from hnurm_radar.air_scheme.point_cloud_processor import AirPointCloudProcessor
from hnurm_radar.air_scheme.cluster_detector import AirClusterDetector, AirTarget
from hnurm_radar.air_scheme.background_subtractor import AirBackgroundSubtractor
from hnurm_radar.air_scheme.target_tracker import AirTargetTracker, TrackedAirTarget
from hnurm_radar.air_scheme.air_kalman_filter import AirKalmanFilter


# ---- 颜色方案 ----
_COLOR_RAW        = np.array([0.6, 0.6, 0.6])   # 原始点云：灰色
_COLOR_BG         = np.array([0.3, 0.3, 0.3])   # 背景点：深灰
_COLOR_PALETTE = [                              # 聚类随机色（固定 seed 保证一致）
    [1.0, 0.2, 0.2],  # 红
    [0.2, 0.6, 1.0],  # 蓝
    [0.2, 0.9, 0.2],  # 绿
    [1.0, 0.8, 0.0],  # 黄
    [0.8, 0.2, 1.0],  # 紫
    [0.0, 0.9, 0.9],  # 青
    [1.0, 0.5, 0.0],  # 橙
    [0.5, 1.0, 0.5],  # 浅绿
]


def parse_args():
    p = argparse.ArgumentParser(description="空中目标检测可视化")
    p.add_argument("--pcd",     required=True,      help="输入 PCD 文件路径")
    p.add_argument("--frames",  nargs="+",           help="多帧 PCD 文件（模拟跟踪序列）")
    p.add_argument("--config",  default=os.path.join(PROJECT_ROOT, "configs", "main_config.yaml"),
                   help="配置文件路径")
    p.add_argument("--show-raw", action="store_true", help="同时显示原始点云")
    p.add_argument("--no-track", action="store_true", help="禁用跟踪，只显示聚类结果")
    p.add_argument("--bg-pcd",   default=None,         help="预建背景PCD路径（跳过在线学习）")
    return p.parse_args()


def make_bbox(min_pt: np.ndarray, max_pt: np.ndarray,
              color=(1.0, 0.0, 0.0)) -> o3d.geometry.AxisAlignedBoundingBox:
    """创建彩色 AABB"""
    bbox = o3d.geometry.AxisAlignedBoundingBox(
        min_bound=min_pt.tolist(), max_bound=max_pt.tolist()
    )
    bbox.color = color
    return bbox


def make_sphere(center: np.ndarray, radius: float = 0.15,
                color=(1.0, 1.0, 0.0)) -> o3d.geometry.TriangleMesh:
    """在目标中心创建一个小球体标注（表示卡尔曼估计位置）"""
    sphere = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
    sphere.translate(center)
    sphere.paint_uniform_color(color)
    sphere.compute_vertex_normals()
    return sphere


def process_single_frame(
    pcd_path: str,
    processor: AirPointCloudProcessor,
    detector:  AirClusterDetector,
    bg_subtractor: AirBackgroundSubtractor,
    tracker:   AirTargetTracker,
    show_raw:  bool,
    use_track: bool,
    frame_idx: int,
) -> list:
    """处理单帧，返回 Open3D 几何体列表"""
    pcd_raw = o3d.io.read_point_cloud(pcd_path)
    if len(pcd_raw.points) == 0:
        print(f"  [警告] {pcd_path} 为空点云，跳过")
        return []

    # 预处理
    pcd_roi      = processor.crop_roi(pcd_raw)
    pcd_down     = processor.downsample(pcd_roi)

    # 背景减除
    bg_subtractor.learn(pcd_down)
    pcd_fg       = bg_subtractor.filter(pcd_down)

    # 高度过滤
    tracked_hs   = [t.height for t in tracker.get_all_targets()] if use_track else None
    pcd_filtered = processor.filter_height(pcd_fg, tracked_hs)

    # 聚类检测
    targets      = detector.detect(pcd_filtered)

    # 跟踪
    tracked: list = []
    if use_track:
        lq_params = {
            "expand_xy": 1.0, "expand_z": 0.5,
            "eps_scale": 1.4, "min_samples_scale": 0.6,
        }
        tracked = tracker.update(
            detections=targets,
            pcd=pcd_filtered,
            detector=detector,
            loose_query_params=lq_params,
        )

    # 打印统计
    print(f"\n  [帧 {frame_idx}] {os.path.basename(pcd_path)}")
    print(f"    原始点数: {len(pcd_raw.points)}  →  空中区域: {len(pcd_filtered.points)}")
    print(f"    检测目标: {len(targets)}  已确认跟踪: {len(tracked)}")
    for i, t in enumerate(targets):
        c = _COLOR_PALETTE[i % len(_COLOR_PALETTE)]
        print(f"      目标[{i}] pts={t.num_points} size={t.size.round(2)} "
              f"conf={t.confidence:.3f} center={t.center.round(3)}")
    if use_track:
        for t in tracked:
            print(f"      Track[{t.id}] pos={t.pos.round(3)} vel={t.velocity.round(3)} "
                  f"seen={t.total_seen} conf={t.confidence:.3f}")

    # 构建几何体
    geoms = []

    # 原始点云（可选，灰色）
    if show_raw:
        raw_vis = o3d.geometry.PointCloud()
        raw_vis.points = o3d.utility.Vector3dVector(np.asarray(pcd_raw.points))
        raw_vis.paint_uniform_color(_COLOR_RAW.tolist())
        geoms.append(raw_vis)

    # 空中区域点云（稍浅灰）
    if len(pcd_filtered.points) > 0:
        fg_vis = o3d.geometry.PointCloud()
        fg_vis.points = o3d.utility.Vector3dVector(np.asarray(pcd_filtered.points))
        fg_vis.paint_uniform_color([0.5, 0.5, 0.5])
        geoms.append(fg_vis)

    # 各聚类（随机色）+ 包围盒
    for i, t in enumerate(targets):
        c = _COLOR_PALETTE[i % len(_COLOR_PALETTE)]
        tpcd = o3d.geometry.PointCloud()
        tpcd.points = o3d.utility.Vector3dVector(t.points)
        tpcd.paint_uniform_color(c)
        geoms.append(tpcd)
        geoms.append(make_bbox(t.bbox_min, t.bbox_max, color=tuple(c)))

    # 已确认跟踪目标：在卡尔曼估计位置标注球体（黄色）
    if use_track:
        for t in tracked:
            center3d = np.array([t.pos[0], t.pos[1], t.height])
            geoms.append(make_sphere(center3d, radius=0.12, color=(1.0, 1.0, 0.0)))

    return geoms


def main():
    args = parse_args()

    # 加载配置
    yaml = YAML(typ="safe")
    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.load(f)
    air_cfg = load_air_target_config(cfg)

    # 注册全局卡尔曼参数
    kp = air_cfg.tracking.kalman
    AirKalmanFilter.set_global_params(
        q_pos=kp.q_pos, q_vel=kp.q_vel, q_pv=kp.q_pv,
        r_pos=kp.r_pos, r_vel=kp.r_vel,
        decay_rate=kp.decay_rate,
        max_velocity=kp.max_velocity,
        cov_factor=kp.cov_factor,
        stop_p_time=kp.stop_p_time,
        init_p_times=kp.init_p_times,
    )

    processor = AirPointCloudProcessor(air_cfg.preprocessing)
    detector  = AirClusterDetector(air_cfg.clustering, air_cfg.target_filter)

    # 背景减除：在线学习时跳过飞行区域 [z_min, z_max]
    flight_z_min = air_cfg.preprocessing.height_filter.z_min
    flight_z_max = air_cfg.preprocessing.height_filter.z_max
    bg_subtractor = AirBackgroundSubtractor(
        voxel_size=air_cfg.background.voxel_size,
        occupy_threshold=air_cfg.background.occupy_threshold,
        learning_frames=air_cfg.background.learning_frames,
        flight_z_min=flight_z_min,
        flight_z_max=flight_z_max,
    )

    # 背景预加载：命令行 --bg-pcd 优先，否则用配置 bg_pcd_file
    bg_pcd_path = args.bg_pcd or air_cfg.background.bg_pcd_file
    if bg_pcd_path:
        if not os.path.isabs(bg_pcd_path):
            bg_pcd_path = os.path.join(PROJECT_ROOT, bg_pcd_path)
        if os.path.isfile(bg_pcd_path):
            bg_pcd = o3d.io.read_point_cloud(bg_pcd_path)
            bg_pcd = processor.crop_roi(bg_pcd)
            bg_pcd = processor.downsample(bg_pcd)
            added = bg_subtractor.load_from_pcd(bg_pcd)
            print(f"✅ 背景预加载: {os.path.basename(bg_pcd_path)} → "
                  f"{bg_subtractor.background_voxel_count} 个背景体素 (新增 {added})")
        else:
            print(f"⚠️ bg_pcd_file 不存在: {bg_pcd_path}，回退到在线学习")

    # 跟踪器
    tr = air_cfg.tracking
    tracker = AirTargetTracker(
        max_lost_frames=tr.max_lost_frames,
        match_distance_threshold=tr.match_distance,
        force_combine_dist=tr.force_combine_dist,
        cc_thres=tr.cc_thres,
        combine_limit=tr.combine_limit,
        separate_limit=tr.separate_limit,
        confirm_frames=tr.confirm_frames,
        kf_params=None,
    )
    use_track = not args.no_track

    # 确定帧列表
    frame_files = args.frames if args.frames else [args.pcd]

    print(f"配置: flight_z=[{flight_z_min}, {flight_z_max}]  confirm_frames={tr.confirm_frames}")
    print(f"共 {len(frame_files)} 帧")

    all_geoms = []
    for i, pcd_path in enumerate(frame_files):
        geoms = process_single_frame(
            pcd_path=pcd_path,
            processor=processor,
            detector=detector,
            bg_subtractor=bg_subtractor,
            tracker=tracker,
            show_raw=args.show_raw,
            use_track=use_track,
            frame_idx=i,
        )
        all_geoms.extend(geoms)

    if not all_geoms:
        print("没有可显示的几何体")
        return

    # 坐标轴
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    all_geoms.append(axes)

    label = f"air clusters ({len(frame_files)} frames)"
    print(f"\n正在打开 Open3D 窗口：{label}")
    o3d.visualization.draw_geometries(all_geoms, window_name=label)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""验证空中目标跟踪鲁棒性：移动/悬停/进出视野点云序列回放。"""

import argparse
import glob
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
from hnurm_radar.air_scheme.cluster_detector import AirClusterDetector
from hnurm_radar.air_scheme.background_subtractor import AirBackgroundSubtractor
from hnurm_radar.air_scheme.target_tracker import AirTargetTracker


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pcd_dir", required=True, help="序列点云目录")
    p.add_argument("--glob", default="*.pcd", help="匹配模式")
    p.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs", "main_config.yaml"))
    return p.parse_args()


def run_sequence(pcd_files, air_cfg):
    from hnurm_radar.air_scheme.air_kalman_filter import AirKalmanFilter

    processor = AirPointCloudProcessor(air_cfg.preprocessing)
    detector  = AirClusterDetector(air_cfg.clustering, air_cfg.target_filter)

    # 注册全局卡尔曼参数（必须在 AirTargetTracker 实例化之前）
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

    tr = air_cfg.tracking
    tracker = AirTargetTracker(
        max_lost_frames=tr.max_lost_frames,
        match_distance_threshold=tr.match_distance,
        force_combine_dist=tr.force_combine_dist,
        cc_thres=tr.cc_thres,
        combine_limit=tr.combine_limit,
        separate_limit=tr.separate_limit,
        confirm_frames=tr.confirm_frames,
        kf_params=None,  # 已通过 set_global_params 注册
    ) if tr.enabled else None

    # 背景减除：在线学习时跳过飞行区域 [z_min, z_max]，不学入悬停无人机
    flight_z_min = air_cfg.preprocessing.height_filter.z_min
    flight_z_max = air_cfg.preprocessing.height_filter.z_max
    bg = AirBackgroundSubtractor(
        voxel_size=air_cfg.background.voxel_size,
        occupy_threshold=air_cfg.background.occupy_threshold,
        learning_frames=air_cfg.background.learning_frames,
        flight_z_min=flight_z_min,
        flight_z_max=flight_z_max,
    ) if air_cfg.background.enabled else None

    total_detect = 0
    total_track = 0
    lost_events = 0

    for i, fp in enumerate(pcd_files):
        pcd = o3d.io.read_point_cloud(fp)
        pcd = processor.crop_roi(pcd)
        pcd = processor.downsample(pcd)

        if bg is not None:
            bg.learn(pcd)
            pcd = bg.filter(pcd)

        heights = [t.height for t in tracker.get_all_targets()] if tracker else None
        pcd = processor.filter_height(pcd, heights)
        targets = detector.detect(pcd)

        if tracker is not None:
            tracked = tracker.update(
                detections=targets,
                pcd=pcd,
                detector=detector,
            )
            lost_events += len([t for t in tracker.get_all_targets() if t.lost_count == 1])
            total_track += len(tracked)
        total_detect += len(targets)

        print(f"[{i:04d}] detect={len(targets)} track={(len(tracked) if tracker else 0)} file={os.path.basename(fp)}")

    n = max(len(pcd_files), 1)
    print("\n==== 验证结果 ====")
    print(f"帧数: {len(pcd_files)}")
    print(f"平均检测数: {total_detect / n:.3f}")
    if tracker:
        print(f"平均跟踪数: {total_track / n:.3f}")
        print(f"新丢失事件数: {lost_events}")


def main():
    args = parse_args()
    yaml = YAML(typ="safe")
    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.load(f)
    air_cfg = load_air_target_config(cfg)

    files = sorted(glob.glob(os.path.join(args.pcd_dir, args.glob)))
    if not files:
        raise RuntimeError(f"未找到点云文件: {args.pcd_dir}/{args.glob}")

    run_sequence(files, air_cfg)


if __name__ == "__main__":
    main()

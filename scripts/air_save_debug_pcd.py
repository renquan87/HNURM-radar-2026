#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""
air_save_debug_pcd.py — 空中检测调试：保存/查看聚类点云

两种模式：

模式一（推荐）：订阅 debug 话题，保存 RViz 可视化的同一份数据
  python3 scripts/air_save_debug_pcd.py --subscribe

模式二：订阅 /lidar_pcds 原始点云，离线运行检测流程，保存+可视化
  python3 scripts/air_save_debug_pcd.py --offline

通用参数：
  --frames N        保存 N 帧后退出（默认 5）
  --output-dir DIR  保存目录（默认 test_output/air_debug/）
  --view            保存后用 Open3D 打开最后一帧
"""

import argparse
import os
import sys
import time
import numpy as np
import open3d as o3d

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src", "hnurm_radar")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

DEFAULT_OUTPUT = os.path.join(PROJECT_ROOT, "test_output", "air_debug")


def parse_args():
    p = argparse.ArgumentParser(description="空中检测调试：保存/查看聚类点云")
    mode = p.add_mutually_exclusive_group(required=True)
    mode.add_argument("--subscribe", action="store_true",
                       help="订阅 air_debug/* 话题保存点云")
    mode.add_argument("--offline", action="store_true",
                       help="订阅 /lidar_pcds 离线运行检测，保存聚类结果")
    p.add_argument("--frames", type=int, default=5,
                   help="保存帧数（默认 5）")
    p.add_argument("--output-dir", default=DEFAULT_OUTPUT,
                   help=f"保存目录（默认 {DEFAULT_OUTPUT}）")
    p.add_argument("--view", action="store_true",
                   help="保存后用 Open3D 查看最后一帧")
    p.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs", "main_config.yaml"),
                   help="配置文件路径")
    return p.parse_args()


# =====================================================================
# 模式一：订阅 debug 话题
# =====================================================================
def run_subscribe_mode(args):
    """订阅 air_target_node 的 debug 话题，直接保存 PCD"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

    os.makedirs(args.output_dir, exist_ok=True)

    class Capturer(Node):
        def __init__(self):
            super().__init__('air_debug_capture')
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST, depth=5,
            )
            self.sub_air = self.create_subscription(
                PointCloud2, "air_debug/air_points", self._air_cb, qos)
            self.sub_cluster = self.create_subscription(
                PointCloud2, "air_debug/cluster_points", self._cluster_cb, qos)
            self.air_count = 0
            self.cluster_count = 0
            self.max_frames = args.frames
            self.last_air_path = None
            self.last_cluster_path = None
            self.get_logger().info(
                f'等待 air_debug 话题... (保存 {self.max_frames} 帧到 {args.output_dir})')

        def _save_pcd(self, points, path):
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
            o3d.io.write_point_cloud(path, pcd)

        def _air_cb(self, msg):
            pts = np.array(list(pc2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
            points = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1)
            path = os.path.join(args.output_dir, f"air_points_{self.air_count:04d}.pcd")
            self._save_pcd(points, path)
            self.get_logger().info(
                f'[air_points] 帧 {self.air_count}: {len(points)} 点 → {path}')
            self.last_air_path = path
            self.air_count += 1

        def _cluster_cb(self, msg):
            pts = np.array(list(pc2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True)),
                dtype=[("x", np.float32), ("y", np.float32),
                       ("z", np.float32), ("intensity", np.float32)])
            points = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1)
            intensities = pts["intensity"]
            n_clusters = int(intensities.max()) if len(intensities) > 0 else 0

            # 保存全部聚类点
            path = os.path.join(args.output_dir, f"cluster_points_{self.cluster_count:04d}.pcd")
            self._save_pcd(points, path)

            # 分别保存每个聚类
            for cid in range(1, n_clusters + 1):
                mask = intensities == float(cid)
                cpts = points[mask]
                cpath = os.path.join(
                    args.output_dir,
                    f"cluster_{self.cluster_count:04d}_id{cid}.pcd")
                self._save_pcd(cpts, cpath)

            self.get_logger().info(
                f'[clusters] 帧 {self.cluster_count}: '
                f'{len(points)} 点, {n_clusters} 个聚类 → {path}')
            self.last_cluster_path = path
            self.cluster_count += 1

            if self.cluster_count >= self.max_frames:
                self.get_logger().info(f'已保存 {self.max_frames} 帧，退出')
                raise SystemExit(0)

    rclpy.init()
    node = Capturer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        last_path = node.last_cluster_path or node.last_air_path
        node.destroy_node()
        rclpy.shutdown()

    if args.view and last_path and os.path.exists(last_path):
        _view_pcd(last_path)


# =====================================================================
# 模式二：离线运行检测流程
# =====================================================================
def run_offline_mode(args):
    """订阅 /lidar_pcds，离线运行空中检测流程，保存聚类结果"""
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2
    import sensor_msgs_py.point_cloud2 as pc2
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from ruamel.yaml import YAML

    from hnurm_radar.air_scheme.air_config import load_air_target_config
    from hnurm_radar.air_scheme.point_cloud_processor import AirPointCloudProcessor
    from hnurm_radar.air_scheme.cluster_detector import AirClusterDetector
    from hnurm_radar.air_scheme.background_subtractor import AirBackgroundSubtractor
    from hnurm_radar.air_scheme.air_kalman_filter import AirKalmanFilter

    os.makedirs(args.output_dir, exist_ok=True)

    # 加载配置
    yaml = YAML(typ="safe")
    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.load(f)
    air_cfg = load_air_target_config(cfg)

    # 注册卡尔曼参数
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
    detector = AirClusterDetector(air_cfg.clustering, air_cfg.target_filter)
    flight_z_min = air_cfg.preprocessing.height_filter.z_min
    flight_z_max = air_cfg.preprocessing.height_filter.z_max
    bg_sub = AirBackgroundSubtractor(
        voxel_size=air_cfg.background.voxel_size,
        occupy_threshold=air_cfg.background.occupy_threshold,
        learning_frames=air_cfg.background.learning_frames,
        flight_z_min=flight_z_min,
        flight_z_max=flight_z_max,
    )

    # 背景预加载（从配置 bg_pcd_file 加载）
    bg_pcd_path = air_cfg.background.bg_pcd_file
    if bg_pcd_path:
        if not os.path.isabs(bg_pcd_path):
            bg_pcd_path = os.path.join(PROJECT_ROOT, bg_pcd_path)
        if os.path.isfile(bg_pcd_path):
            bg_pcd = o3d.io.read_point_cloud(bg_pcd_path)
            bg_pcd = processor.crop_roi(bg_pcd)
            bg_pcd = processor.downsample(bg_pcd)
            added = bg_sub.load_from_pcd(bg_pcd)
            print(f"✅ 背景预加载: {os.path.basename(bg_pcd_path)} → "
                  f"{bg_sub.background_voxel_count} 个背景体素 (新增 {added})")

    # 颜色方案
    COLORS = [
        [1.0, 0.2, 0.2], [0.2, 0.6, 1.0], [0.2, 0.9, 0.2],
        [1.0, 0.8, 0.0], [0.8, 0.2, 1.0], [0.0, 0.9, 0.9],
    ]

    class OfflineDetector(Node):
        def __init__(self):
            super().__init__('air_offline_detector')
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                history=HistoryPolicy.KEEP_LAST, depth=10,
            )
            self.sub = self.create_subscription(
                PointCloud2, "lidar_pcds", self._cb, qos)
            self.frame_id = 0
            self.saved = 0
            self.last_geoms = []
            self.get_logger().info(
                f'等待 /lidar_pcds 话题... '
                f'(保存 {args.frames} 帧检测结果到 {args.output_dir})')

        def _cb(self, msg):
            t0 = time.time()
            pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.array(list(pts_gen),
                           dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
            points = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)

            # 预处理
            pcd_roi = processor.crop_roi(pcd)
            pcd_down = processor.downsample(pcd_roi)
            bg_sub.learn(pcd_down)
            pcd_fg = bg_sub.filter(pcd_down)
            pcd_filtered = processor.filter_height(pcd_fg)

            # 聚类
            targets = detector.detect(pcd_filtered)

            n_air = len(pcd_filtered.points)
            dt = (time.time() - t0) * 1000
            self.frame_id += 1

            print(f"\n  [帧 {self.frame_id}] raw={len(points)}  air={n_air}  "
                  f"targets={len(targets)}  latency={dt:.1f}ms  "
                  f"bg_voxels={bg_sub.background_voxel_count}")

            if len(targets) == 0:
                return

            for i, t in enumerate(targets):
                print(f"    目标[{i}] pts={t.num_points} "
                      f"size=[{t.size[0]:.2f},{t.size[1]:.2f},{t.size[2]:.2f}] "
                      f"center=[{t.center[0]:.3f},{t.center[1]:.3f},{t.center[2]:.3f}] "
                      f"conf={t.confidence:.3f}")

            # 保存空中区域点云
            air_path = os.path.join(
                args.output_dir, f"offline_air_{self.saved:04d}.pcd")
            o3d.io.write_point_cloud(air_path, pcd_filtered)

            # 保存各聚类
            geoms = []
            for i, t in enumerate(targets):
                cpcd = o3d.geometry.PointCloud()
                cpcd.points = o3d.utility.Vector3dVector(t.points)
                c = COLORS[i % len(COLORS)]
                cpcd.paint_uniform_color(c)
                cpath = os.path.join(
                    args.output_dir, f"offline_cluster_{self.saved:04d}_id{i}.pcd")
                o3d.io.write_point_cloud(cpath, cpcd)
                geoms.append(cpcd)

                # 包围盒
                bbox = o3d.geometry.AxisAlignedBoundingBox(
                    min_bound=t.bbox_min.tolist(), max_bound=t.bbox_max.tolist())
                bbox.color = c
                geoms.append(bbox)

            self.last_geoms = geoms
            self.saved += 1
            print(f"    → 已保存到 {args.output_dir}")

            if self.saved >= args.frames:
                self.get_logger().info(f'已保存 {args.frames} 帧，退出')
                raise SystemExit(0)

    rclpy.init()
    node = OfflineDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        last_geoms = node.last_geoms
        node.destroy_node()
        rclpy.shutdown()

    if args.view and last_geoms:
        print("\n正在打开 Open3D 窗口...")
        axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
        last_geoms.append(axes)
        o3d.visualization.draw_geometries(last_geoms, window_name="Air Clusters (offline)")


# =====================================================================
# Open3D 查看器
# =====================================================================
def _view_pcd(path):
    """用 Open3D 打开 PCD 文件"""
    print(f"\n正在打开: {path}")
    pcd = o3d.io.read_point_cloud(path)
    print(f"点数: {len(pcd.points)}")
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1.0)
    o3d.visualization.draw_geometries(
        [pcd, axes], window_name=f"Debug: {os.path.basename(path)}")


def main():
    args = parse_args()
    if args.subscribe:
        run_subscribe_mode(args)
    else:
        run_offline_mode(args)


if __name__ == "__main__":
    main()

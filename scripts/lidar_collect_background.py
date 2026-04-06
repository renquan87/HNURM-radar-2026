#!/usr/bin/env python3
"""
lidar_collect_background.py — 订阅激光雷达点云并生成背景地图

用途：
  订阅实时点云话题，累积多帧点云后进行体素降采样，保存为背景 `.pcd` 文件，
  供 [`hnurm_radar.lidar_scheme.lidar_node.LidarListener`](src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py:95)
  的背景减除流程使用。
"""

import argparse
import os
import sys
import threading

import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src", "hnurm_radar")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from hnurm_radar.shared.paths import DATA_DIR, resolve_path  # noqa: E402

DEFAULT_OUTPUT = os.path.join(DATA_DIR, "pointclouds", "background", "background.pcd")


def parse_args():
    parser = argparse.ArgumentParser(description="采集激光雷达背景点云并保存为 PCD")
    parser.add_argument("--topic", default="/livox/lidar", help="订阅的点云话题")
    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT,
        help=f"输出背景 PCD 路径，默认 {DEFAULT_OUTPUT}",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.1,
        help="保存前体素降采样大小（米），0 表示不降采样",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=120,
        help="最大累积帧数，0 表示持续采集直到手动中断",
    )
    return parser.parse_args()


class BackgroundCollector(Node):
    def __init__(self, args):
        super().__init__('lidar_background_collector')
        self.args = args
        self.lock = threading.Lock()
        self.all_points = []
        self.frame_count = 0
        self.sub = self.create_subscription(PointCloud2, args.topic, self.callback, 10)
        self.get_logger().info(f'订阅话题: {args.topic}')
        self.get_logger().info(f'目标帧数: {args.max_frames} (0 表示无限)')
        self.get_logger().info(f'输出路径: {resolve_path(args.output)}')

    def callback(self, msg):
        points = np.array(
            list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)),
            dtype=[('x', np.float32), ('y', np.float32), ('z', np.float32)],
        )
        if points.size == 0:
            return

        frame = np.stack([points['x'], points['y'], points['z']], axis=-1).astype(np.float32)
        with self.lock:
            self.all_points.append(frame)
            self.frame_count += 1
            current = self.frame_count

        self.get_logger().info(f'已接收 {current} 帧，当前帧点数 {len(frame)}')

        if self.args.max_frames > 0 and current >= self.args.max_frames:
            self.get_logger().info(f'达到目标帧数 {self.args.max_frames}，准备保存背景地图')
            raise SystemExit(0)


def save_background(collector, output_path, voxel_size):
    with collector.lock:
        if not collector.all_points:
            print('未收集到任何点云数据，未生成背景地图')
            return 1
        combined = np.vstack(collector.all_points)

    print(f'合并后总点数: {len(combined)}')

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(combined.astype(np.float64))

    if voxel_size > 0:
        pcd = pcd.voxel_down_sample(voxel_size)
        print(f'降采样后点数: {len(pcd.points)}，voxel_size={voxel_size}')

    output_path = resolve_path(output_path)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    if not o3d.io.write_point_cloud(output_path, pcd):
        print(f'保存背景地图失败: {output_path}')
        return 1

    print(f'背景地图已保存至: {output_path}')
    return 0


def main(args=None):
    parsed_args = parse_args()
    rclpy.init(args=args)
    collector = BackgroundCollector(parsed_args)

    exit_code = 0
    try:
        rclpy.spin(collector)
    except (KeyboardInterrupt, SystemExit):
        collector.get_logger().info('采集结束，开始写入背景地图')
    finally:
        exit_code = save_background(collector, parsed_args.output, parsed_args.voxel_size)
        collector.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()

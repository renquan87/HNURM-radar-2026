"""
lidar_node.py — 激光雷达点云接收与预处理节点（方案二）

功能：
  订阅 Livox HAP 激光雷达驱动发布的 /livox/lidar 话题（PointCloud2），
  对原始点云进行距离滤波（近距离 + 远距离剔除），将多帧点云累积到
  PcdQueue 队列中，并以 ~10Hz 频率重新发布合并后的原始点云到 /lidar_pcds，
  供 registration 节点进行点云配准；同时基于背景地图执行背景减除，并将
  前景点云发布到 /target_pointcloud，供 radar 节点使用。

数据流：
  /livox/lidar (PointCloud2) → listener_callback() → 距离滤波 → PcdQueue(累积10帧)
                                                        ↓
  publish_point_cloud() → /lidar_pcds (PointCloud2) → registration 节点

  /livox/lidar (PointCloud2) → listener_callback() → 距离滤波 → 背景减除
                                                        ↓
                                           /target_pointcloud (PointCloud2) → radar 节点

配置参数（来自 configs/main_config.yaml → lidar 段）:
  - height_threshold:    地面点高度阈值（当前未启用）
  - min_distance:        近距离滤除阈值（m），默认 1
  - max_distance:        远距离滤除阈值（m），默认 40
  - lidar_topic_name:    点云话题名，默认 "/livox/lidar"
  - background_map_path: 背景点云路径，默认 "data/pointclouds/background/background.pcd"
  - background_threshold:背景减除距离阈值（m），默认 0.2
  - voxel_size:          背景地图降采样体素大小（m），默认 0.1
  - publish_projected:   是否发布背景减除后的点云，默认 True
"""

import multiprocessing
import os
import threading
import time
from collections import deque

import numpy as np
import open3d as o3d
import rclpy
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ruamel.yaml import YAML
from scipy.spatial import cKDTree
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

from ..shared.paths import DATA_DIR, MAIN_CONFIG_PATH, resolve_path


class Pcd:
    def __init__(self, pcd_name=""):
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_name = pcd_name

    def set_pcd(self, pcd):
        self.pcd = pcd

    def update_pcd_points(self, pc):
        self.pcd.points = o3d.utility.Vector3dVector(pc)


class PcdQueue(object):
    def __init__(self, max_size=90, voxel_size=0.05):
        self.max_size = max_size
        self.queue = deque(maxlen=max_size)
        self.pc_all = np.empty((0, 3), dtype=np.float64)
        self.record_times = 0
        self.point_num = 0

    def add(self, pc):
        self.queue.append(pc)
        if self.record_times < self.max_size:
            self.record_times += 1

        self.update_pc_all()
        self.point_num = self.cal_point_num()

    def get_all_pc(self):
        return self.pc_all

    def update_pc_all(self):
        if not self.queue:
            self.pc_all = np.empty((0, 3), dtype=np.float64)
            return
        self.pc_all = np.vstack(self.queue)

    def is_full(self):
        return self.record_times == self.max_size

    def cal_point_num(self):
        return len(self.pc_all)


class LidarListener(Node):
    def __init__(self, cfg):
        super().__init__('lidar_listener')
        qos__lidar_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        lidar_cfg = cfg["lidar"]
        self.height_threshold = lidar_cfg["height_threshold"]
        self.min_distance = lidar_cfg["min_distance"]
        self.max_distance = lidar_cfg["max_distance"]
        self.lidar_topic_name = lidar_cfg["lidar_topic_name"]

        # rosbag 模式下使用 rosbag 中点云的 frame_id（livox_frame），
        # 以便 registration 节点发布正确的 TF child_frame_id
        camera_cfg = cfg.get("camera", {})
        camera_mode = camera_cfg.get("mode", "hik")
        if camera_mode == "rosbag":
            self._cloud_frame_id = camera_cfg.get("tf_source_frame", "livox_frame")
        else:
            self._cloud_frame_id = "livox"

        default_background_path = lidar_cfg.get(
            "background_map_path",
            os.path.join("data", "pointclouds", "background", "background.pcd")
        )
        self.declare_parameter('background_map_path', default_background_path)
        self.declare_parameter('background_threshold', lidar_cfg.get('background_threshold', 0.2))
        self.declare_parameter('voxel_size', lidar_cfg.get('voxel_size', 0.1))
        self.declare_parameter('publish_projected', lidar_cfg.get('publish_projected', True))

        self.background_map_path = resolve_path(str(self.get_parameter('background_map_path').value))
        self.background_threshold = float(self.get_parameter('background_threshold').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.publish_projected = bool(self.get_parameter('publish_projected').value)
        self.num_cores = max(1, multiprocessing.cpu_count())

        self.pcdQueue = PcdQueue(max_size=10)

        self.sub_livox = self.create_subscription(
            PointCloud2,
            self.lidar_topic_name,
            self.listener_callback,
            10,
        )
        self.pub_pcds = self.create_publisher(PointCloud2, "lidar_pcds", qos__lidar_profile)
        self.pub_targets = self.create_publisher(PointCloud2, "target_pointcloud", qos__lidar_profile)

        self.background_pcd = None
        self.bg_tree = None
        self.load_background_map()

        self.publish_thread = threading.Thread(target=self.publish_point_cloud, daemon=True)
        self.publish_thread.start()

    def _build_cloud_msg(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._cloud_frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        return pc2.create_cloud(header, fields, points)

    def _read_points_array(self, msg):
        points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float64)

    def publish_point_cloud(self):
        last_time = time.time()
        while rclpy.ok():
            cur_time = time.time()
            _delta_time = cur_time - last_time
            last_time = cur_time
            time.sleep(0.1)

            points = self.get_all_pc()
            if len(points) > 0:
                self.pub_pcds.publish(self._build_cloud_msg(points))

    def load_background_map(self):
        if not self.background_map_path:
            self.get_logger().warn('未配置背景地图路径，背景减除将直接透传实时点云。')
            self.background_pcd = None
            self.bg_tree = None
            return

        self.get_logger().info(f'加载背景地图: {self.background_map_path}')
        if not os.path.exists(self.background_map_path):
            self.get_logger().warn(f'背景地图文件不存在: {self.background_map_path}，背景减除将直接透传实时点云。')
            self.background_pcd = None
            self.bg_tree = None
            return

        pcd = o3d.io.read_point_cloud(self.background_map_path)
        if pcd.is_empty():
            self.get_logger().warn('背景地图加载失败或为空，背景减除将直接透传实时点云。')
            self.background_pcd = None
            self.bg_tree = None
            return

        if self.voxel_size > 0.0:
            pcd = pcd.voxel_down_sample(self.voxel_size)

        bg_points = np.asarray(pcd.points, dtype=np.float32)
        if bg_points.size == 0:
            self.get_logger().warn('背景地图降采样后为空，背景减除将直接透传实时点云。')
            self.background_pcd = None
            self.bg_tree = None
            return

        self.background_pcd = pcd
        self.bg_tree = cKDTree(bg_points)
        self.get_logger().info(
            f'背景地图加载完成，降采样后点数: {len(bg_points)}，阈值: {self.background_threshold:.3f} m'
        )

    def background_subtraction(self, current_points):
        if self.bg_tree is None or current_points.shape[0] == 0:
            return current_points

        query_points = np.asarray(current_points, dtype=np.float32)
        distances, _ = self.bg_tree.query(query_points, k=1, workers=self.num_cores)
        mask = distances > self.background_threshold
        return current_points[mask]

    def publish_target(self, points):
        if not self.publish_projected:
            return
        self.pub_targets.publish(self._build_cloud_msg(points))

    def listener_callback(self, data):
        points = self._read_points_array(data)
        if points.shape[0] > 0:
            dist = np.linalg.norm(points, axis=1)
            points = points[dist > self.min_distance]
            dist = np.linalg.norm(points, axis=1)
            points = points[dist < self.max_distance]

        target_points = self.background_subtraction(points)
        self.publish_target(target_points)
        self.pcdQueue.add(points)

    def get_all_pc(self):
        return self.pcdQueue.get_all_pc()


def main(args=None):
    main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='Utf-8', mode='r'))
    rclpy.init(args=args)
    lidar = LidarListener(main_cfg)
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

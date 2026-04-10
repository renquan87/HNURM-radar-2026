"""
lidar_node.py — 激光雷达点云接收与预处理节点（方案二）
[订阅 registration 全局地图版 —— 持续更新背景，绝不影响配准]

功能：
  订阅 Livox HAP 激光雷达驱动发布的 /livox/lidar 话题（PointCloud2），
  对原始点云进行距离滤波，累积并发布到 /lidar_pcds（供配准节点使用）。
  同时订阅配准节点发布的 /global_pcd_map，每次收到新地图时利用当前 TF
  将其变换到雷达坐标系并更新背景 KDTree，实现动态背景减除。

数据流：
  /livox/lidar → listener_callback → 距离滤波 → PcdQueue → /lidar_pcds (100Hz)
                        ↓
                   背景减除 (使用最新 bg_tree) → /target_pointcloud

  /global_pcd_map → map_callback → 获取 TF → 变换地图 → 更新 bg_tree
                                                      ↓
                                          /background_map_debug (调试用)

配置参数（在 configs/main_config.yaml → lidar 段）:
  - height_threshold:    地面点高度阈值（当前未启用）
  - min_distance:        近距离滤除阈值（m）
  - max_distance:        远距离滤除阈值（m）
  - lidar_topic_name:    点云话题名
  - background_threshold:背景减除距离阈值（m）
  - publish_projected:   是否发布前景点云
  - global_map_topic:    全局地图话题名，默认 "/global_pcd_map"
  - publish_debug_map:   是否发布变换后的背景地图用于调试，默认 True
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
import tf2_ros
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from ruamel.yaml import YAML
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
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

        # 背景减除参数
        self.declare_parameter('background_threshold', lidar_cfg.get('background_threshold', 0.2))
        self.declare_parameter('publish_projected', lidar_cfg.get('publish_projected', True))
        self.background_threshold = float(self.get_parameter('background_threshold').value)
        self.publish_projected = bool(self.get_parameter('publish_projected').value)
        self.num_cores = max(1, multiprocessing.cpu_count())

        # 全局地图话题名
        self.declare_parameter('global_map_topic', lidar_cfg.get('global_map_topic', '/global_pcd_map'))
        self.global_map_topic = self.get_parameter('global_map_topic').value

        # 调试发布选项
        self.declare_parameter('publish_debug_map', lidar_cfg.get('publish_debug_map', True))
        self.publish_debug_map = self.get_parameter('publish_debug_map').value

        # ---------- 原始点云累积队列 ----------
        self.pcdQueue = PcdQueue(max_size=10)

        # ---------- 订阅与发布 ----------
        self.sub_livox = self.create_subscription(
            PointCloud2,
            self.lidar_topic_name,
            self.listener_callback,
            10,
        )
        self.pub_pcds = self.create_publisher(PointCloud2, "lidar_pcds", qos__lidar_profile)
        self.pub_targets = self.create_publisher(PointCloud2, "target_pointcloud", qos__lidar_profile)

        # 调试发布者（变换后的背景地图）
        if self.publish_debug_map:
            self.pub_bg_debug = self.create_publisher(PointCloud2, "background_map_debug", qos__lidar_profile)

        # ---------- 原始发布线程（约100Hz） ----------
        self.publish_thread = threading.Thread(target=self.publish_point_cloud, daemon=True)
        self.publish_thread.start()

        # ===== 新增：订阅全局地图并进行 TF 变换 =====
        # 判断是否为 rosbag 模式，动态设置 frame_id
        camera_cfg = cfg.get("camera", {})
        self._camera_mode = camera_cfg.get("mode", "hik")

        if self._camera_mode == "rosbag":
            # rosbag 模式：从配置读取 TF frame 名称
            self.lidar_frame = camera_cfg.get("tf_source_frame", "livox_frame")
            self.map_frame = camera_cfg.get("tf_target_frame", "map")
            self.get_logger().info(
                f"[rosbag 模式] TF: {self.lidar_frame} → {self.map_frame}"
            )
        else:
            # 正常模式：使用默认值
            self.lidar_frame = "livox"
            self.map_frame = "map"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bg_tree = None            # 雷达坐标系下的 KDTree
        self.bg_tree_lock = threading.Lock()

        # 订阅配准节点发布的全局地图（QoS 与发布方兼容）
        self.map_sub = self.create_subscription(
            PointCloud2,
            self.global_map_topic,
            self.map_callback,
            rclpy.qos.QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=rclpy.qos.DurabilityPolicy.VOLATILE,  # 必须与发布方一致
                depth=1
            )
        )
        # ==============================================

        self.get_logger().info('LidarListener 初始化完成，等待全局地图以启用背景减除...')

    def _build_cloud_msg(self, points):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.lidar_frame
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
        """发布累积点云线程（约 100Hz）—— 原始代码，一字未改"""
        last_time = time.time()
        while rclpy.ok():
            cur_time = time.time()
            _delta_time = cur_time - last_time
            last_time = cur_time
            time.sleep(0.01)

            points = self.pcdQueue.get_all_pc()
            if len(points) > 0:
                self.pub_pcds.publish(self._build_cloud_msg(points))

    def map_callback(self, msg):
        """
        全局地图回调：每次收到配准节点发布的最新地图后，
        立即获取当前 TF 并变换到雷达坐标系，更新背景 KDTree。
        """
        points_map = self._read_points_array(msg)
        if points_map.shape[0] == 0:
            self.get_logger().warn('收到的全局地图点数为空')
            return

        self.get_logger().info(f'收到全局地图，点数: {len(points_map)}，开始 TF 变换...')

        try:
            transform = self.tf_buffer.lookup_transform(
                self.lidar_frame,
                self.map_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
        except Exception as e:
            self.get_logger().warn(f'获取 TF 失败: {e}，本次地图更新跳过')
            return

        # 构造变换矩阵 T_map_to_livox（从 map 到 livox）
        t = transform.transform.translation
        r = transform.transform.rotation
        T_map_to_livox = np.eye(4)
        T_map_to_livox[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T_map_to_livox[:3, 3] = [t.x, t.y, t.z]

        # 变换点云到雷达坐标系
        ones = np.ones((points_map.shape[0], 1))
        points_hom = np.hstack([points_map, ones])
        points_lidar = (T_map_to_livox @ points_hom.T).T[:, :3].astype(np.float32)

        # 更新 KDTree（原子操作）
        with self.bg_tree_lock:
            self.bg_tree = cKDTree(points_lidar)

        self.get_logger().info(
            f'背景地图已更新，雷达系点数: {len(points_lidar)}，'
            f'阈值: {self.background_threshold:.3f} m'
        )

        # 调试：发布变换后的背景地图
        if self.publish_debug_map:
            debug_msg = self._build_cloud_msg(points_lidar)
            self.pub_bg_debug.publish(debug_msg)
            self.get_logger().debug('已发布背景地图到 /background_map_debug')

    def background_subtraction(self, current_points):
        """背景减除：移除与背景地图点距离小于阈值的点"""
        with self.bg_tree_lock:
            tree = self.bg_tree

        if tree is None or current_points.shape[0] == 0:
            return current_points

        query_points = np.asarray(current_points, dtype=np.float32)
        distances, _ = tree.query(query_points, k=1, workers=self.num_cores)
        mask = distances > self.background_threshold
        return current_points[mask]

    def publish_target(self, points):
        if not self.publish_projected:
            return
        if len(points) == 0:
            return
        self.pub_targets.publish(self._build_cloud_msg(points))

    def listener_callback(self, data):
        """点云回调：距离滤波 → 背景减除 → 累积原始点云（原始逻辑未改动）"""
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
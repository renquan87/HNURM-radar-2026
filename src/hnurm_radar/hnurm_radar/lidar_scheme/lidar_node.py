"""
lidar_node.py — 激光雷达动态点云提取节点（由 C++ 版本迁移）
保持原有 Python 节点对外接口不变，不引入 vision_interface 依赖。

订阅：
  /livox/lidar （PointCloud2）
发布：
  /lidar_pcds        : 累积原始点云（距离滤波后）
  /target_pointcloud : 累积动态点云（与静态地图差异）
  /livox/lidar_other : 调试用，累积的“other”点云
  /background_map_debug : 调试用，变换后的静态地图
"""

import multiprocessing
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


class PcdQueue:
    """原始点云累积队列（保持原样）"""
    def __init__(self, max_size=10):
        self.max_size = max_size
        self.queue = deque(maxlen=max_size)
        self.pc_all = np.empty((0, 3), dtype=np.float64)

    def add(self, pc):
        self.queue.append(pc)
        self.update_pc_all()

    def get_all_pc(self):
        return self.pc_all

    def update_pc_all(self):
        if not self.queue:
            self.pc_all = np.empty((0, 3), dtype=np.float64)
            return
        self.pc_all = np.vstack(self.queue)


class DynamicCloudNode(Node):
    def __init__(self, cfg):
        super().__init__('dynamic_cloud_node')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        lidar_cfg = cfg["lidar"]
        self.min_distance = lidar_cfg["min_distance"]
        self.max_distance = lidar_cfg["max_distance"]
        self.lidar_topic_name = lidar_cfg["lidar_topic_name"]

        # ---------- 静态地图加载 ----------
        pcd_path = resolve_path("/home/syh/rm_lidar_2027/HNURM-radar-2026/data/pointclouds/background/RM2025.pcd")
        self.map_cloud = self._load_and_filter_map(pcd_path, leaf_size=0.1)
        self.kd_tree = cKDTree(self.map_cloud)
        self.get_logger().info(f"静态地图加载完成，点数: {len(self.map_cloud)}")

        # ---------- 参数 ----------
        self.accumulate_time = 3                     # 累积帧数
        self.background_threshold = 0.1              # 动态点判定距离阈值
        self.num_cores = max(1, multiprocessing.cpu_count())

        # TF 设置
        camera_cfg = cfg.get("camera", {})
        self._camera_mode = camera_cfg.get("mode", "hik")
        if self._camera_mode == "rosbag":
            self.lidar_frame = camera_cfg.get("tf_source_frame", "livox_frame")
            self.map_frame = camera_cfg.get("tf_target_frame", "map")
        else:
            self.lidar_frame = "livox"
            self.map_frame = "map"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ---------- 点云累积容器 ----------
        self.pcdQueue = PcdQueue(max_size=10)                # 原始点云
        self.accumulated_clouds = deque(maxlen=self.accumulate_time)   # 动态点云
        self.other_accumulated_clouds = deque(maxlen=self.accumulate_time)  # other 点云

        # ---------- 订阅与发布 ----------
        self.sub_livox = self.create_subscription(
            PointCloud2, self.lidar_topic_name, self.callback, 10
        )
        self.pub_pcds = self.create_publisher(PointCloud2, "lidar_pcds", qos_profile)
        self.pub_targets = self.create_publisher(PointCloud2, "target_pointcloud", qos_profile)
        self.pub_other = self.create_publisher(PointCloud2, "livox/lidar_other", qos_profile)  # 调试用

        # 调试：发布变换后的静态地图（可选）
        self.pub_bg_debug = self.create_publisher(PointCloud2, "background_map_debug", qos_profile)

        # 原始发布线程（保持 100Hz 发布 /lidar_pcds）
        self.publish_thread = threading.Thread(target=self._publish_raw_loop, daemon=True)
        self.publish_thread.start()

        self.get_logger().info('DynamicCloudNode (Python迁移版) 启动完成')

    def _load_and_filter_map(self, pcd_path, leaf_size):
        """加载 PCD 文件并进行体素滤波"""
        pcd = o3d.io.read_point_cloud(pcd_path)
        if pcd.is_empty():
            self.get_logger().error(f"无法读取地图文件: {pcd_path}")
            return np.empty((0, 3), dtype=np.float32)
        pcd = pcd.voxel_down_sample(voxel_size=leaf_size)
        return np.asarray(pcd.points, dtype=np.float32)

    def _build_cloud_msg(self, points, frame_id=None):
        """将 numpy 点云转换为 PointCloud2 消息"""
        if frame_id is None:
            frame_id = self.map_frame  # 累积点云已变换至地图系
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        return pc2.create_cloud(header, fields, points)

    def _read_points_array(self, msg):
        """从 PointCloud2 读取 xyz 数组"""
        points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        return np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float32)

    def _publish_raw_loop(self):
        """持续发布累积的原始点云（保持原接口）"""
        last_pub_time = time.time()
        while rclpy.ok():
            time.sleep(0.01)  # ~100Hz
            points = self.pcdQueue.get_all_pc()
            if len(points) > 0:
                self.pub_pcds.publish(self._build_cloud_msg(points, frame_id=self.lidar_frame))
                # 每秒打印一次发布统计（避免刷屏）
                now = time.time()
                if now - last_pub_time > 1.0:
                    self.get_logger().debug(
                        f"/lidar_pcds 发布 {len(points)} 点，队列长度 {len(self.pcdQueue.queue)}"
                    )
                    last_pub_time = now

    # ---------- 区域过滤函数（与 C++ 完全一致）----------
    @staticmethod
    def _dart_filter(point):
        """飞镖区域"""
        x, y, z = point
        return (x > 28 - 0.5889 - 0.1885 and x < 28 - 0.5889) and \
               (y > 3.925 and y < 4.525) and \
               (z > 2.4722 - 0.859 + 0.1 and z < 2.4722)

    @staticmethod
    def _fly_safe_filter(point):
        """飞机起飞安全区"""
        x, y, z = point
        return (x > 28 - 2.775 and x < 27.5) and \
               (y > 0.2 and y < 2.2) and \
               (z > 1.7 and z < 3)

    @staticmethod
    def _fly_warn_filter(point):
        """飞机警告区"""
        x, y, z = point
        return (x > 19.83 and x < 28 - 2.7) and \
               (y > 0.2 and y < 1.356 + 2.4 + 0.8) and \
               (z > 1.7 and z < 3)

    @staticmethod
    def _fly_alarm_filter(point):
        """飞机报警区"""
        x, y, z = point
        return (x > 13 and x < 20.5) and \
               (y > 0.2 and y < 1.356 + 2.4 + 0.8) and \
               (z > 1.7 and z < 3)

    # ---------- 主要回调 ----------
    def callback(self, msg):
        t_start = time.perf_counter()

        # 1. 读取并距离滤波
        points_lidar = self._read_points_array(msg)
        if points_lidar.shape[0] > 0:
            dist = np.linalg.norm(points_lidar, axis=1)
            mask = (dist > self.min_distance) & (dist < self.max_distance)
            points_lidar = points_lidar[mask]
        else:
            points_lidar = np.empty((0, 3), dtype=np.float32)

        # ⚠️ 关键修正：无论后续 TF 是否成功，先将滤波后的点云加入原始队列，
        # 保证 /lidar_pcds 的稳定输出（配准必需）
        if points_lidar.shape[0] > 0:
            self.pcdQueue.add(points_lidar)
            self.get_logger().debug(
                f"原始队列新增 {points_lidar.shape[0]} 点，累计点数: {len(self.pcdQueue.get_all_pc())}"
            )
        else:
            self.get_logger().debug("当前帧距离滤波后无有效点")

        # 2. 获取 TF 变换 (lidar_frame -> map_frame)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF 获取失败: {e}，跳过本帧动态点云处理")
            return  # 此时原始队列已更新，配准不受影响

        t_tf = time.perf_counter()

        # 构建变换矩阵
        t = transform.transform.translation
        r = transform.transform.rotation
        T = np.eye(4)
        T[:3, :3] = Rotation.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]

        # 变换点云到地图坐标系
        if points_lidar.shape[0] > 0:
            ones = np.ones((points_lidar.shape[0], 1))
            points_hom = np.hstack([points_lidar, ones])
            points_map = (T @ points_hom.T).T[:, :3].astype(np.float32)
        else:
            points_map = np.empty((0, 3), dtype=np.float32)

        # 3. 区域过滤（分为 filtered 和 other）
        filtered = []
        other = []
        if points_map.shape[0] > 0:
            x, y, z = points_map[:, 0], points_map[:, 1], points_map[:, 2]

            # 主过滤条件（C++ 中的复杂条件）
            main_mask = (x < 3) | (x > 28) | (y < 0) | (y > 15) | (z < 0) | (z > 1.4) | \
                        ((y > 0) & (y < 5) & (x > 25)) | \
                        (((21.5 - 2.9/np.sqrt(2)) < (x + y)) & ((x + y) < (21.5 + 2.9/np.sqrt(2))) &
                         ((-6.5 - 0.9/np.sqrt(2)) < (y - x)) & ((y - x) < (-6.5 + 0.9/np.sqrt(2))))

            # 属于 other 的条件（在飞镖或飞机区域内）
            # 注意：_dart_filter 接受单点，这里对数组逐点判断
            dart_mask = np.apply_along_axis(self._dart_filter, 1, points_map)
            other_mask = dart_mask | ((x > 13) & (x < 27.5) & (y > 0.2) & (y < 1.356 + 2.4 + 0.8) & (z > 1.7) & (z < 3))

            # 分类
            for i in range(points_map.shape[0]):
                if main_mask[i]:
                    if other_mask[i]:
                        other.append(points_map[i])
                    continue
                filtered.append(points_map[i])

        filtered = np.array(filtered, dtype=np.float32) if filtered else np.empty((0, 3), dtype=np.float32)
        other = np.array(other, dtype=np.float32) if other else np.empty((0, 3), dtype=np.float32)

        # 4. 动态点云提取（基于静态地图 KDTree）
        dynamic_points = self._extract_dynamic(filtered, self.background_threshold)

        # 5. 累积
        self.accumulated_clouds.append(dynamic_points)
        self.other_accumulated_clouds.append(other)

        # 合并累积点云
        acc_dynamic = np.vstack(self.accumulated_clouds) if self.accumulated_clouds else np.empty((0, 2))
        acc_other = np.vstack(self.other_accumulated_clouds) if self.other_accumulated_clouds else np.empty((0, 3))

        # 6. 发布结果（保持原接口）
        if acc_dynamic.shape[0] > 0:
            self.pub_targets.publish(self._build_cloud_msg(acc_dynamic, frame_id=self.map_frame))
        if acc_other.shape[0] > 0:
            self.pub_other.publish(self._build_cloud_msg(acc_other, frame_id=self.map_frame))

        # 7. 飞镖/飞机检测（仅日志输出，不发布消息）
        self._detect_and_log(acc_other)

        # 调试：发布静态地图（变换到当前帧）
        if self.pub_bg_debug.get_subscription_count() > 0:
            self.pub_bg_debug.publish(self._build_cloud_msg(self.map_cloud, frame_id=self.map_frame))

        t_end = time.perf_counter()
        self.get_logger().debug(
            f"回调耗时: {(t_end - t_start)*1000:.2f} ms, TF耗时: {(t_tf - t_start)*1000:.2f} ms"
        )

    def _extract_dynamic(self, points, threshold):
        """利用 KDTree 提取与静态地图距离大于阈值的点"""
        if points.shape[0] == 0:
            return np.empty((0, 3), dtype=np.float32)
        dist, _ = self.kd_tree.query(points, k=1, workers=self.num_cores)
        mask = dist > threshold
        return points[mask]

    def _detect_and_log(self, other_points):
        """飞镖/飞机检测，仅输出日志"""
        if other_points.shape[0] == 0:
            return

        # 飞镖检测
        dart_mask = np.apply_along_axis(self._dart_filter, 1, other_points)
        if np.sum(dart_mask) > 5:
            self.get_logger().warn(f"发现飞镖！点数: {np.sum(dart_mask)}")

        # 飞机区域检测
        safe_mask = np.apply_along_axis(self._fly_safe_filter, 1, other_points)
        warn_mask = np.apply_along_axis(self._fly_warn_filter, 1, other_points)
        alarm_mask = np.apply_along_axis(self._fly_alarm_filter, 1, other_points)

        cnt_safe = np.sum(safe_mask)
        cnt_warn = np.sum(warn_mask)
        cnt_alarm = np.sum(alarm_mask)

        if cnt_alarm > 40:
            self.get_logger().error("🚨 飞机报警区域检测到目标！")
        elif cnt_warn > 40:
            self.get_logger().warn("⚠️  飞机警告区域检测到目标！")
        elif cnt_safe > 40:
            self.get_logger().warn("✈️  飞机安全区域检测到目标！")


def main(args=None):
    main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
    rclpy.init(args=args)
    node = DynamicCloudNode(main_cfg)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
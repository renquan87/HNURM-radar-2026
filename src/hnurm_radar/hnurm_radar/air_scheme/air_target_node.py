"""
air_target_node.py — 空中机器人检测 ROS2 节点

方案三：纯激光雷达空中机器人定位。
不调用相机，通过点云聚类完成空中机器人检测和定位，
将检测结果以 Locations 消息发布到 /location 话题，
复用已有的 ekf_node → judge_messager → display_panel 下游链路。

数据流：
  /livox/lidar (PointCloud2) → lidar_node → /lidar_pcds (PointCloud2)
                                                ↓
                                        air_target_node (本节点)
                                          ├── 点云预处理 (ROI + 高度过滤)
                                          ├── 背景减除
                                          ├── DBSCAN 聚类
                                          ├── 卡尔曼跟踪
                                          └── 坐标变换 (雷达 → 赛场)
                                                ↓
                                        /location (Locations) → ekf_node
                                                                   ↓
                                                /ekf_location_filtered → judge_messager + display_panel

空中机器人 ID 约定：
  - 红方空中机器人: car_id = 6（裁判系统协议）
  - 蓝方空中机器人: car_id = 106
  - 赛场有两架空中机器人（红蓝各一），通过赛场半区判断敌我

订阅话题：
  - /lidar_pcds (PointCloud2) — 由 lidar_node 发布的累积点云

发布话题：
  - /location (Locations) — 空中机器人赛场坐标（供 ekf_node 滤波）
"""

import logging
import time
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from detect_result.msg import Location, Locations
from ruamel.yaml import YAML
import numpy as np
import open3d as o3d

from ..shared.paths import MAIN_CONFIG_PATH, FIELD_WIDTH, FIELD_HEIGHT
from .air_config import load_air_target_config
from .point_cloud_processor import AirPointCloudProcessor
from .cluster_detector import AirClusterDetector
from .background_subtractor import AirBackgroundSubtractor
from .target_tracker import AirTargetTracker

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

logger = logging.getLogger(__name__)


class AirTargetNode(Node):
    """空中机器人检测 ROS2 节点"""

    def __init__(self):
        super().__init__('air_target_node')
        self.get_logger().info('Air Target Node 正在初始化...')

        # 加载配置
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        self.my_color = main_cfg["global"]["my_color"]
        self.air_cfg = load_air_target_config(main_cfg)

        if not self.air_cfg.enabled:
            self.get_logger().warn('空中机器人检测已禁用 (air_target.enabled = false)')
            return

        # 空中机器人 ID：
        # 红方空中=6, 蓝方空中=106（按照裁判系统协议）
        # 敌方空中机器人编号取决于我方颜色
        if self.my_color == "Red":
            self.enemy_air_id = 106   # 蓝方空中
            self.ally_air_id = 6      # 红方空中
            self.enemy_label = "Blue"
            self.ally_label = "Red"
        else:
            self.enemy_air_id = 6     # 红方空中
            self.ally_air_id = 106    # 蓝方空中
            self.enemy_label = "Red"
            self.ally_label = "Blue"

        # 核心模块
        self.processor = AirPointCloudProcessor(self.air_cfg.preprocessing)
        self.detector = AirClusterDetector(
            self.air_cfg.clustering,
            self.air_cfg.target_filter,
        )
        # 坐标变换：复用现有 registration 节点的 ICP TF（livox → map）
        # 与 lidar_scheme/radar.py 方式完全一致，实验室/赛场均适用
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.radar_to_field = None   # 4x4 齐次变换矩阵
        self.tf_timer = self.create_timer(1.0, self._tf_timer_callback)

        # 跟踪器
        self.tracker = None
        if self.air_cfg.tracking.enabled:
            kp = self.air_cfg.tracking.kalman
            self.tracker = AirTargetTracker(
                max_lost_frames=self.air_cfg.tracking.max_lost_frames,
                match_distance_threshold=self.air_cfg.tracking.match_distance,
                force_combine_dist=self.air_cfg.tracking.force_combine_dist,
                cc_thres=self.air_cfg.tracking.cc_thres,
                kf_params={
                    'q_pos': kp.q_pos, 'q_vel': kp.q_vel, 'q_pv': kp.q_pv,
                    'r_pos': kp.r_pos, 'r_vel': kp.r_vel,
                    'decay_rate': kp.decay_rate,
                    'max_velocity': kp.max_velocity,
                    'cov_factor': kp.cov_factor,
                    'stop_p_time': kp.stop_p_time,
                }
            )

        # 背景减除
        self.bg_subtractor = None
        if self.air_cfg.background.enabled:
            self.bg_subtractor = AirBackgroundSubtractor(
                voxel_size=self.air_cfg.background.voxel_size,
                occupy_threshold=self.air_cfg.background.occupy_threshold,
                learning_frames=self.air_cfg.background.learning_frames,
                learn_z_max=self.air_cfg.preprocessing.height_filter.z_min,
            )

        # 多帧缓冲
        self.frame_buffer = []
        self.buffer_enabled = self.air_cfg.buffer.enabled
        self.buffer_count = self.air_cfg.buffer.frame_count

        # 最新点云（线程安全）
        self._latest_points = None
        self._points_lock = threading.Lock()

        # QoS
        qos_lidar = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        qos_pub = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # 订阅累积点云（来自 lidar_node 的 /lidar_pcds）
        self.sub_pcds = self.create_subscription(
            PointCloud2, "lidar_pcds", self._pcds_callback, qos_lidar
        )

        # 发布空中机器人坐标（复用 Locations 消息类型，发布到 /location 话题）
        self.pub_location = self.create_publisher(
            Locations, "location", qos_pub
        )

        # 定时处理（按配置的发布频率）
        interval = 1.0 / self.air_cfg.publish_rate
        self.timer = self.create_timer(interval, self._process_callback)

        self.frame_id = 0
        self.get_logger().info(
            f'Air Target Node 初始化完成: '
            f'我方={self.my_color}, '
            f'发布频率={self.air_cfg.publish_rate}Hz, '
            f'跟踪={"ON" if self.tracker else "OFF"}, '
            f'背景减除={"ON" if self.bg_subtractor else "OFF"}'
        )

    def _pcds_callback(self, msg: PointCloud2):
        """接收 lidar_node 发布的累积点云"""
        try:
            points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(
                list(points),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
            )
            points = np.stack(
                [points["x"], points["y"], points["z"]], axis=-1
            ).astype(np.float64)

            with self._points_lock:
                self._latest_points = points
        except Exception as e:
            self.get_logger().warn(f'点云接收异常: {e}')

    # ---- TF 变换（复用 registration ICP 结果，与 radar.py 一致）----

    def _tf_timer_callback(self):
        """定时查询 livox → map 的 TF 变换"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox',
                time=rclpy.time.Time(),
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            self.radar_to_field = self._tf_to_matrix(t, r)
        except TransformException as ex:
            if self.radar_to_field is None:
                self.get_logger().warn(f'等待 TF (livox→map): {ex}')

    @staticmethod
    def _quaternion_to_rotation_matrix(q):
        """四元数 → 3×3 旋转矩阵"""
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])

    def _tf_to_matrix(self, translation, rotation):
        """TF → 4×4 齐次变换矩阵"""
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        R = self._quaternion_to_rotation_matrix(q)
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3] = [translation.x, translation.y, translation.z]
        return M

    def _lidar_to_field(self, x, y, z):
        """雷达坐标 → 赛场坐标（通过 TF 矩阵）

        与 lidar_scheme/radar.py 中的坐标变换方式完全一致，
        实验室模式和赛场模式均由 registration 节点提供不同的 TF。
        """
        if self.radar_to_field is None:
            return None
        pt = np.array([x, y, z, 1.0])
        field_pt = self.radar_to_field @ pt
        return field_pt[:3]  # [field_x, field_y, field_z]

    def _process_callback(self):
        """定时处理：从最新点云中检测空中目标"""
        # 获取最新点云
        with self._points_lock:
            points = self._latest_points
            # 不清空，允许重复处理最后已知的点云

        if points is None or len(points) == 0:
            return

        t0 = time.time()

        # 构建 Open3D 点云
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        n_raw = len(points)

        # 1. ROI 裁剪 + 降采样（不含高度过滤）
        pcd_roi = self.processor.crop_roi(pcd)
        pcd_down = self.processor.downsample(pcd_roi)

        # 2. 背景减除
        if self.bg_subtractor:
            self.bg_subtractor.learn(pcd_down)
            pcd_fg = self.bg_subtractor.filter(pcd_down)
        else:
            pcd_fg = pcd_down

        # 3. 高度过滤（提取空中区域）
        pcd_filtered = self.processor.filter_height(pcd_fg)
        n_filtered = len(pcd_filtered.points)

        # 4. 多帧缓冲
        if self.buffer_enabled:
            self.frame_buffer.append(pcd_filtered)
            if len(self.frame_buffer) > self.buffer_count:
                self.frame_buffer.pop(0)
            merged = o3d.geometry.PointCloud()
            for buf_pcd in self.frame_buffer:
                merged += buf_pcd
            pcd_for_detect = merged
        else:
            pcd_for_detect = pcd_filtered

        # 5. DBSCAN 聚类检测
        targets = self.detector.detect(pcd_for_detect)

        # 6. 卡尔曼跟踪
        tracked = None
        if self.tracker:
            tracked = self.tracker.update(targets)

        dt = (time.time() - t0) * 1000

        # 7. 构造并发布 Locations 消息
        if self.radar_to_field is None:
            self.frame_id += 1
            return  # TF 尚未就绪，跳过发布

        locations_msg = Locations()

        if tracked is not None:
            # 使用跟踪结果（卡尔曼滤波位置）
            for tt in tracked:
                if tt.last_detection is None:
                    continue
                # 用 KF 位置 (x,y) + 检测的原始 Z → TF 变换到赛场坐标
                field_pt = self._lidar_to_field(tt.pos[0], tt.pos[1], tt.height)
                if field_pt is None:
                    continue
                field_x = max(0.0, min(float(field_pt[0]), FIELD_WIDTH))
                field_y = max(0.0, min(float(field_pt[1]), FIELD_HEIGHT))
                field_z = float(field_pt[2])  # 赛场坐标Z即为离地高度

                # 根据赛场位置粗分敌我
                if field_x < FIELD_WIDTH / 2:
                    air_label, air_id = "Blue", 106
                else:
                    air_label, air_id = "Red", 6

                loc = Location()
                loc.x = field_x
                loc.y = field_y
                loc.z = field_z
                loc.id = air_id
                loc.label = air_label
                locations_msg.locs.append(loc)
        else:
            # 无跟踪器时使用原始检测结果
            for target in targets:
                field_pt = self._lidar_to_field(
                    target.center[0], target.center[1], target.center[2]
                )
                if field_pt is None:
                    continue
                field_x = max(0.0, min(float(field_pt[0]), FIELD_WIDTH))
                field_y = max(0.0, min(float(field_pt[1]), FIELD_HEIGHT))
                field_z = float(field_pt[2])

                if field_x < FIELD_WIDTH / 2:
                    air_label, air_id = "Blue", 106
                else:
                    air_label, air_id = "Red", 6

                loc = Location()
                loc.x = field_x
                loc.y = field_y
                loc.z = field_z
                loc.id = air_id
                loc.label = air_label
                locations_msg.locs.append(loc)

        # 发布
        if len(locations_msg.locs) > 0:
            self.pub_location.publish(locations_msg)

        # 日志
        track_info = f" tracked={len(tracked)}" if tracked is not None else ""
        if len(targets) > 0:
            self.get_logger().info(
                f'Frame {self.frame_id}: {n_raw} pts → {n_filtered} air pts → '
                f'{len(targets)} targets ({dt:.1f}ms){track_info}'
            )
            for i, target in enumerate(targets):
                fp = self._lidar_to_field(
                    target.center[0], target.center[1], target.center[2]
                )
                if fp is not None:
                    self.get_logger().debug(
                        f'  Air Target {i}: field=({fp[0]:.2f}, {fp[1]:.2f}) '
                        f'h={fp[2]:.2f}m pts={target.num_points}'
                    )

        self.frame_id += 1


def main(args=None):
    rclpy.init(args=args)
    node = AirTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

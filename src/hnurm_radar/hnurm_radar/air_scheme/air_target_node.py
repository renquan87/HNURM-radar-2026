"""
air_target_node.py — 空中机器人检测 ROS2 节点

方案三：纯激光雷达空中机器人定位。
不调用相机，通过点云聚类完成空中机器人检测和定位，
将检测结果以 Locations 消息发布到 /location 话题，
复用已有的 ekf_node → judge_messager → display_panel 下游链路。

数据流：
  /livox/lidar → lidar_node → /lidar_pcds (PointCloud2)
                                    ↓
                            air_target_node (本节点)
                              ├── 背景预加载（可选，从PCD加载，跳过在线学习）
                              ├── 点云预处理（ROI + 体素降采样）
                              ├── 背景减除（只学地面区域，不学空中区域）
                              ├── 高度过滤（固定或自适应）
                              ├── 多帧缓冲（可选）
                              ├── DBSCAN 聚类（Open3D，含 Z 轴压缩）
                              ├── 卡尔曼跟踪（含确认窗口/loose_query/separate）
                              └── 坐标变换（雷达 → 赛场 via TF livox→map）
                                    ↓
                            /location (Locations) → ekf_node → display_panel

空中机器人 ID 约定（RoboMaster 2026 裁判系统协议）：
  strict_dual_uav = true  → 红方空中 id=6，蓝方空中 id=106（固定）
  strict_dual_uav = false → 每个跟踪目标赋予独立调试 ID（便于分析误检）

敌我区分：
  赛场 X 轴 [0, field_split_x) = 蓝方半区，[field_split_x, 28] = 红方半区。

订阅话题：
  - /lidar_pcds (PointCloud2) — 由 lidar_node 发布的累积点云

发布话题：
  - /location (Locations) — 空中机器人赛场坐标（供 ekf_node 滤波）
"""

import logging
import os
import struct
import time
import threading
from typing import List, Optional, Tuple

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
from .air_kalman_filter import AirKalmanFilter

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from std_msgs.msg import Header
from sensor_msgs.msg import PointField
try:
    from visualization_msgs.msg import Marker, MarkerArray
    _HAS_VIZ_MARKERS = True
except ImportError:
    _HAS_VIZ_MARKERS = False

logger = logging.getLogger(__name__)


class AirTargetNode(Node):
    """空中机器人检测 ROS2 节点"""

    def __init__(self):
        super().__init__('air_target_node')
        self.get_logger().info('Air Target Node 正在初始化...')

        # 加载配置
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        self.my_color = main_cfg.get("global", {}).get("my_color", "Red")
        self.air_cfg = load_air_target_config(main_cfg)
        self.strict_dual_uav = self.air_cfg.strict_dual_uav
        self.max_targets = int(self.air_cfg.max_targets)
        self.field_split_x = float(self.air_cfg.field_split_x)

        if not self.air_cfg.enabled:
            self.get_logger().warn('空中机器人检测已禁用 (air_target.enabled = false)')
            return

        # 空中机器人 ID（RoboMaster 2026 裁判系统协议）
        if self.my_color == "Red":
            self.ally_air_id, self.ally_label   = 6,   "Red"
            self.enemy_air_id, self.enemy_label = 106, "Blue"
        else:
            self.ally_air_id, self.ally_label   = 106, "Blue"
            self.enemy_air_id, self.enemy_label = 6,   "Red"

        # ---- 全局卡尔曼参数注册（必须在创建 Tracker 前） ----
        kp = self.air_cfg.tracking.kalman
        AirKalmanFilter.set_global_params(
            q_pos=kp.q_pos, q_vel=kp.q_vel, q_pv=kp.q_pv,
            r_pos=kp.r_pos, r_vel=kp.r_vel,
            decay_rate=kp.decay_rate,
            max_velocity=kp.max_velocity,
            cov_factor=kp.cov_factor,
            stop_p_time=kp.stop_p_time,
            init_p_times=kp.init_p_times,
        )

        # ---- 核心模块 ----
        self.processor = AirPointCloudProcessor(self.air_cfg.preprocessing)
        self.detector  = AirClusterDetector(
            self.air_cfg.clustering,
            self.air_cfg.target_filter,
        )

        # ---- TF 变换（livox → map，与 lidar_scheme/radar.py 方式一致） ----
        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.radar_to_field: Optional[np.ndarray] = None
        self.tf_timer = self.create_timer(1.0, self._tf_timer_callback)

        # ---- 跟踪器 ----
        self.tracker: Optional[AirTargetTracker] = None
        if self.air_cfg.tracking.enabled:
            tr = self.air_cfg.tracking
            self.tracker = AirTargetTracker(
                max_lost_frames=tr.max_lost_frames,
                match_distance_threshold=tr.match_distance,
                force_combine_dist=tr.force_combine_dist,
                cc_thres=tr.cc_thres,
                combine_limit=tr.combine_limit,
                separate_limit=tr.separate_limit,
                confirm_frames=tr.confirm_frames,
                kf_params=None,   # 已通过 set_global_params 注册
            )

        # ---- 背景减除 ----
        # 在线学习时跳过飞行区域 [z_min, z_max]，避免将悬停无人机学入背景
        # Z 轴方向：更负 = 更接近地面，更接近0 = 更高（更接近 LiDAR）
        # 例：地面 z≈-1.05，无人机 z∈[-0.97,-0.5]，LiDAR z=0
        self.bg_subtractor: Optional[AirBackgroundSubtractor] = None
        if self.air_cfg.background.enabled:
            flight_z_min = self.air_cfg.preprocessing.height_filter.z_min  # e.g., -1.5
            flight_z_max = self.air_cfg.preprocessing.height_filter.z_max  # e.g., -0.5
            self.bg_subtractor = AirBackgroundSubtractor(
                voxel_size=self.air_cfg.background.voxel_size,
                occupy_threshold=self.air_cfg.background.occupy_threshold,
                learning_frames=self.air_cfg.background.learning_frames,
                flight_z_min=flight_z_min,
                flight_z_max=flight_z_max,
            )

        # ---- 背景预加载（从预建 PCD 地图加载，可跳过在线学习） ----
        self._bg_pcd_path = ""
        self._bg_preloaded = False
        self._bg_preload_tf: Optional[np.ndarray] = None  # BG 预加载时使用的 TF
        self._bg_drift_threshold = 0.10  # TF 漂移超过此值(m)时自动重载 BG
        if self.bg_subtractor is not None and self.air_cfg.background.bg_pcd_file:
            from ..shared.paths import PROJECT_ROOT
            pcd_path = self.air_cfg.background.bg_pcd_file
            if not os.path.isabs(pcd_path):
                pcd_path = os.path.join(PROJECT_ROOT, pcd_path)
            if os.path.isfile(pcd_path):
                self._bg_pcd_path = pcd_path
                self.get_logger().info(f'背景预加载已配置: {pcd_path}，等待 TF 就绪后加载')
            else:
                self.get_logger().warn(f'bg_pcd_file 不存在: {pcd_path}，回退到在线学习')

        # ---- 松弛查询参数（传入 tracker.update） ----
        self._lq_params = None
        if self.air_cfg.loose_query.enabled:
            lq = self.air_cfg.loose_query
            self._lq_params = {
                "expand_xy":         lq.expand_xy,
                "expand_z":          lq.expand_z,
                "eps_scale":         lq.eps_scale,
                "min_samples_scale": lq.min_samples_scale,
            }

        # ---- 多帧缓冲 ----
        self.frame_buffer:   List[o3d.geometry.PointCloud] = []
        self.buffer_enabled: bool = self.air_cfg.buffer.enabled
        self.buffer_count:   int  = self.air_cfg.buffer.frame_count

        # ---- 最新点云（线程安全） ----
        self._latest_points: Optional[np.ndarray] = None
        self._points_lock = threading.Lock()

        # ---- QoS ----
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
        qos_debug = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # ---- 订阅 / 发布 ----
        self.sub_pcds = self.create_subscription(
            PointCloud2, "lidar_pcds", self._pcds_callback, qos_lidar
        )
        self.pub_location = self.create_publisher(
            Locations, "location", qos_pub
        )

        # ---- Debug 可视化发布（供 RViz 查看检测到的空中点云聚类） ----
        self.pub_debug_air = self.create_publisher(
            PointCloud2, "air_debug/air_points", qos_debug
        )
        self.pub_debug_clusters = self.create_publisher(
            PointCloud2, "air_debug/cluster_points", qos_debug
        )
        if _HAS_VIZ_MARKERS:
            self.pub_debug_markers = self.create_publisher(
                MarkerArray, "air_debug/cluster_markers", qos_debug
            )

        # ---- 定时处理 ----
        interval = 1.0 / max(self.air_cfg.publish_rate, 1.0)
        self.timer = self.create_timer(interval, self._process_callback)

        self.frame_id = 0
        bg_info = 'OFF'
        if self.bg_subtractor:
            if self._bg_pcd_path:
                bg_info = f'PRELOAD ({os.path.basename(self._bg_pcd_path)})'
            else:
                bg_info = (f'ON (flight_zone=[{flight_z_min}, {flight_z_max}], '
                           f'frames={self.air_cfg.background.learning_frames})')
        self.get_logger().info(
            f'Air Target Node 初始化完成: '
            f'我方={self.my_color}, '
            f'发布频率={self.air_cfg.publish_rate}Hz, '
            f'双机严格模式={"ON" if self.strict_dual_uav else "OFF"}, '
            f'跟踪={"ON" if self.tracker else "OFF"}, '
            f'背景减除={bg_info}, '
            f'confirm_frames={self.air_cfg.tracking.confirm_frames}'
        )

    # ------------------------------------------------------------------ #
    # 点云接收
    # ------------------------------------------------------------------ #
    def _pcds_callback(self, msg: PointCloud2):
        """接收 lidar_node 发布的累积点云"""
        try:
            pts_gen = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            arr = np.array(list(pts_gen),
                           dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)])
            points = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float64)
            with self._points_lock:
                self._latest_points = points
        except Exception as e:
            self.get_logger().warn(f'点云接收异常: {e}')

    # ------------------------------------------------------------------ #
    # TF 变换
    # ------------------------------------------------------------------ #
    def _tf_timer_callback(self):
        """定时查询 livox → map 的 TF 变换，并检测 TF 漂移触发 BG 重载"""
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox',
                time=rclpy.time.Time(),
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            self.radar_to_field = self._tf_to_matrix(t, r)

            # TF 漂移检测：如果当前 TF 与 BG 预加载时的 TF 平移差 > 阈值，
            # 自动触发 BG 重新预加载（使用最新 TF 重新对齐 PCD 到 livox 帧）
            if (self._bg_preload_tf is not None
                    and self._bg_preloaded
                    and self._bg_pcd_path):
                old_t = self._bg_preload_tf[:3, 3]
                new_t = self.radar_to_field[:3, 3]
                drift = float(np.linalg.norm(new_t - old_t))
                if drift > self._bg_drift_threshold:
                    self.get_logger().warn(
                        f'⚠ TF 漂移 {drift:.3f}m（阈值{self._bg_drift_threshold}m），'
                        f'触发 BG 重新预加载'
                    )
                    self._bg_preloaded = False  # 下次 _process_callback 时重载
        except TransformException as ex:
            if self.radar_to_field is None:
                self.get_logger().warn(f'等待 TF (livox→map): {ex}')

    @staticmethod
    def _quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
        x, y, z, w = q
        return np.array([
            [1 - 2*(y*y + z*z),   2*(x*y - z*w),     2*(x*z + y*w)  ],
            [2*(x*y + z*w),       1 - 2*(x*x + z*z), 2*(y*z - x*w)  ],
            [2*(x*z - y*w),       2*(y*z + x*w),     1 - 2*(x*x + y*y)],
        ])

    def _tf_to_matrix(self, translation, rotation) -> np.ndarray:
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        R = self._quaternion_to_rotation_matrix(q)
        M = np.eye(4)
        M[:3, :3] = R
        M[:3, 3]  = [translation.x, translation.y, translation.z]
        return M

    def _lidar_to_field(self, x: float, y: float, z: float) -> Optional[np.ndarray]:
        """雷达坐标 → 赛场坐标（通过 livox→map TF 矩阵）"""
        if self.radar_to_field is None:
            return None
        pt = np.array([x, y, z, 1.0])
        return (self.radar_to_field @ pt)[:3]

    # ------------------------------------------------------------------ #
    # 敌我区分
    # ------------------------------------------------------------------ #
    def _classify_side(self, field_x: float) -> Tuple[str, int]:
        """根据赛场 X 坐标（MAP 坐标系）判断半区：X < split → 蓝方，X ≥ split → 红方"""
        if field_x < self.field_split_x:
            return "Blue", 106
        return "Red", 6

    # ------------------------------------------------------------------ #
    # ID 编码
    # ------------------------------------------------------------------ #
    def _compose_air_id(self, label: str, track_id: int) -> int:
        """计算发布 ID。

        strict_dual_uav=True  → 固定 id 6/106
        strict_dual_uav=False → 独立调试 ID（每个独立跟踪目标有唯一编号）
        """
        if self.strict_dual_uav:
            return 6 if label == "Red" else 106
        # 非严格模式：给每个目标独立 ID，以便小地图同时显示多个目标
        base = 600 if label == "Red" else 1600
        return base + (track_id % 400)

    # ------------------------------------------------------------------ #
    # 目标筛选（严格模式 / 宽松模式）
    # ------------------------------------------------------------------ #
    def _select_publish_targets(self, raw_targets: List[dict]) -> List[dict]:
        if not raw_targets:
            return []

        raw_targets.sort(key=lambda x: x["confidence"], reverse=True)

        if not self.strict_dual_uav:
            # 宽松模式：不限制数量（或按 max_targets 限制）
            if self.max_targets <= 0:
                return raw_targets
            return raw_targets[: self.max_targets]

        # 严格模式：最多 2 架（红1蓝1），按置信度择优
        best: dict = {}
        for t in raw_targets:
            lbl = t["label"]
            if lbl not in best:
                best[lbl] = t
            if len(best) == 2:
                break
        return list(best.values())

    # ------------------------------------------------------------------ #
    # 主处理接口（供内部定时器调用，也可外部集成测试调用）
    # ------------------------------------------------------------------ #
    def process_frame_pcd(
        self, pcd: o3d.geometry.PointCloud
    ) -> Tuple[List[Location], dict]:
        """处理一帧 Open3D 点云，返回 (待发布 Location 列表, 诊断统计字典)。

        接受 Open3D 点云对象，不依赖文件路径（P0：process_frame_pcd 接口）。
        """
        t0 = time.time()
        n_raw = len(pcd.points)

        # 1) ROI 裁剪 + 体素降采样
        pcd_roi  = self.processor.crop_roi(pcd)
        n_roi = len(pcd_roi.points)
        pcd_down = self.processor.downsample(pcd_roi)
        n_down = len(pcd_down.points)

        # 2) 背景减除（对所有点做背景检查，含 PCD 预建背景）
        if self.bg_subtractor is not None:
            self.bg_subtractor.learn(pcd_down)
            pcd_fg = self.bg_subtractor.filter(pcd_down)
        else:
            pcd_fg = pcd_down
        n_fg = len(pcd_fg.points)

        # 3) 高度过滤（固定 or 自适应）
        tracked_heights: Optional[List[float]] = None
        if self.tracker is not None:
            tracked_heights = [t.height for t in self.tracker.get_all_targets()]
        pcd_filtered = self.processor.filter_height(pcd_fg, tracked_heights)
        n_filtered = len(pcd_filtered.points)

        # 4) 多帧缓冲
        if self.buffer_enabled:
            self.frame_buffer.append(pcd_filtered)
            if len(self.frame_buffer) > self.buffer_count:
                self.frame_buffer.pop(0)
            pcd_for_detect = o3d.geometry.PointCloud()
            for buf_pcd in self.frame_buffer:
                pcd_for_detect += buf_pcd
        else:
            pcd_for_detect = pcd_filtered

        # 5) DBSCAN 聚类 + 筛选
        # 先聚类（不筛选），记录诊断信息
        all_clusters = self.detector.cluster(pcd_for_detect)
        targets = self.detector.filter_targets(all_clusters)

        # ---- 管线诊断日志（每 10 帧打印一次） ----
        self._diag_counter = getattr(self, '_diag_counter', 0) + 1
        if self._diag_counter % 10 == 1:
            cluster_info = ', '.join(
                f'n={len(c)}' for c in all_clusters[:8]
            ) if all_clusters else 'none'
            target_info = ', '.join(
                f'n={t.num_points}/conf={t.confidence:.2f}' for t in targets[:5]
            ) if targets else 'none'
            bg_voxels = len(self.bg_subtractor.voxel_counts) if self.bg_subtractor else 0
            self.get_logger().info(
                f'[管线诊断] raw={n_raw} → roi={n_roi} → down={n_down} '
                f'→ bg_filter={n_fg}(bg_voxels={bg_voxels}) → height={n_filtered} '
                f'| clusters({len(all_clusters)})=[{cluster_info}] '
                f'| targets({len(targets)})=[{target_info}]'
            )
            # 被拒绝的簇（诊断核心！可以看到无人机为什么没通过）
            rejected = getattr(self.detector, '_last_rejected', [])
            if rejected:
                for r in rejected[:10]:
                    self.get_logger().info(f'  ❌ {r}')

        # 6) 更新跟踪器（内置 separate / loose_query / force_combine）
        tracked = None
        if self.tracker is not None:
            tracked = self.tracker.update(
                detections=targets,
                pcd=pcd_for_detect,
                detector=self.detector,
                loose_query_params=self._lq_params if self.air_cfg.loose_query.enabled else None,
            )

        # 7) 坐标变换 + 敌我区分
        raw_publish: List[dict] = []
        if self.radar_to_field is not None:
            source_targets = tracked if tracked is not None else []

            if source_targets:
                for t in source_targets:
                    field_pt = self._lidar_to_field(t.pos[0], t.pos[1], t.height)
                    if field_pt is None:
                        continue
                    fx = float(np.clip(field_pt[0], 0.0, FIELD_WIDTH))
                    fy = float(np.clip(field_pt[1], 0.0, FIELD_HEIGHT))
                    fz = float(field_pt[2])
                    label, _ = self._classify_side(fx)
                    raw_publish.append({
                        "track_id":  t.id,
                        "label":     label,
                        "x":         fx,
                        "y":         fy,
                        "z":         fz,
                        "confidence": float(getattr(t, "confidence", 0.0)),
                    })
            elif tracked is None:
                # 无跟踪时直接发布检测目标（调试模式）
                for i, tgt in enumerate(targets):
                    field_pt = self._lidar_to_field(
                        tgt.center[0], tgt.center[1], tgt.center[2]
                    )
                    if field_pt is None:
                        continue
                    fx = float(np.clip(field_pt[0], 0.0, FIELD_WIDTH))
                    fy = float(np.clip(field_pt[1], 0.0, FIELD_HEIGHT))
                    fz = float(field_pt[2])
                    label, _ = self._classify_side(fx)
                    raw_publish.append({
                        "track_id":  i,
                        "label":     label,
                        "x":         fx,
                        "y":         fy,
                        "z":         fz,
                        "confidence": float(getattr(tgt, "confidence", 0.0)),
                    })

        selected = self._select_publish_targets(raw_publish)

        out_locations: List[Location] = []
        for t in selected:
            loc       = Location()
            loc.id    = self._compose_air_id(t["label"], t["track_id"])
            loc.label = t["label"]
            loc.x     = float(t["x"])
            loc.y     = float(t["y"])
            loc.z     = float(t["z"])
            out_locations.append(loc)

        stats = {
            "raw_points": n_raw,
            "air_points": len(pcd_filtered.points),
            "detected":   len(targets),
            "published":  len(out_locations),
            "latency_ms": (time.time() - t0) * 1000.0,
            "tracked":    0 if tracked is None else len(tracked),
            "bg_voxels":  (self.bg_subtractor.background_voxel_count
                           if self.bg_subtractor else 0),
            # Debug 中间数据（供 _publish_debug 使用）
            "_pcd_filtered": pcd_filtered,
            "_targets": targets,
            "_all_clusters": all_clusters,
        }
        return out_locations, stats

    # ------------------------------------------------------------------ #
    # Debug 点云发布（供 RViz 实时查看检测到的空中目标聚类）
    # ------------------------------------------------------------------ #
    _CLUSTER_COLORS = [
        (1.0, 0.2, 0.2), (0.2, 0.6, 1.0), (0.2, 0.9, 0.2),
        (1.0, 0.8, 0.0), (0.8, 0.2, 1.0), (0.0, 0.9, 0.9),
    ]

    def _publish_debug(self, stats: dict):
        """发布调试点云和标记到 RViz

        话题：
          air_debug/air_points      — 高度过滤后的空中区域点云 (PointCloud2)
          air_debug/cluster_points  — 聚类点云, intensity=聚类ID (PointCloud2)
          air_debug/cluster_markers — 包围盒 + 文本标注 (MarkerArray)
        """
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'livox'

        fields_xyz = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        # 1) 空中区域过滤后点云
        pcd_filtered = stats.get('_pcd_filtered')
        if pcd_filtered is not None and len(pcd_filtered.points) > 0:
            pts = np.ascontiguousarray(
                np.asarray(pcd_filtered.points, dtype=np.float32)
            )
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = len(pts)
            msg.fields = fields_xyz
            msg.is_bigendian = False
            msg.point_step = 12
            msg.row_step = 12 * len(pts)
            msg.data = pts.tobytes()
            msg.is_dense = True
            self.pub_debug_air.publish(msg)

        # 2) 聚类点云（所有聚类可视化：通过筛选=彩色, 拒绝=暗红色）
        #    RViz Color Transformer 选 "RGB8"
        targets = stats.get('_targets', [])
        all_clusters = stats.get('_all_clusters', [])
        accepted_ids = {id(t.points) for t in targets}

        if all_clusters:
            # 预计算通过筛选的聚类颜色（彩色循环）
            rgb_floats_ok = []
            for c in self._CLUSTER_COLORS:
                r, g, b = int(c[0]*255), int(c[1]*255), int(c[2]*255)
                rgb_int = (r << 16) | (g << 8) | b
                rgb_floats_ok.append(
                    struct.unpack('f', struct.pack('I', rgb_int))[0]
                )
            # 被拒绝的聚类统一用暗红色
            _rej_rgb_int = (180 << 16) | (60 << 8) | 60
            rej_rgb_float = struct.unpack('f', struct.pack('I', _rej_rgb_int))[0]

            parts = []
            acc_color_idx = 0
            for cpts in all_clusters:
                pts_f32 = np.asarray(cpts, dtype=np.float32)
                if id(cpts) in accepted_ids:
                    rgb_val = rgb_floats_ok[acc_color_idx % len(rgb_floats_ok)]
                    acc_color_idx += 1
                else:
                    rgb_val = rej_rgb_float
                rgbs = np.full((len(pts_f32), 1), rgb_val, dtype=np.float32)
                parts.append(np.hstack([pts_f32, rgbs]))
            all_pts = np.ascontiguousarray(np.vstack(parts))
            msg = PointCloud2()
            msg.header = header
            msg.height = 1
            msg.width = len(all_pts)
            msg.fields = fields_xyz + [
                PointField(name='rgb', offset=12,
                           datatype=PointField.FLOAT32, count=1),
            ]
            msg.is_bigendian = False
            msg.point_step = 16
            msg.row_step = 16 * len(all_pts)
            msg.data = all_pts.tobytes()
            msg.is_dense = True
            self.pub_debug_clusters.publish(msg)

        # 3) 包围盒 + 文本标注 (MarkerArray) — 显示所有聚类
        #    通过筛选 = 绿色盒子 + ✓ 标签, 被拒绝 = 红色盒子 + ✗ 标签
        if _HAS_VIZ_MARKERS and hasattr(self, 'pub_debug_markers'):
            target_map = {id(t.points): t for t in targets}
            ma = MarkerArray()
            clr = Marker()
            clr.header = header
            clr.ns = 'air_clusters'
            clr.action = Marker.DELETEALL
            ma.markers.append(clr)
            for i, cpts in enumerate(all_clusters):
                is_ok = id(cpts) in accepted_ids
                t = target_map.get(id(cpts))
                center   = t.center   if t else cpts.mean(axis=0)
                bbox_min = t.bbox_min if t else cpts.min(axis=0)
                bbox_max = t.bbox_max if t else cpts.max(axis=0)
                n_pts    = t.num_points if t else len(cpts)
                conf     = t.confidence if t else 0.0

                # 包围盒
                m = Marker()
                m.header = header
                m.ns = 'air_bbox'
                m.id = i
                m.type = Marker.CUBE
                m.action = Marker.ADD
                m.pose.position.x = float((bbox_min[0] + bbox_max[0]) / 2)
                m.pose.position.y = float((bbox_min[1] + bbox_max[1]) / 2)
                m.pose.position.z = float((bbox_min[2] + bbox_max[2]) / 2)
                m.pose.orientation.w = 1.0
                m.scale.x = float(max(bbox_max[0] - bbox_min[0], 0.05))
                m.scale.y = float(max(bbox_max[1] - bbox_min[1], 0.05))
                m.scale.z = float(max(bbox_max[2] - bbox_min[2], 0.05))
                if is_ok:
                    m.color.r, m.color.g, m.color.b = 0.2, 0.9, 0.2   # 绿色
                else:
                    m.color.r, m.color.g, m.color.b = 0.9, 0.2, 0.2   # 红色
                m.color.a = 0.35
                m.lifetime.sec = 1
                ma.markers.append(m)

                # 文本标注
                txt = Marker()
                txt.header = header
                txt.ns = 'air_labels'
                txt.id = i
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = float(center[0])
                txt.pose.position.y = float(center[1])
                txt.pose.position.z = float(center[2]) - 0.3
                txt.scale.z = 0.15
                txt.color.r = txt.color.g = txt.color.b = 1.0
                txt.color.a = 1.0
                if is_ok:
                    txt.text = f'#{i} n={n_pts} conf={conf:.2f} \u2713'
                else:
                    sz = bbox_max - bbox_min
                    max_dim = float(sz.max())
                    txt.text = f'#{i} n={n_pts} sz={max_dim:.1f} \u2717'
                txt.lifetime.sec = 1
                ma.markers.append(txt)
            self.pub_debug_markers.publish(ma)

    # ------------------------------------------------------------------ #
    # 背景预加载（从预建 PCD 地图加载，跳过在线学习）
    # ------------------------------------------------------------------ #
    def _preload_background(self):
        """从预建 PCD 加载背景模型（TF 就绪后调用，TF 漂移时自动重载）。

        PCD 文件通常在世界坐标系（map 帧）中，而实时点云处理
        在 livox 帧中进行，因此需要将 PCD 逆变换到 livox 帧。

        当 registration_node 的 TF 发生显著漂移时（平移 > _bg_drift_threshold），
        _tf_timer_callback 会重置 _bg_preloaded 标志，触发此方法重新执行，
        使用最新 TF 重新对齐 BG 到当前 livox 帧。
        """
        try:
            pcd = o3d.io.read_point_cloud(self._bg_pcd_path)
            n_total = len(pcd.points)
            if n_total == 0:
                self.get_logger().warn(f'背景PCD为空: {self._bg_pcd_path}')
                return

            # 重载时先清空旧模型（避免新旧 BG 体素混合）
            self.bg_subtractor.reset()
            # 重置后恢复 is_ready（reset 会将其改为 learning_frames==0 时的默认值）
            self.bg_subtractor.is_ready = True

            # PCD 在 map（世界）坐标系，需要逆变换到 livox 帧
            inv_tf = np.linalg.inv(self.radar_to_field)
            pcd.transform(inv_tf)

            # ROI 裁剪 + 体素降采样（与实时处理流水线一致）
            pcd = self.processor.crop_roi(pcd)
            pcd = self.processor.downsample(pcd)

            # 加载为背景模型
            added = self.bg_subtractor.load_from_pcd(pcd)

            # 记录本次预加载使用的 TF（用于后续漂移检测）
            self._bg_preload_tf = self.radar_to_field.copy()

            self.get_logger().info(
                f'✅ 背景预加载完成: {os.path.basename(self._bg_pcd_path)} '
                f'({n_total} 点 → {len(pcd.points)} 点 ROI内, '
                f'{added} 个新背景体素, '
                f'总背景={self.bg_subtractor.background_voxel_count})'
            )
        except Exception as e:
            self.get_logger().error(f'背景预加载失败: {e}')

    # ------------------------------------------------------------------ #
    # 定时处理回调
    # ------------------------------------------------------------------ #
    def _process_callback(self):
        with self._points_lock:
            points = self._latest_points

        if points is None or len(points) == 0:
            return

        if self.radar_to_field is None:
            # TF 未就绪，但仍需推进背景学习（用下采样后的点云）
            self.frame_id += 1
            return

        # 背景预加载：TF 就绪后第一时间从 PCD 加载背景模型
        if self._bg_pcd_path and not self._bg_preloaded:
            self._preload_background()
            self._bg_preloaded = True

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        locations, stats = self.process_frame_pcd(pcd)

        if locations:
            msg      = Locations()
            msg.locs = locations
            self.pub_location.publish(msg)

        # 发布调试点云到 RViz（air_debug/* 话题）
        if hasattr(self, 'pub_debug_air'):
            self._publish_debug(stats)

        if stats["detected"] > 0 or stats["published"] > 0:
            self.get_logger().info(
                f"[air] frame={self.frame_id} raw={stats['raw_points']} "
                f"air_pts={stats['air_points']} detect={stats['detected']} "
                f"track={stats['tracked']} pub={stats['published']} "
                f"bg={stats['bg_voxels']} latency={stats['latency_ms']:.1f}ms"
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

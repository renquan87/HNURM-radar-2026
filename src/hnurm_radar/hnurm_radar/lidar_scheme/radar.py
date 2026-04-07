"""
radar.py — 激光雷达方案的融合定位节点（方案二）

功能：
  订阅 detector_node 发布的 2D 检测结果（detect_result）和 lidar_node 发布的
  背景减除点云（target_pointcloud），通过先聚类后投影+匈牙利匹配获取目标的三维位置，
  再通过 TF 变换将坐标从激光雷达坐标系转换到赛场坐标系，最终发布到 /location 话题。

数据流：
  detect_result (Robots)  ──┐
                            ├→ radar_callback() → 点云聚类 → 匹配 → 坐标变换 → /location (Locations)
  target_pointcloud (PointCloud2) ─┘
  TF (livox → map)        ──→ on_timer() 定时查询

配置文件：
  - configs/main_config.yaml      — 全局颜色、调试开关
  - configs/converter_config.yaml — 外参 R/T、内参 K、畸变系数
  - configs/detector_config.yaml  — 类别标签列表、卡尔曼参数等
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import time
from ..Car.Car import *
from sensor_msgs.msg import Image
import open3d as o3d
from ..Lidar.Converter import Converter , ROISelector
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from collections import deque
from ruamel.yaml import YAML
import os
from ..shared.paths import MAIN_CONFIG_PATH, CONVERTER_CONFIG_PATH, DETECTOR_CONFIG_PATH
from detect_result.msg import DetectResult
from detect_result.msg import Robots
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from detect_result.msg import Location, Locations

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import yaml
from std_msgs.msg import Header

from sklearn.cluster import DBSCAN, KMeans
from scipy.optimize import linear_sum_assignment
from typing import List, Tuple, Dict, Optional

# 导入新的固定槽位追踪器
from .tracker_enhanced import FixedSlotTracker

# ========== 移除旧的追踪器类（EKFPredictor, TrackState, Track, MultiTargetTracker）==========
# 它们已被 tracker_enhanced.py 中的实现替代，此处不再保留。

# ---------------------- 主节点 -----------------------------
class Radar(Node):

    def __init__(self):
        super().__init__('Radar')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos__lidar_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        converter_config_path = CONVERTER_CONFIG_PATH
        detector_cfg = YAML().load(open(DETECTOR_CONFIG_PATH, encoding='utf-8', mode='r'))
        
        # 全局变量
        self.global_my_color = main_cfg['global']['my_color']
        is_debug = main_cfg['global']['is_debug']
        self.carList = CarList(main_cfg)
        self.carList_results = []
        self.all_detections = []
        self.last_all_detections = []
        self.frame_id = 1
        self.counter = 0
        
        today = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        self.converter = Converter(self.global_my_color, converter_config_path)
        
        # 加载转换器配置文件（外参）
        with open(converter_config_path, 'r', encoding='utf-8') as file:
            data_loader = yaml.safe_load(file)
        self.R = np.array(data_loader['calib']['extrinsic']['R']['data']).reshape(
            (data_loader['calib']['extrinsic']['R']['rows'], data_loader['calib']['extrinsic']['R']['cols']))
        self.T = np.array(data_loader['calib']['extrinsic']['T']['data']).reshape(
            (data_loader['calib']['extrinsic']['T']['rows'], data_loader['calib']['extrinsic']['T']['cols']))
        self.extrinsic_matrix = np.hstack((self.R, self.T))
        self.extrinsic_matrix = np.vstack((self.extrinsic_matrix, [0, 0, 0, 1]))
        self.extrinsic_matrix_inv = np.linalg.inv(self.extrinsic_matrix)
        
        self.start_time = time.time()
        self.fps_queue = deque(maxlen=10)
        self.lidar_points = None
        
        # 订阅和发布
        self.sub_detect = self.create_subscription(Robots, "detect_result", self.radar_callback, qos_profile)
        self.get_logger().info('Radar subscriber has been started at {}.'.format(today))
        self.sub_pcds = self.create_subscription(PointCloud2, "target_pointcloud", self.pcd_callback, qos__lidar_profile)
        self.pub_location = self.create_publisher(Locations, "location", qos_profile)
        self.last_frameid = -1
        self.pub_nognd = self.create_publisher(PointCloud2, "pcd_removed", qos__lidar_profile)
        
        # TF 监听
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.radar_to_field = np.ones((4, 4))
        self.radar_to_field_inv = np.ones((4, 4))

        # ========== 初始化新的固定槽位追踪器 ==========
        # 读取卡尔曼滤波参数（从 detector_config.yaml 的 filter 段）
        filter_cfg = detector_cfg.get('filter', {})
        kf_process_noise = filter_cfg.get('process_noise', 0.005)
        kf_measurement_noise = filter_cfg.get('measurement_noise', 0.15)
        kf_jump_threshold = filter_cfg.get('jump_threshold', 1.0)
        kf_max_velocity = filter_cfg.get('max_velocity', 5.0)
        
        # 读取类别标签列表
        self.labels = detector_cfg['params']['labels']  # ["B1","B2",...,"R7"]
        
        # 根据己方颜色确定敌方 car_id 列表（固定槽位）
        if self.global_my_color == "Red":
            enemy_car_ids = [101, 102, 103, 104, 105, 106, 107]  # 蓝方
        else:
            enemy_car_ids = [1, 2, 3, 4, 5, 6, 7]                # 红方
        
        self.tracker = FixedSlotTracker(
            enemy_car_ids=enemy_car_ids,
            class_num=len(self.labels),
            max_distance=0.5,
            w_pos=0.7,
            w_app=0.3,
            process_noise=kf_process_noise,
            measurement_noise=kf_measurement_noise,
            jump_threshold=kf_jump_threshold,
            max_velocity=kf_max_velocity
        )
        
        # 保留旧的参数（用于簇匹配）
        self.MATCH_THRESHOLD = 0.8
        self.MIN_BOX_DIST_PX = 100
        self.MIN_CLUSTER_POINTS_FOR_SPLIT = 20

    # ---------- TF 相关方法（保持不变）----------
    def on_timer(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox',
                time=rclpy.time.Time()
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            transform_matrix = self.tf_to_matrix(translation, rotation)
            self.radar_to_field = transform_matrix
            self.radar_to_field_inv = np.linalg.inv(self.radar_to_field)
        except TransformException as ex:
            self.radar_to_field = np.ones((4, 4))
            self.get_logger().error(f"获取 TF 失败: {ex}")    
    
    def tf_to_matrix(self, translation, rotation):
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        R = self.quaternion_to_rotation_matrix(q)
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return transform_matrix
    
    def quaternion_to_rotation_matrix(self, q):
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])
    
    def camera_to_lidar(self, pc):
        pc = np.hstack((pc, np.ones((pc.shape[0], 1))))
        ret = np.dot(pc, self.extrinsic_matrix)
        ret = ret[:, :3]
        return ret
    
    # ---------- 点云回调（保持不变）----------
    def pcd_callback(self, msg):
        points = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if points.size == 0:
            self.lidar_points = None
            return
        points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float64)
        self.lidar_points = np.ascontiguousarray(points)
    
    # ---------- DBSCAN 聚类（保持不变）----------
    def cluster_points_dbscan(self, points: np.ndarray, eps: float = 0.25, min_samples: int = 3) -> List[np.ndarray]:
        if points.shape[0] == 0:
            return []
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
        labels = clustering.labels_
        clusters = []
        for label in set(labels):
            if label == -1:
                continue
            cluster_points = points[labels == label]
            clusters.append(cluster_points)
        return clusters

    def compute_cluster_features(self, cluster_pts):
        if cluster_pts.shape[0] == 0:
            return None
        center_lidar = np.mean(cluster_pts, axis=0)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cluster_pts)
        self.converter.lidar_to_camera(pcd)
        camera_points = np.asarray(pcd.points)
        uvz = self.converter.camera_to_image(camera_points)
        if uvz.size == 0:
            return None
        pixels = uvz[:, :2]
        center_camera = np.mean(camera_points, axis=0)
        center_pixel = np.mean(pixels, axis=0)
        return {
            'center_lidar': center_lidar,
            'center_camera': center_camera,
            'center_pixel': center_pixel,
            'pixels': pixels,
            'raw_points': cluster_pts
        }

    def split_cluster_with_kmeans(self, cluster_pts, matched_boxes):
        K = len(matched_boxes)
        if K < 2 or cluster_pts.shape[0] < self.MIN_CLUSTER_POINTS_FOR_SPLIT:
            return []
        kmeans = KMeans(n_clusters=K, init='k-means++', n_init=10, max_iter=300, random_state=0)
        labels = kmeans.fit_predict(cluster_pts)
        sub_features = []
        for k in range(K):
            mask = (labels == k)
            if np.sum(mask) < 3:
                continue
            sub_pts = cluster_pts[mask]
            feat = self.compute_cluster_features(sub_pts)
            if feat is None:
                continue
            feat['matched_box'] = matched_boxes[k]
            sub_features.append(feat)
        return sub_features
    
    # ---------- 辅助发布（保持不变）----------
    def _publish_through_carlist(self, null_robot_locations):
        self.carList.update_car_info(self.carList_results)
        all_infos = self.carList.get_all_info()
        allLocation = Locations()
        for all_info in all_infos:
            track_id, car_id, center_xy, camera_xyz, field_xyz, color, is_valid = all_info
            if color != self.global_my_color and track_id != -1:
                loc = Location()
                loc.x = float(field_xyz[0])
                loc.y = float(field_xyz[1])
                loc.z = float(field_xyz[2])
                loc.id = car_id
                loc.label = color
                allLocation.locs.append(loc)
        for null_xyz in null_robot_locations:
            loc = Location()
            loc.x = float(null_xyz[0])
            loc.y = float(null_xyz[1])
            loc.z = float(null_xyz[2])
            loc.id = 0
            loc.label = 'NULL'
            allLocation.locs.append(loc)
        self.pub_location.publish(allLocation)
    
    # ---------- 核心雷达回调（已适配新追踪器）----------
    def radar_callback(self, msg):
        detect_results = msg.detect_results
        if self.lidar_points is None:
            return

        # 1. 点云聚类
        clusters = self.cluster_points_dbscan(self.lidar_points, eps=0.15, min_samples=9)
        if not clusters:
            # 无点云簇，直接更新追踪器（观测为空）
            tracked_results = self.tracker.update([], time.time())
            self.carList_results.clear()
            null_robot_locations = []
            for car_id, pos in tracked_results:
                if car_id > 0:
                    self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
                elif car_id == 0:
                    null_robot_locations.append(pos)
            self._publish_through_carlist(null_robot_locations)
            return

        # 2. 提取簇特征
        cluster_features = []
        cluster_raw_points = []
        for cluster_pts in clusters:
            feat = self.compute_cluster_features(cluster_pts)
            if feat is not None:
                cluster_features.append(feat)
                cluster_raw_points.append(cluster_pts)
        if not cluster_features:
            return

        # 3. 匈牙利匹配（与检测框）
        n_clusters = len(cluster_features)
        n_boxes = len(detect_results)
        if n_boxes == 0:
            # 无检测框，所有簇作为点云观测（from_visual=False）
            observations = []
            for feat in cluster_features:
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False, -1, 0.0))   # (car_id, pos, from_visual, class_label, conf)
            tracked_results = self.tracker.update(observations, time.time())
            self.carList_results.clear()
            null_robot_locations = []
            for car_id, pos in tracked_results:
                if car_id > 0:
                    self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
                elif car_id == 0:
                    null_robot_locations.append(pos)
            self._publish_through_carlist(null_robot_locations)
            return

        # 构建代价矩阵（基于投影占比和像素距离）
        INF_COST = 1e9
        cost_matrix = np.full((n_clusters, n_boxes), INF_COST, dtype=np.float32)
        for i, feat in enumerate(cluster_features):
            for j, det in enumerate(detect_results):
                xywh_box = det.xywh_box
                box_center = np.array([xywh_box[0], xywh_box[1]])
                w, h = xywh_box[2], xywh_box[3]
                x1 = xywh_box[0] - w/2
                y1 = xywh_box[1] - h/2
                x2 = xywh_box[0] + w/2
                y2 = xywh_box[1] + h/2
                pixels = feat['pixels']
                inside = (pixels[:,0] >= x1) & (pixels[:,0] <= x2) & \
                         (pixels[:,1] >= y1) & (pixels[:,1] <= y2)
                ratio = np.sum(inside) / pixels.shape[0] if pixels.shape[0] > 0 else 0
                dist = np.linalg.norm(feat['center_pixel'] - box_center)
                if ratio > 0.75 and dist < 200:
                    diag = np.sqrt(w**2 + h**2)
                    norm_dist = dist / diag
                    cost = (1 - ratio) + norm_dist
                    cost_matrix[i, j] = cost

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matches = []
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < INF_COST / 2:
                matches.append((r, c))

        matched_cluster_to_box = {}
        matched_box_to_cluster = {}
        for cluster_idx, det_idx in matches:
            matched_cluster_to_box[cluster_idx] = det_idx
            matched_box_to_cluster[det_idx] = cluster_idx

        extra_matches = {i: [] for i in matched_cluster_to_box.keys()}
        if n_boxes > 0:
            unmatched_boxes = [j for j in range(n_boxes) if j not in matched_box_to_cluster]
            for j in unmatched_boxes:
                best_cluster = None
                best_cost = INF_COST
                for i in matched_cluster_to_box.keys():
                    if cost_matrix[i, j] < self.MATCH_THRESHOLD:
                        existing_box_idx = matched_cluster_to_box[i]
                        existing_box = detect_results[existing_box_idx]
                        cur_box = detect_results[j]
                        dist_centers = np.linalg.norm(np.array(existing_box.xywh_box[:2]) - np.array(cur_box.xywh_box[:2]))
                        if dist_centers < self.MIN_BOX_DIST_PX:
                            continue
                        if cost_matrix[i, j] < best_cost:
                            best_cost = cost_matrix[i, j]
                            best_cluster = i
                if best_cluster is not None:
                    extra_matches[best_cluster].append(j)

        final_cluster_to_boxes = {}
        for i in matched_cluster_to_box.keys():
            boxes = [matched_cluster_to_box[i]] + extra_matches[i]
            final_cluster_to_boxes[i] = boxes

        # 构建观测列表（包含类别和置信度）
        observations = []   # (car_id, field_xyz, from_visual, class_label, confidence)
        all_matched_clusters = set(final_cluster_to_boxes.keys())

        # 匹配成功的簇（视觉关联）
        for cluster_idx, box_indices in final_cluster_to_boxes.items():
            if len(box_indices) == 1:
                det = detect_results[box_indices[0]]
                label = det.label
                if label == "NULL":
                    car_id = 0
                    class_label = -1
                    confidence = 0.0
                else:
                    car_id = self.carList.get_car_id(label)
                    # 颜色过滤：己方车辆不生成观测
                    if self.global_my_color == "Red" and car_id < 100 and car_id != 7:
                        continue
                    if self.global_my_color == "Blue" and car_id > 100 and car_id != 107:
                        continue
                    # 获取类别索引
                    try:
                        class_label = self.labels.index(label)
                    except ValueError:
                        class_label = -1
                    confidence = det.confidence   # 注意：DetectResult 中需要有 confidence 字段
                feat = cluster_features[cluster_idx]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((car_id, field_xyz, True, class_label, confidence))
            else:
                # 多匹配：拆分簇
                cluster_pts = cluster_raw_points[cluster_idx]
                matched_boxes = [detect_results[j] for j in box_indices]
                sub_features = self.split_cluster_with_kmeans(cluster_pts, matched_boxes)
                for sub_feat in sub_features:
                    det = sub_feat['matched_box']
                    label = det.label
                    if label == "NULL":
                        car_id = 0
                        class_label = -1
                        confidence = 0.0
                    else:
                        car_id = self.carList.get_car_id(label)
                        if self.global_my_color == "Red" and car_id < 100 and car_id != 7:
                            continue
                        if self.global_my_color == "Blue" and car_id > 100 and car_id != 107:
                            continue
                        try:
                            class_label = self.labels.index(label)
                        except ValueError:
                            class_label = -1
                        confidence = det.confidence
                    center_h = np.append(sub_feat['center_lidar'], 1.0)
                    field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                    observations.append((car_id, field_xyz, True, class_label, confidence))

        # 未匹配的簇（点云观测）
        for i in range(n_clusters):
            if i not in all_matched_clusters:
                feat = cluster_features[i]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False, -1, 0.0))

        # 更新追踪器
        current_time = time.time()
        tracked_results = self.tracker.update(observations, current_time)

        # 转换为 carList_results 和 null_robot_locations
        self.carList_results.clear()
        null_robot_locations = []
        for car_id, pos in tracked_results:
            if car_id > 0:
                self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
            elif car_id == 0:
                null_robot_locations.append(pos)

        # 发布
        self._publish_through_carlist(null_robot_locations)

    def __del__(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    radar = Radar()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

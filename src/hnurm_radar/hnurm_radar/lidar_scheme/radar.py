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
from std_msgs.msg import String, Bool
import cv2
from cv_bridge import CvBridge
import time
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from collections import deque
from ruamel.yaml import YAML
import os
import yaml
from sklearn.cluster import DBSCAN, KMeans
from scipy.optimize import linear_sum_assignment
from typing import List, Tuple, Dict, Optional

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from detect_result.msg import DetectResult, Robots, Location, Locations
from tf2_ros import TransformException, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

from ..Car.Car import CarList
from ..shared.paths import MAIN_CONFIG_PATH, CONVERTER_CONFIG_PATH, DETECTOR_CONFIG_PATH
from ..Lidar.Converter import Converter
from .tracker_enhanced import FixedSlotTracker

# 为 rosbag 模式导入压缩图像消息类型
from sensor_msgs.msg import CompressedImage


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

        self.sub_detect = self.create_subscription(Robots, "detect_result", self.radar_callback, qos_profile)
        self.get_logger().info('Radar subscriber has been started at {}.'.format(today))
        self.sub_pcds = self.create_subscription(PointCloud2, "target_pointcloud", self.pcd_callback, qos__lidar_profile)
        self.pub_location = self.create_publisher(Locations, "location", qos_profile)
        self.last_frameid = -1
        self.pub_nognd = self.create_publisher(PointCloud2, "pcd_removed", qos__lidar_profile)

        # rosbag 模式接口（仅占位）
        camera_mode = main_cfg.get('camera', {}).get('mode', 'hik')
        if camera_mode == 'rosbag':
            compressed_topic = main_cfg.get('camera', {}).get('compressed_image_topic', '/compressed_image')
            self.sub_compressed = self.create_subscription(
                CompressedImage,
                compressed_topic,
                self._compressed_image_callback,
                qos_profile
            )
            self.get_logger().info(f'[Rosbag 接口] 订阅压缩图像话题: {compressed_topic} (仅占位，不参与融合)')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.radar_to_field = np.ones((4, 4))
        self.radar_to_field_inv = np.ones((4, 4))

        filter_cfg = detector_cfg.get('filter', {})
        kf_process_noise = filter_cfg.get('process_noise', 0.005)
        kf_measurement_noise = filter_cfg.get('measurement_noise', 0.15)
        kf_jump_threshold = filter_cfg.get('jump_threshold', 1.0)
        kf_max_velocity = filter_cfg.get('max_velocity', 5.0)

        self.labels = detector_cfg['params']['labels']

        # 槽位包含敌方+己方所有 ID
        if self.global_my_color == "Red":
            enemy_ids = [101, 102, 103, 104, 105, 106, 107]
            friend_ids = [1, 2, 3, 4, 5, 6, 7]
        else:
            enemy_ids = [1, 2, 3, 4, 5, 6, 7]
            friend_ids = [101, 102, 103, 104, 105, 106, 107]
        all_track_ids = enemy_ids + friend_ids

        self.tracker = FixedSlotTracker(
            enemy_car_ids=all_track_ids,
            class_num=len(self.labels),
            max_distance=0.5,
            w_pos=0.7,
            w_app=0.3,
            process_noise=kf_process_noise,
            measurement_noise=kf_measurement_noise,
            jump_threshold=kf_jump_threshold,
            max_velocity=kf_max_velocity
        )

        self.MATCH_THRESHOLD = 0.6
        self.MIN_BOX_DIST_PX = 1000
        self.MIN_CLUSTER_POINTS_FOR_SPLIT = 20

        # ---------- 预筛选参数 ----------
        self.lat_base = 1.5
        self.lat_scale = 0.03
        self.lon_base = 2.0
        self.lon_scale = 0.1

    def _compressed_image_callback(self, msg):
        pass

    def on_timer(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox_frame',
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

    # ---------- 新增：单目3D预筛选 ----------
    def _filter_clusters_by_mono(self, mono_pos, cluster_centers_lidar):
        """
        根据单目3D坐标筛选候选簇索引
        mono_pos: (x, y, z) 或 None
        返回候选簇索引列表
        """
        if mono_pos is None:
            return None  # 表示不筛选
        depth = abs(mono_pos[1])
        lat_th = self.lat_base + self.lat_scale * depth
        lon_th = self.lon_base + self.lon_scale * depth
        candidates = []
        for i, center_lidar in enumerate(cluster_centers_lidar):
            center_h = np.append(center_lidar, 1.0)
            field_center = np.dot(self.radar_to_field, center_h)[:3]
            if abs(field_center[0] - mono_pos[0]) <= lat_th and abs(field_center[1] - mono_pos[1]) <= lon_th:
                candidates.append(i)
        return candidates

    # ---------- 核心回调（修改代价矩阵部分） ----------
    def radar_callback(self, msg):
        detect_results = msg.detect_results
        if self.lidar_points is None:
            return

        clusters = self.cluster_points_dbscan(self.lidar_points, eps=0.15, min_samples=7)
        if not clusters:
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

        cluster_features = []
        cluster_raw_points = []
        for cluster_pts in clusters:
            feat = self.compute_cluster_features(cluster_pts)
            if feat is not None:
                cluster_features.append(feat)
                cluster_raw_points.append(cluster_pts)
        if not cluster_features:
            return

        n_clusters = len(cluster_features)
        n_boxes = len(detect_results)
        if n_boxes == 0:
            observations = []
            for feat in cluster_features:
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False, -1, 0.0))
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

        # ========== 优化1&2&3：向量化代价矩阵 + 单目预筛选 + 仅用中心距离 ==========
        # 提取数据为 NumPy 数组
        cluster_centers_pixel = np.array([f['center_pixel'] for f in cluster_features])  # (N,2)
        cluster_centers_lidar = [f['center_lidar'] for f in cluster_features]

        box_centers = np.array([[d.xywh_box[0], d.xywh_box[1]] for d in detect_results])  # (M,2)
        box_sizes = np.array([[d.xywh_box[2], d.xywh_box[3]] for d in detect_results])    # (M,2)
        diag = np.sqrt(box_sizes[:,0]**2 + box_sizes[:,1]**2)  # (M,)

        # 单目坐标列表
        mono_positions = []
        for det in detect_results:
            if 0 < det.field_x < 28 and 0 < det.field_y < 15:
                mono_positions.append(np.array([det.field_x, det.field_y, det.field_z]))
            else:
                mono_positions.append(None)

        INF_COST = 1e9
        cost_matrix = np.full((n_clusters, n_boxes), INF_COST, dtype=np.float32)

        # 计算所有簇中心与所有框中心的像素距离 (N,M)
        diff = cluster_centers_pixel[:, np.newaxis, :] - box_centers[np.newaxis, :, :]
        dist_matrix = np.linalg.norm(diff, axis=2)   # (N,M)

        # 归一化距离代价
        norm_dist = dist_matrix / (diag[np.newaxis, :] + 1e-6)

        # 基础代价 = 归一化距离
        base_cost = norm_dist.copy()

        # 应用预筛选：对于每个检测框，若簇不在候选集中，代价置为 INF
        for j, mono_pos in enumerate(mono_positions):
            candidates = self._filter_clusters_by_mono(mono_pos, cluster_centers_lidar)
            if candidates is not None:
                mask = np.ones(n_clusters, dtype=bool)
                mask[candidates] = False
                base_cost[mask, j] = INF_COST

        # 像素距离超过200的直接置 INF
        base_cost[dist_matrix > 200] = INF_COST

        cost_matrix = base_cost.astype(np.float32)
        # ========== 优化部分结束 ==========

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

        observations = []
        all_matched_clusters = set(final_cluster_to_boxes.keys())

        # 以下观测生成部分保持不变
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
                    try:
                        class_label = self.labels.index(label)
                    except ValueError:
                        class_label = -1
                    confidence = det.confidence
                feat = cluster_features[cluster_idx]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((car_id, field_xyz, True, class_label, confidence))
            else:
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
                        try:
                            class_label = self.labels.index(label)
                        except ValueError:
                            class_label = -1
                        confidence = det.confidence
                    center_h = np.append(sub_feat['center_lidar'], 1.0)
                    field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                    observations.append((car_id, field_xyz, True, class_label, confidence))

        for i in range(n_clusters):
            if i not in all_matched_clusters:
                feat = cluster_features[i]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False, -1, 0.0))

        current_time = time.time()
        tracked_results = self.tracker.update(observations, current_time)

        self.carList_results.clear()
        null_robot_locations = []
        for car_id, pos in tracked_results:
            if car_id > 0:
                self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
            elif car_id == 0:
                null_robot_locations.append(pos)

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
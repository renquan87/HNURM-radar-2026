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
from ..shared.paths import MAIN_CONFIG_PATH, CONVERTER_CONFIG_PATH
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

# ---------------------- 扩展卡尔曼滤波器（仅用于预测） -----------------------------
class EKFPredictor:
    """
    简单的 EKF 预测器，状态为 [x, y, vx, vy]，仅用于预测位置，不进行观测更新。
    用于多目标追踪器的数据关联。
    """
    def __init__(self, dt=0.1):
        self.dt = dt
        # 状态转移矩阵
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], dtype=np.float32)
        # 过程噪声协方差（简易）
        self.Q = np.eye(4, dtype=np.float32) * 0.01
        # 状态向量 [x, y, vx, vy]
        self.x = np.zeros((4, 1), dtype=np.float32)
        # 协方差
        self.P = np.eye(4, dtype=np.float32) * 1.0

    def predict(self):
        """预测一步，返回预测位置 (x, y)"""
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return float(self.x[0, 0]), float(self.x[1, 0])

    def set_state(self, x, y, vx=0.0, vy=0.0):
        """设置状态（由观测直接赋值）"""
        self.x = np.array([[x], [y], [vx], [vy]], dtype=np.float32)

    def get_position(self):
        return float(self.x[0, 0]), float(self.x[1, 0])


# ---------------------- 多目标追踪器（基于 EKF 预测，内部自建 ID） -----------------------------
class TrackState:
    Tentative = 0
    Confirmed = 1
    Deleted = 2

class Track:
    def __init__(self, internal_id: int, car_id: int, position: np.ndarray, dt=0.1, from_visual=False):
        self.internal_id = internal_id
        self.car_id = car_id                      # 外部 ID（来自视觉或 0）
        self.state = TrackState.Tentative
        self.hits = 1
        self.no_loss = 0
        self.age = 0
        self.position = position.copy()
        self.predictor = EKFPredictor(dt=dt)
        self.predictor.set_state(position[0], position[1])
        self.has_ever_matched = from_visual       # 是否曾与检测框匹配过（即有 label）

    def predict(self):
        self.age += 1
        pred_x, pred_y = self.predictor.predict()
        return np.array([pred_x, pred_y, self.position[2]])

    def update(self, position: np.ndarray, car_id: int, from_visual: bool):
        """使用观测更新轨迹"""
        self.position = position.copy()
        self.predictor.set_state(position[0], position[1])
        if from_visual:
            # 视觉观测：更新 car_id（如果 car_id 为正且有效），并标记已匹配过
            if car_id != 0:
                self.car_id = car_id
            self.has_ever_matched = True
        else:
            # 点云观测（未匹配簇）：不改变 car_id，也不改变 has_ever_matched 状态
            pass
        self.hits += 1
        self.no_loss = 0
        if self.state == TrackState.Tentative and self.hits >= 3:
            self.state = TrackState.Confirmed

    def mark_missed(self):
        self.no_loss += 1
        if self.no_loss > 5:
            self.state = TrackState.Deleted

    def get_position(self) -> np.ndarray:
        return self.position

    def get_predicted_position(self) -> np.ndarray:
        pred_x, pred_y = self.predictor.get_position()
        return np.array([pred_x, pred_y, self.position[2]])


class MultiTargetTracker:
    def __init__(self, max_distance=0.5, max_age=5, min_hits=3, dt=0.1):
        self.tracks: Dict[int, Track] = {}   # internal_id -> Track
        self.next_id = 1
        self.max_distance = max_distance
        self.max_age = max_age
        self.min_hits = min_hits
        self.dt = dt

    def _get_new_id(self):
        new_id = self.next_id
        self.next_id += 1
        return new_id

    def predict(self):
        for track in self.tracks.values():
            track.predict()

    def update(self, detections: List[Tuple[int, np.ndarray, bool]]) -> List[Tuple[int, np.ndarray]]:
        """
        detections: (car_id, position, from_visual)
            - car_id: 0 表示点云观测（未匹配簇），正数表示视觉观测
            - from_visual: True 表示该观测来自检测框（即有 label），False 表示来自未匹配簇
        返回确认态轨迹的 (car_id, position)（仅返回 has_ever_matched == True 的轨迹）
        """
        self.predict()

        if len(detections) == 0:
            for track in self.tracks.values():
                track.mark_missed()
            self._remove_dead_tracks()
            return []

        # 构建代价矩阵（使用预测位置）
        track_ids = list(self.tracks.keys())
        n_tracks = len(track_ids)
        n_dets = len(detections)
        cost_matrix = np.zeros((n_tracks, n_dets))
        for i, tid in enumerate(track_ids):
            pred_pos = self.tracks[tid].get_predicted_position()
            for j, (_, det_pos, _) in enumerate(detections):
                cost_matrix[i, j] = np.linalg.norm(pred_pos - det_pos)

        # 匈牙利匹配
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matched_tracks = set()
        matched_dets = set()
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.max_distance:
                matched_tracks.add(r)
                matched_dets.add(c)
                tid = track_ids[r]
                obs_car_id, obs_pos, from_visual = detections[c]
                self.tracks[tid].update(obs_pos, obs_car_id, from_visual)

        # 未匹配的轨迹标记丢失
        for i in range(n_tracks):
            if i not in matched_tracks:
                tid = track_ids[i]
                self.tracks[tid].mark_missed()

        # 未匹配的观测创建新轨迹（仅当观测来自视觉时才能创建）
        for j in range(n_dets):
            if j not in matched_dets:
                car_id, pos, from_visual = detections[j]
                if from_visual:
                    # 视觉观测可以创建新轨迹
                    new_tid = self._get_new_id()
                    self.tracks[new_tid] = Track(new_tid, car_id, pos, dt=self.dt, from_visual=True)
                # 点云观测（from_visual=False）不能创建新轨迹，直接丢弃

        self._remove_dead_tracks()

        # 返回确认态轨迹（只返回 has_ever_matched == True 的轨迹）
        result = []
        for track in self.tracks.values():
            if track.state == TrackState.Confirmed and track.has_ever_matched:
                result.append((track.car_id, track.get_position()))
        return result

    def _remove_dead_tracks(self):
        dead = [tid for tid, t in self.tracks.items() if t.state == TrackState.Deleted]
        for tid in dead:
            del self.tracks[tid]


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
        # detector_config_path = "./configs/detector_config.yaml"
        # binocular_camera_cfg_path = "./configs/bin_cam_config.yaml"
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        converter_config_path = CONVERTER_CONFIG_PATH
        
        # 全局变量
        self.global_my_color = main_cfg['global']['my_color']
        is_debug = main_cfg['global']['is_debug']
        self.carList = CarList(main_cfg)
        self.carList_results = []
        self.all_detections = [] # 创建一个空列表来存储所有检测的结果
        self.last_all_detections = [] # 创建一个空列表来存储上一帧的检测结果
        # 当前帧ID
        self.frame_id = 1
        # 计数器，用于计算fps
        self.counter = 0
        
        today = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) # 今日日期，例如2024年5月6日则为20240506
        self.converter = Converter(self.global_my_color,converter_config_path)  # 传入的是path
        # 场地解算
        # converter.camera_to_field_init(capture)
        # self.converter_inted = False
        
        # 加载转换器配置文件
        with open(converter_config_path, 'r', encoding='utf-8') as file:
            data_loader = yaml.safe_load(file)
            
        # 读取激光雷达到相机外参
        # 构建旋转和平移矩阵，4*3的矩阵，前三列为旋转矩阵，第四列为平移矩阵
        self.R = np.array(data_loader['calib']['extrinsic']['R']['data']).reshape(
            (data_loader['calib']['extrinsic']['R']['rows'], data_loader['calib']['extrinsic']['R']['cols']))
        self.T = np.array(data_loader['calib']['extrinsic']['T']['data']).reshape(
            (data_loader['calib']['extrinsic']['T']['rows'], data_loader['calib']['extrinsic']['T']['cols']))
        # 构建外参矩阵，4*4的矩阵，激光雷达到相机的外参矩阵
        self.extrinsic_matrix = np.hstack((self.R, self.T))
        self.extrinsic_matrix = np.vstack((self.extrinsic_matrix, [0, 0, 0, 1]))
        # 相机到激光雷达的变换矩阵（通过求逆得到）
        self.extrinsic_matrix_inv = np.linalg.inv(self.extrinsic_matrix)
        
        
        self.start_time = time.time()
        # fps计算
        N = 10
        self.fps_queue = deque(maxlen=10)
        self.lidar_points = None
        # 订阅Robots话题
        self.sub_detect = self.create_subscription(Robots, "detect_result", self.radar_callback, qos_profile)
        self.get_logger().info('Radar subscriber has been started at {}.'.format(today))
        # 订阅背景减除后的点云话题
        self.sub_pcds = self.create_subscription(PointCloud2, "target_pointcloud", self.pcd_callback, qos__lidar_profile)
        # 发布车辆位置信息
        self.pub_location = self.create_publisher(Locations, "location", qos_profile)
        self.last_frameid = -1
        # 可视化去除地面后的点云
        self.pub_nognd = self.create_publisher(PointCloud2, "pcd_removed", qos__lidar_profile)
        
        # 订阅实时icp传来的tf消息
        self.tf_buffer = Buffer()  # 创建 TF 缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建监听器
        self.timer = self.create_timer(1.0, self.on_timer)  # 定时查询 TF
        self.radar_to_field = np.ones((4, 4)) # 激光雷达到赛场的tf矩阵
        self.radar_to_field_inv = np.ones((4, 4)) # 激光雷达到赛场的tf矩阵的逆（用于将点云转换回雷达坐标系）

        # 新增：追踪器参数
        self.MATCH_THRESHOLD = 0.8
        self.MIN_BOX_DIST_PX = 100
        self.MIN_CLUSTER_POINTS_FOR_SPLIT = 20
        self.tracker = MultiTargetTracker(max_distance=0.5, max_age=5, min_hits=3, dt=0.1)
    ######################################################
    # 定时查询 TF
    def on_timer(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox',
                time=rclpy.time.Time()  # 获取最新可用变换
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            # self.log_transform(transform)
            # 转换为 4x4 变换矩阵
            transform_matrix = self.tf_to_matrix(translation, rotation)
            self.radar_to_field = transform_matrix
            self.radar_to_field_inv = np.linalg.inv(self.radar_to_field)
            # self.get_logger().info(f"获取 TF 成功: {transform}")
        except TransformException as ex:
            self.radar_to_field = np.ones((4, 4))
            self.get_logger().error(f"获取 TF 失败: {ex}")    
    
    # 将 TF 转换为 4x4 齐次变换矩阵
    def tf_to_matrix(self, translation, rotation):
        # 四元数转旋转矩阵
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        R = self.quaternion_to_rotation_matrix(q)

        # 构建 4x4 齐次变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return transform_matrix
    def quaternion_to_rotation_matrix(self, q):
        # 四元数转旋转矩阵
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])
    
    # 将相机坐标系下的点云转换到激光雷达坐标系
    def camera_to_lidar(self, pc):
        pc = np.hstack((pc, np.ones((pc.shape[0], 1)))) # 齐次化
        ret = np.dot(pc, self.extrinsic_matrix) # 矩阵变换
        ret = ret[:, :3] # 去齐次化
        return ret
    # 点云回调函数
    def pcd_callback(self, msg):
        '''
        子线程函数，对于/livox/lidar topic数据的处理 , data是传入的
        '''
        points = np.array(
                list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if points.size == 0:
            self.lidar_points = None
            return

        points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float64)
        self.lidar_points = np.ascontiguousarray(points)
        # self.converter.lidar_to_field(points)
   ########################################################3
    # DBSCAN 聚类（在雷达坐标系下）
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

    # 新增方法：计算簇特征
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

    # 新增方法：K-means 拆分簇
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
    #############################################################
    # 辅助发布方法（原始 CarList 发布逻辑）
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
    ######################################################
    # 雷达回调函数（增强版）
    def radar_callback(self, msg):
        detect_results = msg.detect_results
        if self.lidar_points is None:
            return

        # 1. 对雷达坐标系点云聚类
        clusters = self.cluster_points_dbscan(self.lidar_points, eps=0.15, min_samples=9)
        if not clusters:
            # 无点云簇，直接更新追踪器（观测为空）
            tracked = self.tracker.update([])
            self.carList_results.clear()
            null_robot_locations = []
            for car_id, pos in tracked:
                if car_id > 0:
                    self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
                elif car_id == 0:
                    null_robot_locations.append(pos)
            self._publish_through_carlist(null_robot_locations)
            return

        # 2. 提取每个簇的特征（利用 Converter 完成坐标转换和投影）
        cluster_features = []
        cluster_raw_points = []
        for cluster_pts in clusters:
            feat = self.compute_cluster_features(cluster_pts)
            if feat is not None:
                cluster_features.append(feat)
                cluster_raw_points.append(cluster_pts)

        if not cluster_features:
            return

        # 3. 匈牙利匹配（全局最优）
        n_clusters = len(cluster_features)
        n_boxes = len(detect_results)
        if n_boxes == 0:
            # 无检测框，所有簇作为点云观测（from_visual=False）
            observations = []
            for feat in cluster_features:
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False))
            tracked = self.tracker.update(observations)
            self.carList_results.clear()
            null_robot_locations = []
            for car_id, pos in tracked:
                if car_id > 0:
                    self.carList_results.append([car_id, car_id, [0,0,0,0], 1, [0,0,0], pos])
                elif car_id == 0:
                    null_robot_locations.append(pos)
            self._publish_through_carlist(null_robot_locations)
            return

        # 构建代价矩阵
        INF_COST = 1e9
        cost_matrix = np.full((n_clusters, n_boxes), INF_COST, dtype=np.float32)

        #填充代价
        for i, feat in enumerate(cluster_features):
            for j, det in enumerate(detect_results):
                xywh_box = det.xywh_box          # [cx, cy, w, h]
                box_center = np.array([xywh_box[0], xywh_box[1]])
                w, h = xywh_box[2], xywh_box[3]
                # 框边界
                x1 = xywh_box[0] - w/2
                y1 = xywh_box[1] - h/2
                x2 = xywh_box[0] + w/2
                y2 = xywh_box[1] + h/2

                pixels = feat['pixels']
                inside = (pixels[:,0] >= x1) & (pixels[:,0] <= x2) & \
                        (pixels[:,1] >= y1) & (pixels[:,1] <= y2)
                ratio = np.sum(inside) / pixels.shape[0] if pixels.shape[0] > 0 else 0

                dist = np.linalg.norm(feat['center_pixel'] - box_center)
                # 宽松过滤
                if ratio > 0.75 and dist < 200:
                    diag = np.sqrt(w**2 + h**2)
                    norm_dist = dist / diag  # 使用对角线归一化

                    # 代价 = 占比惩罚 + 归一化距离
                    cost = (1 - ratio) + norm_dist
                    cost_matrix[i, j] = cost
                # 否则保持 INF_COST

        # 执行匈牙利算法
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matches = []
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < INF_COST / 2:  # 有效匹配
                matches.append((r, c))

        # 第一步：一对一匹配
        matched_cluster_to_box = {}
        matched_box_to_cluster = {}
        for cluster_idx, det_idx in matches:
            matched_cluster_to_box[cluster_idx] = det_idx
            matched_box_to_cluster[det_idx] = cluster_idx

        # 第二步：未匹配框尝试分配给已有簇（一对多补充）
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

        # 合并匹配结果
        final_cluster_to_boxes = {}
        for i in matched_cluster_to_box.keys():
            boxes = [matched_cluster_to_box[i]] + extra_matches[i]
            final_cluster_to_boxes[i] = boxes

        # 收集观测 (car_id, field_xyz, from_visual)
        observations = []   # 每个元素为 (car_id, field_xyz, from_visual)
        all_matched_clusters = set(final_cluster_to_boxes.keys())

        # 7.1 匹配成功的簇（单匹配或多匹配）-> from_visual=True
        for cluster_idx, box_indices in final_cluster_to_boxes.items():
            if len(box_indices) == 1:
                det = detect_results[box_indices[0]]
                label = det.label
                if label == "NULL":
                    car_id = 0
                else:
                    car_id = self.carList.get_car_id(label)
                    # 颜色过滤：己方车辆不生成观测（但追踪器内部仍可能保留，最终发布时会过滤）
                    if self.global_my_color == "Red" and car_id < 100 and car_id != 7:
                        continue
                    if self.global_my_color == "Blue" and car_id > 100 and car_id != 107:
                        continue
                feat = cluster_features[cluster_idx]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((car_id, field_xyz, True))
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
                    else:
                        car_id = self.carList.get_car_id(label)
                        if self.global_my_color == "Red" and car_id < 100 and car_id != 7:
                            continue
                        if self.global_my_color == "Blue" and car_id > 100 and car_id != 107:
                            continue
                    center_h = np.append(sub_feat['center_lidar'], 1.0)
                    field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                    observations.append((car_id, field_xyz, True))

        # 7.2 未匹配的簇（完全没有匹配到任何框的簇）-> from_visual=False
        for i in range(n_clusters):
            if i not in all_matched_clusters:
                feat = cluster_features[i]
                center_h = np.append(feat['center_lidar'], 1.0)
                field_xyz = np.dot(self.radar_to_field, center_h)[:3]
                observations.append((0, field_xyz, False))

        # 8. 更新追踪器
        tracked_results = self.tracker.update(observations)

        # 9. 转换为 carList_results 和 null_robot_locations
        self.carList_results.clear()
        null_robot_locations = []
        for car_id, pos in tracked_results:
            if car_id > 0:
                self.carList_results.append([
                    car_id, car_id, [0,0,0,0], 1, [0,0,0], pos
                ])
            elif car_id == 0:
                null_robot_locations.append(pos)

        # 10. 发布
        self._publish_through_carlist(null_robot_locations)

    def __del__(self):
        pass
    #######################################################
def main(args=None):
    rclpy.init(args=args)
    radar = Radar()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
"""
detector_node.py — 激光雷达方案的视觉检测节点（带单目3D坐标 + 小地图 + 匈牙利跟踪）
================================================================================
功能：
  - 使用 YoloPipeline 进行三阶段推理与 ByteTrack/BotSORT 跟踪
  - 透视变换计算检测框底部中心的赛场坐标（单目3D）
  - 匈牙利跟踪器 (HungarianTracker) 进行时序关联、卡尔曼平滑、身份投票
  - 小地图窗口 "MiniMap (Camera Only)"，显示跟踪后的机器人赛场位置
  - 保留原有录制、rosbag 支持、"Window" 窗口
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from ruamel.yaml import YAML

import torch
import functools

# ---------- PyTorch 兼容性 Monkey-Patch ----------
_original_torch_load = torch.load
@functools.wraps(_original_torch_load)
def _patched_torch_load(*args, **kwargs):
    kwargs.setdefault('weights_only', False)
    return _original_torch_load(*args, **kwargs)
torch.load = _patched_torch_load

from detect_result.msg import DetectResult, Robots
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import time
import threading
import numpy as np
import os
import json
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ..shared.paths import (
    PROJECT_ROOT, DETECTOR_CONFIG_PATH, MAIN_CONFIG_PATH, RECORD_DIR,
    PERSPECTIVE_CALIB_PATH, PFA_MAP_2025_PATH, PFA_MAP_MASK_2025_PATH,
    resolve_path, FIELD_WIDTH, FIELD_HEIGHT,
)
from ..detection.yolo_pipeline import YoloPipeline
from ..filters.hungarian_tracker import HungarianTracker
from ..shared.type import SingleDetectionResult, TrackingState
from ..shared.utils import nms_xywh


# ==================== 透视变换类（移植自 camera_detector） ====================
class HomographyTransformer:
    def __init__(self, logger, my_color, calib_path=PERSPECTIVE_CALIB_PATH,
                 mask_path=PFA_MAP_MASK_2025_PATH):
        self.logger = logger
        self.my_color = my_color
        self.H_ground = None
        self.H_highland = None
        self.calib_map_w = None
        self.calib_map_h = None
        self.calib_map_portrait = False
        self.mask_img = None
        self._load_calib(calib_path)
        self._load_mask(mask_path)

    def _load_calib(self, path):
        if os.path.exists(path):
            with open(path, 'r') as f:
                data = json.load(f)
            if 'H_ground' in data:
                self.H_ground = np.array(data['H_ground'], dtype=np.float64)
            if 'H_highland' in data:
                self.H_highland = np.array(data['H_highland'], dtype=np.float64)
            if 'H' in data and self.H_ground is None:
                self.H_ground = np.array(data['H'], dtype=np.float64)
            if 'map_w' in data and 'map_h' in data:
                self.calib_map_w = int(data['map_w'])
                self.calib_map_h = int(data['map_h'])
                self.calib_map_portrait = data.get('map_is_portrait', False)
            self.logger.info("透视变换矩阵加载成功")
        else:
            self.logger.error(f"透视变换标定文件不存在: {path}")

    def _load_mask(self, path):
        if os.path.exists(path):
            self.mask_img = cv2.imread(path)

    def _map_pixel_to_field(self, map_px, map_py):
        if self.calib_map_w is None:
            return map_px, map_py
        if self.calib_map_portrait:
            if self.my_color == 'Red':
                field_x = (self.calib_map_h - map_py) / self.calib_map_h * FIELD_WIDTH
                field_y = (self.calib_map_w - map_px) / self.calib_map_w * FIELD_HEIGHT
            else:
                field_x = map_py / self.calib_map_h * FIELD_WIDTH
                field_y = map_px / self.calib_map_w * FIELD_HEIGHT
        else:
            field_x = map_px / self.calib_map_w * FIELD_WIDTH
            field_y = (self.calib_map_h - map_py) / self.calib_map_h * FIELD_HEIGHT
        return field_x, field_y

    def pixel_to_field(self, px, py):
        if self.H_ground is None:
            return None
        pt = np.array([[[px, py]]], dtype=np.float64)
        transformed = cv2.perspectiveTransform(pt, self.H_ground)
        map_x = transformed[0][0][0]
        map_y = transformed[0][0][1]
        margin = 500
        if self.calib_map_w is not None:
            if (map_x < -margin or map_x > self.calib_map_w + margin or
                map_y < -margin or map_y > self.calib_map_h + margin):
                return None
        # 高地层检测（简化，完整逻辑参照 camera_detector）
        if self.mask_img is not None and self.H_highland is not None:
            mask_h, mask_w = self.mask_img.shape[:2]
            mx = int(map_x * mask_w / self.calib_map_w)
            my = int(map_y * mask_h / self.calib_map_h)
            mx = max(0, min(mx, mask_w - 1))
            my = max(0, min(my, mask_h - 1))
            pixel_color = self.mask_img[my, mx]
            is_highland = not (pixel_color[0] == 0 and pixel_color[1] == 0 and pixel_color[2] == 0)
            if is_highland:
                transformed_h = cv2.perspectiveTransform(pt, self.H_highland)
                map_x = transformed_h[0][0][0]
                map_y = transformed_h[0][0][1]
        fx, fy = self._map_pixel_to_field(map_x, map_y)
        return fx, fy


# ==================== 检测节点类 ====================
class Detector(Node):
    def __init__(self):
        super().__init__('detector')

        self.start_time = time.time()
        self.bridge = CvBridge()
        self.cfg = YAML().load(open(DETECTOR_CONFIG_PATH, encoding='utf-8', mode='r'))
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='utf-8', mode='r'))
        self.camera_mode = main_cfg.get('camera', {}).get('mode', 'hik')
        self.get_logger().info(f'图像来源模式: {self.camera_mode}')

        self.my_color = main_cfg['global']['my_color']
        self.is_debug = main_cfg['global']['is_debug']

        # ---------- 初始化 YoloPipeline（原有推理管线） ----------
        self.pipeline = YoloPipeline(
            det_cfg=self.cfg,
            resolve_fn=resolve_path,
            logger=self.get_logger(),
            high_conf_correction=True,
        )

        # ---------- 透视变换器 ----------
        self.homography = HomographyTransformer(self.get_logger(), self.my_color)

        # ---------- 匈牙利跟踪器（来自 camera_detector） ----------
        track_params = self.cfg.get('track', {})
        self.hungarian = HungarianTracker(
            iou_thr=float(track_params.get('iou_thr', 0.05)),
            dist_thr=float(track_params.get('dist_thr', 120)),
            max_miss=int(track_params.get('max_miss', 54)),
            lost_thr=int(track_params.get('lost_thr', 3)),
            guess_thr=int(track_params.get('guess_thr', 18))
        )
        self.publish_predict_when_no_det = bool(track_params.get('publish_predict_when_no_det', True))

        # ---------- 小地图 ----------
        self.map_img = None
        if os.path.exists(PFA_MAP_2025_PATH):
            self.map_img = cv2.imread(PFA_MAP_2025_PATH)
            if self.map_img is not None:
                self.get_logger().info(f"小地图图片加载成功: {PFA_MAP_2025_PATH}")
            else:
                self.get_logger().warn("小地图图片读取失败")
        else:
            self.get_logger().info("小地图文件不存在，禁用小地图")
        self.minimap_initialized = False

        # 记录与录制
        self.is_record = self.cfg['is_record']
        self.record_fps = self.cfg['record_fps']

        res_cfg = self.cfg.get('resolution', {})
        self.infer_w = int(res_cfg.get('infer_width', 1920))
        self.infer_h = int(res_cfg.get('infer_height', 1080))
        self.display_w = int(res_cfg.get('display_width', 1920))
        self.display_h = int(res_cfg.get('display_height', 1080))

        self.loop_times = 0
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )

        self.frame = None
        self._frame_lock = threading.Lock()

        # 图像来源
        if self.camera_mode == 'rosbag':
            compressed_topic = main_cfg.get('camera', {}).get('compressed_image_topic', '/compressed_image')
            self.get_logger().info(f'Rosbag 模式: 订阅压缩图像 topic "{compressed_topic}"')
            self.sub_compressed = self.create_subscription(
                CompressedImage, compressed_topic, self._compressed_image_callback, qos_profile
            )
            self.is_record = False
        else:
            from ..Camera.HKCam import HKCam
            self.cam = HKCam(0)
            self.frame_threading = threading.Thread(target=self.sync_frame)
            self.frame_threading.start()

        self.publisher_ = self.create_publisher(Robots, 'detect_result', qos_profile)
        self.pub_res = self.create_publisher(Image, 'detect_view', qos_profile)

        self.threading = threading.Thread(target=self.infer_loop)
        self.threading.start()

        self.image_queue = deque(maxlen=10)
        self.timestamp_queue = deque(maxlen=10)
        if self.is_record:
            os.makedirs(RECORD_DIR, exist_ok=True)
            cur_date = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
            self.video_writer = cv2.VideoWriter(
                os.path.join(RECORD_DIR, cur_date + ".avi"),
                cv2.VideoWriter_fourcc(*'XVID'), 60, (3072, 2048)
            )
            self.timestamp_file = open(os.path.join(RECORD_DIR, cur_date + ".txt"), "w")
            self.save_thread = threading.Thread(target=self.save_image)
            self.save_thread.start()

        self._need_reset_tracker = False

    # ---------- 图像回调 ----------
    def _compressed_image_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                with self._frame_lock:
                    self.frame = cv_image
        except Exception as e:
            self.get_logger().error(f'解码压缩图像失败: {e}')

    def save_image(self):
        while rclpy.ok():
            if self.is_record:
                if self.image_queue and self.timestamp_queue:
                    img = self.image_queue.popleft()
                    ts = self.timestamp_queue.popleft()
                    self.video_writer.write(img)
                    self.timestamp_file.write(str(ts) + "\n")
                    self.timestamp_file.flush()
                    time.sleep(1/self.record_fps)
            else:
                time.sleep(0.1)

    def sync_frame(self):
        while rclpy.ok():
            cam_frame = self.cam.getFrame()
            self.image_queue.append(cam_frame)
            self.timestamp_queue.append(time.time())
            with self._frame_lock:
                self.frame = cam_frame.copy()

    def getFrame(self):
        with self._frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def _reset_tracking_state(self):
        self.pipeline.reset_tracking_state()
        if hasattr(self, 'hungarian'):
            self.hungarian.tracks.clear()
            self.hungarian.next_id = 1
        if self.is_debug:
            self.get_logger().info("跟踪器状态已重置")

    # ---------- 小地图绘制 ----------
    def _draw_minimap(self, active_robots):
        if self.map_img is None:
            return
        try:
            show_map = self.map_img.copy()
            for robot in active_robots:
                if robot.field_x is None or robot.field_y is None:
                    continue
                x, y = robot.field_x, robot.field_y
                map_xx = int(round(x * 100))
                map_yy = int(round(1500 - y * 100))
                map_xx = max(0, min(map_xx, 2800))
                map_yy = max(0, min(map_yy, 1500))
                best_label = max(robot.vote_pool, key=robot.vote_pool.get) if robot.vote_pool else "NULL"
                if best_label.startswith('R'):
                    color = (0, 0, 255)
                elif best_label.startswith('B'):
                    color = (250, 100, 0)
                else:
                    color = (0, 255, 0)
                cv2.circle(show_map, (map_xx, map_yy), 40, color, 3)
                cv2.putText(show_map, best_label, (map_xx - 15, map_yy - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.imshow("MiniMap (Camera Only)", show_map)
        except Exception as e:
            self.get_logger().warn(f"小地图绘制异常: {e}")

    # ---------- 推理主循环 ----------
    def infer_loop(self):
        cv2.namedWindow("Window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Window", self.display_w, self.display_h)

        if self.map_img is not None and not self.minimap_initialized:
            cv2.namedWindow("MiniMap (Camera Only)", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("MiniMap (Camera Only)", 800, 471)
            self.minimap_initialized = True

        orig_w, orig_h = None, None
        while rclpy.ok():
            if getattr(self, '_need_reset_tracker', False):
                self._reset_tracking_state()
                self._need_reset_tracker = False

            now = time.time()
            dt = max(0.01, min(0.2, now - self.start_time))
            fps = 1.0 / max(now - self.start_time, 1e-6)
            self.start_time = now

            try:
                cv_image = self.getFrame()
                if cv_image is None:
                    continue

                if orig_w is None:
                    orig_h, orig_w = cv_image.shape[:2]
                    self.get_logger().info(f'原图: {orig_w}×{orig_h}, 推理: {self.infer_w}×{self.infer_h}')

                infer_img = cv2.resize(cv_image, (self.infer_w, self.infer_h))
                result_img, results = self.pipeline.infer(infer_img)

                detections = []
                if results is not None:
                    for res in results:
                        xyxy, xywh, track_id, label = res
                        detections.append(SingleDetectionResult(
                            xyxy=xyxy, xywh=xywh, label=label, conf=1.0, track_id=track_id
                        ))

                # 匈牙利跟踪器更新
                active_robots = self.hungarian.update(detections, dt)

                # 生成 DetectResult 消息（带单目3D坐标）
                allRobots = Robots()
                for robot in active_robots:
                    if robot.state not in [TrackingState.TRACKING, TrackingState.LOST, TrackingState.GUESSING]:
                        continue
                    best_label = max(robot.vote_pool, key=robot.vote_pool.get) if robot.vote_pool else "NULL"

                    kf_cx, kf_cy, kf_w, kf_h = robot.bbox_kf_state[:4]
                    orig_cx = kf_cx * orig_w / self.infer_w
                    orig_cy = kf_cy * orig_h / self.infer_h
                    orig_w_box = kf_w * orig_w / self.infer_w
                    orig_h_box = kf_h * orig_h / self.infer_h

                    msg = DetectResult()
                    x1 = int(orig_cx - orig_w_box / 2)
                    y1 = int(orig_cy - orig_h_box / 2)
                    x2 = int(orig_cx + orig_w_box / 2)
                    y2 = int(orig_cy + orig_h_box / 2)
                    msg.xyxy_box = [x1, y1, x2, y2]
                    msg.xywh_box = [float(orig_cx), float(orig_cy), float(orig_w_box), float(orig_h_box)]
                    msg.track_id = robot.id
                    msg.label = best_label

                    # 单目3D坐标
                    px = orig_cx
                    py = orig_cy + orig_h_box / 2.0
                    field_xy = self.homography.pixel_to_field(px, py)
                    if field_xy is not None:
                        msg.field_x = float(field_xy[0])
                        msg.field_y = float(field_xy[1])
                        msg.field_z = 0.0
                        robot.field_x = msg.field_x
                        robot.field_y = msg.field_y
                    else:
                        msg.field_x = 0.0
                        msg.field_y = 0.0
                        msg.field_z = 0.0

                    allRobots.detect_results.append(msg)

                # 发布可视化图像
                if result_img is not None:
                    img_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
                    self.pub_res.publish(img_msg)
                    cv2.imshow("Window", result_img)
                else:
                    cv2.imshow("Window", infer_img)

                self.publisher_.publish(allRobots)

                # 小地图
                self._draw_minimap(active_robots)

                cv2.waitKey(1)

            except Exception as e:
                self.get_logger().error(f'推理循环异常: {e}')
                import traceback
                traceback.print_exc()

    def __del__(self):
        cv2.destroyAllWindows()
        self.get_logger().info('Detector node 正在销毁。')


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
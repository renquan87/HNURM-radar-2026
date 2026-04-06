"""
camera_detector.py — 纯相机透视变换雷达站检测节点
功能：调用海康工业相机，使用 YOLO 识别赛场地面机器人，
      通过透视变换（Homography）将图像像素坐标映射到赛场坐标，
      并通过 ROS2 话题发布检测结果供 display_panel 等节点使用。

与已有代码（相机+激光雷达方案）的区别：
  - 不依赖激光雷达点云和外参矩阵
  - 不依赖 ICP / TF 重定位
  - 使用 4 组以上「图像像素 ↔ 赛场坐标」对应点计算单应性矩阵（Homography）
  - 检测到机器人后取包围框底部中心点，通过透视变换直接得到赛场 (x, y)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from detect_result.msg import Location, Locations
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ruamel.yaml import YAML

# from ..Camera.HKCam import *  # 使用海康工业相机时取消注释
from ..Car.Car import CarList
from ..camera_locator.anchor import Anchor
from ..camera_locator.point_picker import PointsPicker
from ..filters.kalman_filter import KalmanFilterWrapper

import cv2
import numpy as np
import threading
import time
import sys
import os
import json
from collections import deque
from ..shared.paths import (
    MAIN_CONFIG_PATH, DETECTOR_CONFIG_PATH, PERSPECTIVE_CALIB_PATH,
    PFA_MAP_MASK_2025_PATH, TEST_RESOURCES_DIR,
    resolve_path, FIELD_WIDTH, FIELD_HEIGHT,
)
from ..detection.yolo_pipeline import YoloPipeline

# ======================== 配置路径 ========================
# 分区掩码图（PFA 2025 赛季掩码，后续用 make_mask 重新绘制 2026 版本）
MASK_IMAGE_PATH = PFA_MAP_MASK_2025_PATH

# ======================== 视频输入源 ========================
# 现在通过 main_config.yaml 中的 camera_mode 配置项控制：
#   'test'  — 测试模式，使用静态图片反复推理
#   'video' — USB 摄像头 / 视频文件
#   'hik'   — 海康工业相机


class CameraDetector(Node):
    """
    纯相机透视变换雷达站节点
    ──────────────────────
    启动后流程：
      1. 打开海康相机并持续读取图像
      2. （首次/需要时）标定：用户在图像上选 4 个点，输入对应赛场坐标，
         计算 Homography 矩阵并保存
      3. YOLO 三阶段推理（复用已有 detector_node 的逻辑）识别机器人
      4. 取每个机器人检测框底部中心的像素坐标，通过 H 矩阵变换到赛场坐标
      5. 将结果发布到 ROS2 话题 & 在小地图窗口中实时绘制
    """

    def __init__(self):
        super().__init__('camera_detector')
        self.get_logger().info("CameraDetector 节点启动 ...")

        # ---------- 加载配置 ----------
        self.main_cfg = YAML().load(
            open(MAIN_CONFIG_PATH, encoding='Utf-8', mode='r'))
        self.det_cfg = YAML().load(
            open(DETECTOR_CONFIG_PATH, encoding='Utf-8', mode='r'))

        self.my_color = self.main_cfg['global']['my_color']
        self.is_debug = self.main_cfg['global']['is_debug']
        self.debug_coordinate_publish = self.main_cfg['global'].get(
            'debug_coordinate_publish', False)
        self.bridge = CvBridge()

        # ---------- 初始化 CarList ----------
        self.carList = CarList(self.main_cfg)

        # ---------- 三阶段推理管线 ----------
        self.pipeline = YoloPipeline(
            det_cfg=self.det_cfg,
            resolve_fn=resolve_path,
            logger=self.get_logger(),
            high_conf_correction=True,
        )
        self.class_num = self.pipeline.class_num

        # ── 推理/显示分辨率（从配置读取，兼容旧配置） ──
        res_cfg = self.det_cfg.get('resolution', {})
        self.infer_w = int(res_cfg.get('infer_width', 1920))
        self.infer_h = int(res_cfg.get('infer_height', 1080))
        self.display_w = int(res_cfg.get('display_width', 1920))
        self.display_h = int(res_cfg.get('display_height', 1080))

        # ---------- QoS ----------
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )

        # ---------- ROS2 发布 ----------
        self.pub_location = self.create_publisher(Locations, "location", qos_profile)
        self.pub_detect_view = self.create_publisher(Image, 'detect_view', qos_profile)

        # ---------- 视频输入源（根据配置选择模式） ----------
        self.frame = None
        self._frame_lock = threading.Lock()
        self.use_hkcam = False
        self.video_cap = None
        self.cam = None
        self.test_image = None  # test 模式下的静态图片

        camera_cfg = self.main_cfg.get('camera', {})
        self.camera_mode = camera_cfg.get('mode', 'video')
        _raw_video_source = camera_cfg.get('video_source', 0)
        # 字符串路径需要 resolve_path() 解析相对路径；整数（USB 设备号）直接使用
        video_source = (resolve_path(str(_raw_video_source))
                        if isinstance(_raw_video_source, str) else _raw_video_source)
        test_img_path = resolve_path(camera_cfg.get('test_image',
            os.path.join(TEST_RESOURCES_DIR, 'pfa_test_image.jpg')))

        if self.camera_mode == 'test':
            # ====== 测试模式：使用静态图片反复推理 ======
            self.test_image = cv2.imread(test_img_path)
            if self.test_image is None:
                self.get_logger().error(f"测试图片无法加载: {test_img_path}")
            else:
                self.get_logger().info(f"测试模式：使用静态图片 {test_img_path} "
                                       f"({self.test_image.shape[1]}×{self.test_image.shape[0]})")
        elif self.camera_mode == 'hik':
            # ====== 海康工业相机（比赛时使用） ======
            try:
                from ..Camera.HKCam import HKCam
                self.cam = HKCam(0)
                self.use_hkcam = True
                self.get_logger().info("使用海康工业相机 (HKCam)")
            except Exception as e:
                self.get_logger().error(f"海康相机初始化失败: {e}，回退到测试模式")
                self.camera_mode = 'test'
                self.test_image = cv2.imread(test_img_path)
        else:
            # ====== video 模式：USB 摄像头 / 视频文件 ======
            self.video_cap = cv2.VideoCapture(video_source)
            if not self.video_cap.isOpened():
                self.get_logger().error(
                    f"无法打开视频源: {video_source}，回退到测试模式")
                self.camera_mode = 'test'
                self.test_image = cv2.imread(test_img_path)
            else:
                src_name = (f"USB 摄像头 {video_source}"
                            if isinstance(video_source, int) else video_source)
                self.get_logger().info(f"使用视频源: {src_name}")

        # ---------- 透视变换矩阵（多层） ----------
        self.H_ground = None    # 地面层 Homography (相机像素→地图像素)
        self.H_highland = None  # 高地层 Homography
        self.H = None           # 兼容：默认指向地面层
        self.calib_map_w = None # 标定时使用的地图宽度(px)
        self.calib_map_h = None # 标定时使用的地图高度(px)
        self.calib_map_portrait = False  # 标定地图是否竖版
        self.mask_img = None    # 分区掩码图
        self._load_or_calibrate_homography()
        self._load_mask()

        # ---------- 坐标卡尔曼滤波器 ----------
        filter_cfg = self.det_cfg.get('filter', {})
        self.kf_wrapper = KalmanFilterWrapper(
            process_noise=float(filter_cfg.get('process_noise', 1e-2)),
            measurement_noise=float(filter_cfg.get('measurement_noise', 1e-1)),
            jump_threshold=float(filter_cfg.get('jump_threshold', 1.0)),
            max_velocity=float(filter_cfg.get('max_velocity', 5.0)),
            max_inactive_time=float(filter_cfg.get('max_inactive_time', 3.0)),
        )
        self._cleanup_counter = 0

        # ---------- 启动线程 ----------
        self.frame_thread = threading.Thread(target=self._sync_frame, daemon=True)
        self.frame_thread.start()

        self.infer_thread = threading.Thread(target=self._infer_loop, daemon=True)
        self.infer_thread.start()

        self.get_logger().info("CameraDetector 初始化完成。")

    # ================================================================
    #  透视变换标定
    # ================================================================
    def _load_or_calibrate_homography(self):
        """尝试从文件加载 Homography（支持多层），若不存在则启动交互式标定"""
        if os.path.exists(PERSPECTIVE_CALIB_PATH):
            self.get_logger().info(f"从 {PERSPECTIVE_CALIB_PATH} 加载透视变换标定 ...")
            with open(PERSPECTIVE_CALIB_PATH, 'r') as f:
                data = json.load(f)
            # 兼容旧格式（单层 H）和新格式（H_ground + H_highland）
            if 'H_ground' in data:
                self.H_ground = np.array(data['H_ground'], dtype=np.float64)
                self.get_logger().info("地面层透视变换矩阵加载成功。")
            if 'H_highland' in data:
                self.H_highland = np.array(data['H_highland'], dtype=np.float64)
                self.get_logger().info("高地层透视变换矩阵加载成功。")
            if 'H' in data and self.H_ground is None:
                # 旧格式兼容：只有单个 H，作为地面层使用
                self.H_ground = np.array(data['H'], dtype=np.float64)
                self.get_logger().info("兼容旧格式：单层 H 作为地面层矩阵。")
            self.H = self.H_ground  # 默认指向地面层
            # 读取标定时的地图尺寸信息（新格式）
            if 'map_w' in data and 'map_h' in data:
                self.calib_map_w = int(data['map_w'])
                self.calib_map_h = int(data['map_h'])
                self.calib_map_portrait = data.get('map_is_portrait', False)
                self.get_logger().info(
                    f"标定地图尺寸: {self.calib_map_w}×{self.calib_map_h}, "
                    f"竖版={self.calib_map_portrait}")
            else:
                self.get_logger().info("旧格式标定文件，无地图尺寸信息（假设H直接输出赛场米坐标）")
        else:
            self.get_logger().info("未找到透视变换标定文件，进入交互式标定 ...")
            self._interactive_calibrate()

    def _load_mask(self):
        """加载分区掩码图（黑色=地面，非黑色=高地）"""
        if os.path.exists(MASK_IMAGE_PATH):
            self.mask_img = cv2.imread(MASK_IMAGE_PATH)
            if self.mask_img is not None:
                h, w = self.mask_img.shape[:2]
                # 判断掩码方向：竖版(H>W) 或 横版(W>=H)
                self.mask_is_portrait = (h > w)
                if self.mask_is_portrait:
                    self.get_logger().info(
                        f"掩码图为竖版({w}×{h})，H→{FIELD_WIDTH}m, W→{FIELD_HEIGHT}m")
                else:
                    self.get_logger().info(
                        f"掩码图为横版({w}×{h})，W→{FIELD_WIDTH}m, H→{FIELD_HEIGHT}m")
                self.get_logger().info(f"分区掩码已加载: {MASK_IMAGE_PATH}")
            else:
                self.get_logger().warn(f"无法读取掩码图: {MASK_IMAGE_PATH}，将仅使用地面层")
        else:
            self.mask_is_portrait = False
            self.get_logger().info("未找到分区掩码图，将仅使用地面层透视变换。"
                                   f"（可使用 make_mask 工具生成: {MASK_IMAGE_PATH}）")

    def _interactive_calibrate(self):
        """
        交互式标定：提示用户运行独立的 GUI 标定工具。
        不再在 camera_detector 节点内做标定，避免阻塞 ROS2 节点。
        """
        self.get_logger().error(
            "═" * 60 + "\n"
            "  未找到透视变换标定文件！\n"
            f"  需要的文件: {PERSPECTIVE_CALIB_PATH}\n"
            "\n"
            "  请先运行 GUI 标定工具进行标定：\n"
            "    ros2 run hnurm_radar perspective_calibrator\n"
            "\n"
            "  标定完成后重新启动 camera_detector。\n"
            + "═" * 60
        )
        self.get_logger().info("节点将以无标定模式继续运行（坐标映射不可用）。")

    def recalibrate(self):
        """重新标定（可在运行时调用）"""
        if os.path.exists(PERSPECTIVE_CALIB_PATH):
            os.remove(PERSPECTIVE_CALIB_PATH)
        self.H_ground = None
        self.H_highland = None
        self.H = None
        self._interactive_calibrate()

    # ================================================================
    #  透视变换：像素 → 赛场坐标
    # ================================================================
    def _map_pixel_to_field(self, map_px, map_py):
        """
        将地图像素坐标转换为赛场米坐标。
        与原始 PFA 标定方案一致：H 矩阵输出地图像素，运行时再转赛场米。

        关键：红方和蓝方的地图是 180° 旋转关系，坐标转换公式不同！
        （参考 pfa_vision_radar/main.py 中 send_point_B / send_point_R）

        蓝方竖版地图:
          field_x = map_py / map_h * FIELD_WIDTH    (地图纵轴→赛场x)
          field_y = map_px / map_w * FIELD_HEIGHT    (地图横轴→赛场y)
        红方竖版地图（相对蓝方旋转180°）:
          field_x = (map_h - map_py) / map_h * FIELD_WIDTH
          field_y = (map_w - map_px) / map_w * FIELD_HEIGHT
        """
        if self.calib_map_w is None:
            # 旧格式：H 直接输出赛场米坐标，无需转换
            return map_px, map_py

        if self.calib_map_portrait:
            if self.my_color == 'Red':
                # 红方地图：坐标需要翻转（180°旋转关系）
                field_x = (self.calib_map_h - map_py) / self.calib_map_h * FIELD_WIDTH
                field_y = (self.calib_map_w - map_px) / self.calib_map_w * FIELD_HEIGHT
            else:
                # 蓝方地图
                field_x = map_py / self.calib_map_h * FIELD_WIDTH
                field_y = map_px / self.calib_map_w * FIELD_HEIGHT
        else:
            field_x = map_px / self.calib_map_w * FIELD_WIDTH
            field_y = (self.calib_map_h - map_py) / self.calib_map_h * FIELD_HEIGHT
        return field_x, field_y

    def pixel_to_field(self, px, py):
        """
        将图像像素坐标 (px, py) 通过多层 Homography 变换到赛场坐标 (fx, fy)。

        流程:
          1. H 矩阵将相机像素变换到地图像素坐标
          2. 用地图像素坐标查掩码判定高度层
          3. 地图像素坐标转换为赛场米坐标

        参数:
            px, py: 原始分辨率下的像素坐标
        返回:
            (field_x, field_y): 赛场坐标（单位 m），若变换失败返回 None
        """
        if self.H_ground is None:
            return None

        pt = np.array([[[px, py]]], dtype=np.float64)

        # Step 1: 地面层变换 → 地图像素坐标
        transformed = cv2.perspectiveTransform(pt, self.H_ground)
        map_x = transformed[0][0][0]
        map_y = transformed[0][0][1]

        # ★ 异常值检测：地图像素坐标超出合理范围则返回 None
        # 允许一定的边界外扩，但不能太离谱
        margin = 500  # 像素边界外扩
        if self.calib_map_w is not None:
            if (map_x < -margin or map_x > self.calib_map_w + margin or
                map_y < -margin or map_y > self.calib_map_h + margin):
                if self.is_debug:
                    self.get_logger().warn(
                        f"透视变换异常: px=({px:.0f},{py:.0f}) → map=({map_x:.1f},{map_y:.1f}) 超出范围")
                return None

        # Step 2: 查掩码判定高度层（直接用地图像素坐标查掩码）
        if self.mask_img is not None and self.H_highland is not None:
            mask_h, mask_w = self.mask_img.shape[:2]
            # 掩码与标定地图同方向同尺寸，直接按比例映射
            if self.calib_map_w is not None:
                mx = int(map_x * mask_w / self.calib_map_w)
                my = int(map_y * mask_h / self.calib_map_h)
            else:
                # 旧格式：map_x/map_y 是赛场米坐标
                if self.mask_is_portrait:
                    mx = int(map_y * mask_w / FIELD_HEIGHT)
                    my = int(map_x * mask_h / FIELD_WIDTH)
                else:
                    mx = int(map_x * mask_w / FIELD_WIDTH)
                    my = int(mask_h - map_y * mask_h / FIELD_HEIGHT)
            mx = max(0, min(mx, mask_w - 1))
            my = max(0, min(my, mask_h - 1))

            pixel_color = self.mask_img[my, mx]
            is_highland = not (pixel_color[0] == 0 and
                               pixel_color[1] == 0 and
                               pixel_color[2] == 0)

            if is_highland:
                # 用高地层矩阵重新变换
                transformed_h = cv2.perspectiveTransform(pt, self.H_highland)
                map_x = transformed_h[0][0][0]
                map_y = transformed_h[0][0][1]
                # 高地层变换后也要检查范围
                if (map_x < -margin or map_x > self.calib_map_w + margin or
                    map_y < -margin or map_y > self.calib_map_h + margin):
                    if self.is_debug:
                        self.get_logger().warn(
                            f"高地层透视变换异常: px=({px:.0f},{py:.0f}) → map=({map_x:.1f},{map_y:.1f})")
                    return None

        # Step 3: 地图像素 → 赛场米坐标
        fx, fy = self._map_pixel_to_field(map_x, map_y)
        return (fx, fy)

    def get_box_bottom_center(self, xyxy_box):
        """
        从检测框 [x1, y1, x2, y2] 中取底部中心点的像素坐标。
        底部中心更接近地面，透视变换精度更高。
        """
        x1, y1, x2, y2 = xyxy_box
        cx = (x1 + x2) / 2.0
        cy = y2  # 底部
        return cx, cy

    # ================================================================
    #  相机图像同步
    # ================================================================
    def _sync_frame(self):
        """持续从视频源获取图像（支持 test/video/hik 三种模式）"""
        while rclpy.ok():
            cam_frame = None

            if self.camera_mode == 'test':
                # 测试模式：反复使用同一张图片
                if self.test_image is not None:
                    cam_frame = self.test_image.copy()
                time.sleep(0.05)  # 模拟 ~20fps

            elif self.camera_mode == 'hik' and self.use_hkcam and self.cam is not None:
                # 海康工业相机
                cam_frame = self.cam.getFrame()

            elif self.video_cap is not None and self.video_cap.isOpened():
                # USB 摄像头 / 视频文件
                ret, cam_frame = self.video_cap.read()
                if not ret:
                    # 视频播放完毕，循环播放
                    self.video_cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ret, cam_frame = self.video_cap.read()
                    if not ret:
                        cam_frame = None
                    # ★ 重置跟踪器和滤波器状态，避免状态残留
                    self._reset_tracking_state()
                time.sleep(0.033)  # 30fps 视频帧率控制

            if cam_frame is not None:
                with self._frame_lock:
                    self.frame = cam_frame.copy()
            else:
                time.sleep(0.03)  # 没有画面时短暂等待

    def _get_frame(self):
        with self._frame_lock:
            if self.frame is None:
                return None
            return self.frame.copy()

    def _reset_tracking_state(self):
        """
        重置跟踪器和滤波器状态。
        在视频循环播放时调用，避免状态残留导致坐标错误。
        """
        # 重置卡尔曼滤波器
        self.kf_wrapper.reset()

        # 重置推理管线的投票表和 ByteTrack 跟踪器
        self.pipeline.reset_tracking_state()

        if self.is_debug:
            self.get_logger().info("跟踪器和滤波器状态已重置")

    # ================================================================
    #  主推理循环
    # ================================================================
    def _infer_loop(self):
        """主循环：推理 → 透视变换 → 发布位置 → 显示小地图"""
        # 等待标定完成
        while self.H_ground is None and rclpy.ok():
            self.get_logger().warn("等待透视变换标定 ...")
            time.sleep(1.0)

        cv2.namedWindow("CameraDetector", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("CameraDetector", self.display_w, self.display_h)

        start_time = time.time()
        # 推理用分辨率（从配置读取）
        INFER_W, INFER_H = self.infer_w, self.infer_h

        while rclpy.ok():
            now = time.time()
            fps = 1.0 / max(now - start_time, 1e-6)
            start_time = now

            try:
                cv_image = self._get_frame()
                if cv_image is None:
                    time.sleep(0.01)
                    continue

                # 动态获取原始图像分辨率
                ORIG_H, ORIG_W = cv_image.shape[:2]

                # 缩放到推理分辨率
                infer_image = cv2.resize(cv_image, (INFER_W, INFER_H))

                # 推理
                result_img, results = self.pipeline.infer(infer_image)

                # ---------- 透视变换 + 发布 ----------
                allLocation = Locations()
                carList_results = []

                if results is not None:
                    for result in results:
                        xyxy_box, xywh_box, track_id, label = result

                        # 暂时保留 NULL 标签用于测试定位效果
                        # if label == "NULL":
                        #     continue
                        
                        # 过滤己方车辆（debug 模式下保留己方，用于小地图展示）
                        car_id = self.carList.get_car_id(label) if label != "NULL" else -1
                        # 测试模式：允许 NULL 标签通过，使用 track_id 作为临时 ID
                        if car_id == -1 and label == "NULL":
                            car_id = 9000 + track_id  # 使用 9000+ 作为 NULL 机器人的临时 ID
                        elif car_id == -1:
                            continue
                        is_friendly = False
                        if self.my_color == "Red" and car_id < 100 and car_id != 7:
                            if not self.debug_coordinate_publish:
                                continue
                            is_friendly = True
                        if self.my_color == "Blue" and car_id > 100 and car_id != 107:
                            if not self.debug_coordinate_publish:
                                continue
                            is_friendly = True

                        # 将推理分辨率坐标还原到原始分辨率
                        orig_xyxy = [
                            int(xyxy_box[0] * ORIG_W / INFER_W),
                            int(xyxy_box[1] * ORIG_H / INFER_H),
                            int(xyxy_box[2] * ORIG_W / INFER_W),
                            int(xyxy_box[3] * ORIG_H / INFER_H),
                        ]

                        # 取检测框底部中心
                        px, py = self.get_box_bottom_center(orig_xyxy)

                        # 透视变换得到赛场坐标
                        field_coord = self.pixel_to_field(px, py)
                        if field_coord is None:
                            continue
                        field_x, field_y = field_coord

                        # 范围检查（赛场 FIELD_WIDTH × FIELD_HEIGHT）
                        if not (0 <= field_x <= FIELD_WIDTH and 0 <= field_y <= FIELD_HEIGHT):
                            if self.is_debug:
                                self.get_logger().warn(
                                    f"{label} 坐标超出范围: ({field_x:.2f}, {field_y:.2f})")
                            # 仍然保留，但限定范围
                            field_x = max(0, min(FIELD_WIDTH, field_x))
                            field_y = max(0, min(FIELD_HEIGHT, field_y))

                        # ★ 卡尔曼滤波平滑坐标
                        field_x, field_y = self.kf_wrapper.update(
                            car_id, field_x, field_y)
                        # 滤波后再次 clamp
                        field_x = max(0.0, min(FIELD_WIDTH, field_x))
                        field_y = max(0.0, min(FIELD_HEIGHT, field_y))

                        field_xyz = np.array([field_x, field_y, 0.0])

                        if self.is_debug:
                            self.get_logger().info(
                                f"[{label}] px=({px:.0f},{py:.0f}) → field=({field_x:.2f},{field_y:.2f})")

                        # 在推理图像上标注赛场坐标
                        disp_x = int(xyxy_box[0])
                        disp_y = int(xyxy_box[1]) - 10
                        cv2.putText(result_img,
                                    f"({field_x:.1f},{field_y:.1f})",
                                    (disp_x, disp_y),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

                        # 组装 CarList 结果（己方和 NULL 机器人跳过 CarList 更新）
                        if label != "NULL" and not is_friendly:
                            orig_xywh = [
                                float(xywh_box[0] * ORIG_W / INFER_W),
                                float(xywh_box[1] * ORIG_H / INFER_H),
                                float(xywh_box[2] * ORIG_W / INFER_W),
                                float(xywh_box[3] * ORIG_H / INFER_H),
                            ]
                            camera_xyz = np.array([0.0, 0.0, 0.0])  # 透视变换无3D相机坐标
                            carList_results.append([
                                track_id, car_id, orig_xywh, 1, camera_xyz, field_xyz
                            ])

                        # 发布 Location 消息
                        loc = Location()
                        loc.x = float(field_x)
                        loc.y = float(field_y)
                        loc.z = 0.0
                        loc.id = car_id
                        if is_friendly:
                            loc.label = "Friendly"
                        else:
                            loc.label = "Red" if car_id < 100 else "Blue"
                        allLocation.locs.append(loc)

                # 更新 CarList
                if carList_results:
                    self.carList.update_car_info(carList_results)
                self.pub_location.publish(allLocation)

                # ★ 定期清理超时的卡尔曼滤波器
                self._cleanup_counter += 1
                if self._cleanup_counter % 100 == 0:
                    self.kf_wrapper.cleanup()

                # ---------- 在推理图像上显示 FPS ----------
                cv2.putText(result_img, f"FPS: {fps:.1f}", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 2)
                cv2.imshow("CameraDetector", result_img)

                cv2.waitKey(1)

            except Exception as e:
                self.get_logger().error(f"推理循环异常: {e}")
                import traceback
                traceback.print_exc()

    # ================================================================
    #  析构
    # ================================================================
    def __del__(self):
        self.get_logger().info('CameraDetector 节点正在销毁。')


# ================================================================
#  入口
# ================================================================
def main(args=None):
    rclpy.init(args=args)
    node = CameraDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

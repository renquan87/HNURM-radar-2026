"""
detector_node.py — 激光雷达方案的视觉检测节点（方案二）

功能：
  获取图像并使用 YOLO 三阶段推理识别赛场机器人，
  将检测结果（bbox + track_id + 分类标签）通过 ROS2 话题发布给
  下游的 radar_node 进行点云投影定位。

  支持多种图像来源（由 main_config.yaml → camera.mode 控制）：
    - "hik":    海康工业相机硬件直接获取（默认，比赛/实验室用）
    - "rosbag": 从 ROS2 话题 /compressed_image 订阅压缩图像（rosbag 回放离线测试用）

三阶段推理流程（委托给 detection.yolo_pipeline.YoloPipeline）：
  Stage 1 — track_infer():     全图目标检测 + ByteTrack 多目标追踪
  Stage 2 — classify_infer():  ROI 装甲板颜色+编号分类
  Stage 3 — classify_infer():  ROI 灰色装甲板专用分类

与 camera_detector（方案一）的区别：
  - 本节点不做透视变换定位，仅输出 2D 检测框
  - 定位由下游 radar_node 结合点云完成（3D 投影 + DBSCAN 聚类）

发布话题：
  - detect_result (Robots)   — 所有检测到的机器人 bbox + 标签
  - detect_view   (Image)    — 带标注的检测结果图像

配置文件：
  - configs/detector_config.yaml — 模型路径、置信度阈值、生命周期等
  - configs/main_config.yaml     — camera.mode 控制图像来源
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from ruamel.yaml import YAML

from detect_result.msg import DetectResult
from detect_result.msg import Robots
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import time
import threading
import numpy as np
import os
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ..shared.paths import PROJECT_ROOT, DETECTOR_CONFIG_PATH, MAIN_CONFIG_PATH, RECORD_DIR, resolve_path
from ..detection.yolo_pipeline import YoloPipeline


# 检测节点类
class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        
        self.start_time = time.time()
        self.bridge = CvBridge()
        # 加载配置文件
        self.cfg = YAML().load(open(DETECTOR_CONFIG_PATH, encoding='Utf-8', mode='r'))
        # 加载主配置文件（用于读取 camera.mode）
        main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='Utf-8', mode='r'))
        self.camera_mode = main_cfg.get('camera', {}).get('mode', 'hik')
        self.get_logger().info(f'图像来源模式: {self.camera_mode}')

        # 记录相关参数 flag and fps
        self.is_record = self.cfg['is_record']
        self.record_fps = self.cfg['record_fps']

        # ── 推理/显示分辨率（从配置读取，兼容旧配置） ──
        res_cfg = self.cfg.get('resolution', {})
        self.infer_w = int(res_cfg.get('infer_width', 1920))
        self.infer_h = int(res_cfg.get('infer_height', 1080))
        self.display_w = int(res_cfg.get('display_width', 1920))
        self.display_h = int(res_cfg.get('display_height', 1080))
        
        # ── 初始化三阶段推理管线 ──
        self.pipeline = YoloPipeline(
            det_cfg=self.cfg,
            resolve_fn=resolve_path,
            logger=self.get_logger(),
            high_conf_correction=True,
        )
        
        # 设置计数器（pipeline 内部也维护 loop_times，此处用于节点级逻辑）
        self.loop_times = 0
        # QoS 设置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        
        self.frame = None
        # 创建图像资源锁
        self._frame_lock = threading.Lock()

        # ── 根据模式初始化图像来源 ──
        if self.camera_mode == 'rosbag':
            # rosbag 模式：订阅压缩图像 topic
            compressed_topic = main_cfg.get('camera', {}).get(
                'compressed_image_topic', '/compressed_image')
            self.get_logger().info(
                f'Rosbag 模式: 订阅压缩图像 topic "{compressed_topic}"')
            self.sub_compressed = self.create_subscription(
                CompressedImage,
                compressed_topic,
                self._compressed_image_callback,
                qos_profile
            )
            # rosbag 模式不录制（数据来自录像本身）
            self.is_record = False
        else:
            # hik 模式（默认）：打开海康工业相机
            from ..Camera.HKCam import HKCam
            self.cam = HKCam(0)
            # 启动图像同步线程
            self.frame_threading = threading.Thread(target=self.sync_frame)
            self.frame_threading.start()

        # 定期发送检测结果
        self.publisher_ = self.create_publisher(Robots, 'detect_result', qos_profile)
        # 用于发布检测图像结果节点
        self.pub_res = self.create_publisher(Image, 'detect_view', qos_profile)
        
        # 创建线程用于推理
        self.threading = threading.Thread(target=self.infer_loop)
        self.threading.start()
        self.init_flag = False
        
        # 创建一个图像队列
        self.image_queue = deque(maxlen=10)
        self.timestamp_queue = deque(maxlen=10)
        # 创建一个线程用于保存图像，录制文件存放在 record/ 文件夹下
        if self.is_record:
            os.makedirs(RECORD_DIR, exist_ok=True)
            cur_date = time.strftime("%Y-%m-%d-%h-%s", time.localtime())
            self.video_writer = cv2.VideoWriter(os.path.join(RECORD_DIR, cur_date + ".avi"), cv2.VideoWriter_fourcc(*'XVID'), 60, (3072, 2048))
            # 打开文件用于保存时间戳
            self.timestamp_file = open(os.path.join(RECORD_DIR, cur_date + ".txt"), "w")
            self.save_thread = threading.Thread(target=self.save_image)
            self.save_thread.start()

    # ── rosbag 模式：压缩图像回调 ──
    def _compressed_image_callback(self, msg):
        """从 /compressed_image topic 接收压缩图像并解码为 BGR numpy 数组"""
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                with self._frame_lock:
                    self.frame = cv_image
        except Exception as e:
            self.get_logger().error(f'解码压缩图像失败: {e}')

    # 保存图像函数
    def save_image(self):
        while rclpy.ok():
            if self.is_record:
                if len(self.image_queue) > 0 and len(self.timestamp_queue) > 0:
                    img = self.image_queue.popleft()
                    timestp = self.timestamp_queue.popleft()
                    
                    self.video_writer.write(img)
                    self.timestamp_file.write(str(timestp) + "\n")
                    
                    # 刷新
                    self.timestamp_file.flush()
                    time.sleep(1/self.record_fps)
            else:
                time.sleep(0.1)

    # 同步图像函数（仅 hik 模式使用）
    def sync_frame(self):
        while rclpy.ok():
            cam_frame = self.cam.getFrame()
            self.image_queue.append(cam_frame)
            self.timestamp_queue.append(time.time())
            # 加锁防止多线程同时访问
            with self._frame_lock:
                self.frame = cam_frame.copy()

    # 获取图像函数
    def getFrame(self):
        with self._frame_lock:
            if self.frame is None:
                return None
            else:
                return self.frame

    # 画出result
    def draw_result(self, frame, box, result, conf):
        if result != "NULL":
            # 画上分类结果
            x, y, w, h = box
            cv2.putText(frame, str(result), (int(box[0] - 5), int(box[1] - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 122), 2)
            # 画上置信度 debug
            cv2.putText(frame, str(round(conf, 2)), (int(box[0] - 5), int(box[1] - 25)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 122), 2)
            cv2.rectangle(frame, (int(x - w / 2), int(y - h / 2)), (int(x + w / 2), int(y + h / 2)),
                          (0, 255, 122), 5)
        return frame

    # 推理循环函数
    def infer_loop(self):
        cv2.namedWindow("Window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Window", self.display_w, self.display_h)
        # 原图分辨率（首帧自动检测，不再硬编码 3072×2048）
        orig_w, orig_h = None, None
        while rclpy.ok():
            now = time.time()
            fps = 1 / (now - self.start_time)
        
            self.start_time = now

            try:
                # 获取缓存的图像
                cv_image = self.getFrame()
                if cv_image is not None:
                    cv_image = cv_image.copy()
                else:
                    continue

                # 首帧自动检测原图分辨率
                if orig_w is None:
                    orig_h, orig_w = cv_image.shape[:2]
                    self.get_logger().info(
                        f'原图分辨率: {orig_w}×{orig_h}, '
                        f'推理分辨率: {self.infer_w}×{self.infer_h}')

                cv_image = cv2.resize(cv_image, (self.infer_w, self.infer_h))
                
                cv2.waitKey(1)
                allRobots = Robots()
                # Inference — 使用 pipeline
                if cv_image is not None:
                    infer_result = self.pipeline.infer(cv_image)
                
                    if infer_result is not None:
                        result_img, results = infer_result
                        if results is not None:
                            for result in results:
                                msg = DetectResult()
                                # 将推理分辨率坐标还原到原图分辨率
                                result[0][0] = int(result[0][0] * orig_w / self.infer_w)
                                result[0][1] = int(result[0][1] * orig_h / self.infer_h)
                                result[0][2] = int(result[0][2] * orig_w / self.infer_w)
                                result[0][3] = int(result[0][3] * orig_h / self.infer_h)

                                result[1][0] = int(result[1][0] * orig_w / self.infer_w)
                                result[1][1] = int(result[1][1] * orig_h / self.infer_h)
                                result[1][2] = int(result[1][2] * orig_w / self.infer_w)
                                result[1][3] = int(result[1][3] * orig_h / self.infer_h)

                                msg.xyxy_box = result[0]
                                msg.xywh_box = [float(x) for x in result[1]]
                                msg.track_id = result[2]
                                msg.label = result[3]
                                allRobots.detect_results.append(msg)

                        # 发布检测可视化图像到 detect_view
                        if result_img is not None:
                            img_msg = self.bridge.cv2_to_imgmsg(result_img, encoding='bgr8')
                            self.pub_res.publish(img_msg)
                            cv2.imshow("Window", result_img)
                        else:
                            img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                            self.pub_res.publish(img_msg)
                            cv2.imshow("Window", cv_image)
                        cv2.waitKey(1)
                else:
                    cv2.imshow("Window", cv_image)

                self.publisher_.publish(allRobots)
        
            except Exception as e:
                self.get_logger().error('Error {0}'.format(e))
    
    def __del__(self):
        # Clean Up
        self.pipeline = None
        self.threading = None
        self.init_flag = None
        self.get_logger().info('Detector node is being destroyed.')


def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

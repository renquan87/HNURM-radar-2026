"""
detector_node.py — 激光雷达方案的视觉检测节点（方案二）

功能：
  调用海康工业相机获取图像，使用 YOLO 三阶段推理识别赛场机器人，
  将检测结果（bbox + track_id + 分类标签 + 置信度 + 时间戳）通过 ROS2 话题发布给
  下游的 radar_node 进行点云投影定位。

三阶段推理流程：
  Stage 1 — track_infer():  全图目标检测 + bot-sort 多目标追踪（stage_one.pt, imgsz=1280）
  Stage 2 — classify_infer(): ROI 装甲板颜色+编号分类（stage_two.pt, imgsz=256）
  Stage 3 — classify_infer(): ROI 灰色装甲板专用分类（stage_three.pt, imgsz=256）

与 camera_detector（方案一）的区别：
  - 本节点不做透视变换定位，仅输出 2D 检测框 + 原始分类标签 + 置信度
  - 定位由下游 radar_node 结合点云完成（3D 投影 + DBSCAN 聚类）
  - 直接依赖海康工业相机硬件（HKCam），无 test/video 模式

发布话题：
  - detect_result (Robots)   — 所有检测到的机器人 bbox + 标签 + 置信度 + 时间戳
  - detect_view   (Image)    — 带标注的检测结果图像

配置文件：
  - configs/detector_config.yaml — 模型路径、置信度阈值等
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
from ruamel.yaml import YAML

from detect_result.msg import DetectResult
from detect_result.msg import Robots
from sensor_msgs.msg import Image
from ..Camera.HKCam import *
from cv_bridge import CvBridge
import cv2
import time
import math
import threading
import sys
import os
from collections import deque
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ..shared.paths import PROJECT_ROOT, DETECTOR_CONFIG_PATH, RECORD_DIR, ULTRALYTICS_DIR, resolve_path
sys.path.append(ULTRALYTICS_DIR)
from ultralytics import YOLO

class Detector(Node):
    def __init__(self):
        super().__init__('detector')
        
        self.start_time = time.time()
        self.bridge = CvBridge()
        # 加载配置文件
        self.cfg = YAML().load(open(DETECTOR_CONFIG_PATH, encoding='Utf-8', mode='r'))
        # 记录相关参数 flag and fps
        self.is_record = self.cfg['is_record']
        self.record_fps = self.cfg['record_fps']
        
        # 初始化并加载 3 个 YOLO 模型
        self.get_logger().info('Loading Yolo models...')
        self.model_car = YOLO(resolve_path(self.cfg['path']['stage_one_path']), task="detect")
        self.model_car.overrides['imgsz'] = 1280
        self.model_car2 = YOLO(resolve_path(self.cfg['path']['stage_two_path']))
        self.model_car2.overrides['imgsz'] = 256
        self.model_car3 = YOLO(resolve_path(self.cfg['path']['stage_three_path']))
        self.model_car3.overrides['imgsz'] = 256
        self.get_logger().info('Yolo models loaded.')

        # 读取追踪器路径和各阶段置信度阈值
        self.tracker_path = resolve_path(self.cfg['path']['tracker_path'])
        self.stage_one_conf = self.cfg['params']['stage_one_conf']
        self.stage_two_conf = self.cfg['params']['stage_two_conf']
        self.stage_three_conf = self.cfg['params']['stage_three_conf']
        
        self.labels = self.cfg['params']['labels']  # 类别标签列表 ["B1","B2",...,"R7"]
        self.class_num = len(self.labels)
        
        # 灰色装甲板的类别映射（用于将灰色装甲板分类结果映射到标准兵种）
        self.Gray2Blue = {12:5,13:1,14:0,15:3,16:2,17:4}
        self.Gray2Red = {12:11,13:7,14:6,15:9,16:8,17:10}
        self.gray2gray = {0:14,1:13,2:16,3:15,4:17,5:12}
        self.Blue2Gray = {v: k for k, v in self.Gray2Blue.items()}
        self.Red2Gray = {v: k for k, v in self.Gray2Red.items()}
        
        # QoS 设置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        # 打开相机
        self.cam = HKCam(0)
        self.frame = None
        # 启动图像同步线程
        self.frame_threading = threading.Thread(target=self.sync_frame)
        self.frame_threading.start()
        # 发布检测结果
        self.publisher_ = self.create_publisher(Robots, 'detect_result', qos_profile)
        # 发布可视化图像
        self.pub_res = self.create_publisher(Image, 'detect_view', qos_profile)
        
        # 图像资源锁
        self._frame_lock = threading.Lock()
        # 推理线程
        self.threading = threading.Thread(target=self.infer_loop)
        self.threading.start()
        
        # 图像录制队列
        self.image_queue = deque(maxlen=10)
        self.timestamp_queue = deque(maxlen=10)
        os.makedirs(RECORD_DIR, exist_ok=True)
        cur_date = time.strftime("%Y-%m-%d-%h-%s", time.localtime())
        self.video_writer = cv2.VideoWriter(os.path.join(RECORD_DIR, cur_date + ".avi"), cv2.VideoWriter_fourcc(*'XVID'), 60, (3072, 2048))
        self.timestamp_file = open(os.path.join(RECORD_DIR, cur_date + ".txt"), "w")
        self.save_thread = threading.Thread(target=self.save_image)
        self.save_thread.start()

    def save_image(self):
        """录制图像和 timestamp 的线程"""
        while rclpy.ok():
            if self.is_record:
                if len(self.image_queue) > 0 and len(self.timestamp_queue) > 0:
                    img = self.image_queue.popleft()
                    timestp = self.timestamp_queue.popleft()
                    self.video_writer.write(img)
                    self.timestamp_file.write(str(timestp) + "\n")
                    self.timestamp_file.flush()
                    time.sleep(1/self.record_fps)
            else:
                time.sleep(0.1)

    def sync_frame(self):
        """相机图像获取线程"""
        while rclpy.ok():
            cam_frame = self.cam.getFrame()
            self.image_queue.append(cam_frame)
            self.timestamp_queue.append(time.time())
            with self._frame_lock:
                self.frame = cam_frame.copy()

    def getFrame(self):
        with self._frame_lock:
            if self.frame is None:
                return None
            else:
                return self.frame

    def is_results_empty(self, results):
        if results is None:
            return True
        if results[0].boxes.id is None:
            return True
        return False

    def parse_results(self, results):
        confidences = results[0].boxes.conf.cpu().numpy()
        boxes = results[0].boxes.xywh.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        return confidences, boxes, track_ids
    
    def track_infer(self, frame):
        results = self.model_car.track(frame, persist=True, tracker=self.tracker_path, verbose=False)
        return results
    
    def classify_infer(self, roi_list):
        """
        对 ROI 列表进行二阶段（装甲板分类）和三阶段（灰色装甲板分类）推理。
        返回:
            label_list: 整数类别索引列表（-1 表示无效，0~11 为有效兵种）
            conf_list:  对应置信度列表
        """
        if not roi_list:
            return [], []
        
        results = []
        gray_results = []
        for roi in roi_list:
            results.extend(self.model_car2.predict(roi, conf=self.stage_two_conf, device=0, verbose=False))
            gray_results.extend(self.model_car3.predict(roi, device=0, verbose=False))
        
        if len(results) == 0:
            # 没有任何分类结果，全部返回无效
            return [-1] * len(roi_list), [0.0] * len(roi_list)
        
        label_list = []
        conf_list = []
        for result, gray_result, roi in zip(results, gray_results, roi_list):
            roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            data = result.boxes.data
            gray_data = gray_result.boxes.data
            
            maxConf = 0
            maxGrayConf = 0
            label = -1
            gray_label = -1
            gray_tmp = []
            tmp = []
            
            for i in range(len(gray_data)):
                if gray_data[i][4] > maxGrayConf:
                    maxGrayConf = gray_data[i][4]
                    gray_label = int(gray_data[i][5])
                    gray_tmp = gray_data[i]
            
            for i in range(len(data)):
                if data[i][4] > maxConf:
                    maxConf = data[i][4]
                    label = int(data[i][5])
                    tmp = data[i]
            
            if len(tmp) != 0:
                x1, y1, x2, y2 = map(int, tmp[:4])
                gray_roi = roi_gray[y1:y2, x1:x2]
                if label < 6 and cv2.mean(gray_roi)[0] < 35:   # 蓝色装甲板变灰
                    label = self.Blue2Gray.get(label, label)
                elif label > 5 and cv2.mean(gray_roi)[0] < 15: # 红色装甲板变灰
                    label = self.Red2Gray.get(label, label)
            else:
                if len(gray_tmp) != 0:
                    x1, y1, x2, y2 = map(int, gray_tmp[:4])
                    gray_roi = roi_gray[y1:y2, x1:x2]
                    if cv2.mean(gray_roi)[0] < 30:
                        label = self.gray2gray.get(int(gray_label), -1)
            
            label_list.append(label)
            conf_list.append(float(maxConf))
        
        return label_list, conf_list

    def infer(self, frame):
        """主推理流程，返回 (标注后的图像, 结果列表)"""
        if frame is None:
            return None, None
        
        results = self.track_infer(frame)
        if self.is_results_empty(results):
            return frame, None
        
        confidences, boxes, track_ids = self.parse_results(results)
        
        roi_list = []
        id_list = []
        box_list = []
        
        for box, track_id, conf in zip(boxes, track_ids, confidences):
            x, y, w, h = box
            x_left = int(x - w / 2)
            y_left = int(y - h / 2)
            roi = frame[y_left:y_left+int(h), x_left:x_left+int(w)]
            roi_list.append(roi)
            id_list.append(track_id)
            box_list.append(box)
        
        label_list, conf_list = self.classify_infer(roi_list)
        
        zip_results = []
        draw_candidate = []   # [track_id, x1, y1, x2, y2, label_str]
        
        for i, (box, track_id) in enumerate(zip(box_list, id_list)):
            class_idx = label_list[i]
            if class_idx == -1:
                label_str = "NULL"
                confidence = 0.0
            else:
                label_str = self.labels[class_idx]   # 如 "B1"
                confidence = conf_list[i]
            
            x, y, w, h = box
            x_left = int(x - w / 2)
            y_left = int(y - h / 2)
            x_right = int(x + w / 2)
            y_right = int(y + h / 2)
            xyxy_box = [x_left, y_left, x_right, y_right]
            xywh_box = [x, y, w, h]
            
            zip_results.append([xyxy_box, xywh_box, track_id, label_str, confidence])
            draw_candidate.append([track_id, x_left, y_left, x_right, y_right, label_str])
        
        # 绘制所有检测框和标签
        for item in draw_candidate:
            _, x1, y1, x2, y2, lab = item
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 128, 0), 3, 8)
            cv2.putText(frame, lab, (x1, y1-5), cv2.FONT_HERSHEY_SIMPLEX, 1.5,
                        (0, 255, 122), 2)
            cv2.putText(frame, str(item[0]), (x2+5, y2+5), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (0, 255, 122), 2)
        
        return frame, zip_results

    def infer_loop(self):
        cv2.namedWindow("Window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Window", 1920, 1080)
        while rclpy.ok():
            now = time.time()
            fps = 1 / (now - self.start_time)
            self.start_time = now
            
            try:
                cv_image = self.getFrame()
                if cv_image is not None:
                    cv_image = cv_image.copy()
                else:
                    continue
                cv_image = cv2.resize(cv_image, (1920, 1080))
                cv2.waitKey(1)
                
                allRobots = Robots()
                infer_result = self.infer(cv_image)
                if infer_result is not None:
                    result_img, results = infer_result
                    if results is not None:
                        for res in results:
                            # res = [xyxy, xywh, track_id, label_str, confidence]
                            msg = DetectResult()
                            # 将可视化尺寸（1920x1080）的框坐标还原到原始图像尺寸（3072x2048）
                            msg.xyxy_box = [
                                int(res[0][0] * 3072 / 1920),
                                int(res[0][1] * 2048 / 1080),
                                int(res[0][2] * 3072 / 1920),
                                int(res[0][3] * 2048 / 1080)
                            ]
                            msg.xywh_box = [
                                float(res[1][0] * 3072 / 1920),
                                float(res[1][1] * 2048 / 1080),
                                float(res[1][2] * 3072 / 1920),
                                float(res[1][3] * 2048 / 1080)
                            ]
                            msg.track_id = res[2]
                            msg.label = res[3]          # 原始分类字符串
                            msg.confidence = float(res[4])
                            msg.stamp = self.get_clock().now().to_msg()
                            allRobots.detect_results.append(msg)
                    
                    # 发布可视化图像
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
                self.get_logger().error(f'Error: {e}')
    
    def __del__(self):
        self.get_logger().info('Detector node is being destroyed.')

def main(args=None):
    rclpy.init(args=args)
    node = Detector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""
display_panel.py — 小地图可视化面板节点

功能：
  订阅 EKF 滤波后的机器人位置信息（/ekf_location_filtered），
  在标准赛场地图（std_map.png）上实时绘制敌我双方机器人位置，
  以 ~15fps 刷新 OpenCV 窗口显示，并通过 /map_view 话题发布渲染结果
  供 Foxglove / 远程 RViz 查看。

  当收到 EKF 诊断数据（/ekf_diagnostics）时，使用协方差椭圆代替固定
  半径圆圈来表示每个目标的定位不确定性，精确坐标用小实心点标注。

坐标映射：
  赛场坐标 (m) → 像素坐标：x_px = x * 100, y_px = MAP_PX_H - y * 100
  红方坐标需做对称翻转（FIELD_WIDTH-x, FIELD_HEIGHT-y）以统一到蓝方视角

订阅话题：
  - /ekf_location_filtered (Locations) — EKF 滤波后的机器人位置
  - /ekf_diagnostics (EkfDiagnosticsArray) — EKF 诊断指标（协方差椭圆等）

发布话题：
  - /map_view (sensor_msgs/Image) — 渲染后的小地图画面（BGR8）

依赖：
  - map/std_map.png — 标准赛场底图（FIELD_WIDTH*100 × FIELD_HEIGHT*100 像素）
"""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import Image as SensorImage
from detect_result.msg import Location, Locations
from detect_result.msg import EkfDiagnosticsArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from cv_bridge import CvBridge
import threading
import time
import cv2
import numpy as np


# 协方差椭圆缩放因子: 3σ 表示 99.7% 置信区间
_COV_SIGMA = 3.0
# 协方差椭圆最小/最大半轴像素值（防止太小看不见或太大遮挡）
_COV_MIN_PX = 8
_COV_MAX_PX = 200
# 地图坐标到像素的缩放比 (像素/米)
_M_TO_PX = 100


class DisplayPanel(Node):

    def __init__(self):
        super().__init__('display_panel')
        self.get_logger().info('Display Panel Node is running')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        self.locations = Locations()
        from .paths import STD_MAP_PATH, FIELD_WIDTH, FIELD_HEIGHT
        # 地图像素尺寸：赛场米坐标 × 100
        self._map_px_w = int(FIELD_WIDTH * 100)   # e.g. 2800
        self._map_px_h = int(FIELD_HEIGHT * 100)   # e.g. 1500
        self._field_w = FIELD_WIDTH
        self._field_h = FIELD_HEIGHT
        self.map = cv2.imread(STD_MAP_PATH)
        
        self.sub_location = self.create_subscription(Locations, "/ekf_location_filtered", self.location_callback, qos_profile)

        # 订阅 EKF 诊断数据（用于协方差椭圆绘制）
        diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        self._diag_data = None   # 最新的诊断数据
        self._diag_lock = threading.Lock()
        self.sub_diagnostics = self.create_subscription(
            EkfDiagnosticsArray, "/ekf_diagnostics",
            self._diagnostics_callback, diag_qos)

        # 发布渲染后的小地图画面，供 Foxglove / 远程 RViz 查看
        # 使用 RELIABLE QoS 确保 foxglove_bridge 能订阅到
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_map_view = self.create_publisher(SensorImage, '/map_view', pub_qos)
        self._cv_bridge = CvBridge()
        # 控制 /map_view 发布频率（每 N 帧发布一次，降低 Foxglove WebSocket 带宽压力）
        self._map_pub_counter = 0
        self._MAP_PUB_EVERY_N = 5  # 约 15fps / 5 = 3fps
        # 创建一个锁
        self.lock = threading.Lock()
        
        cv2.namedWindow('map', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('map', 800, 471)

    def _get_slot_cov(self, robot_id: int):
        """根据 robot_id 从诊断数据中查找对应 slot 的协方差。

        Returns:
            (cov_xx, cov_yy) 或 None（未找到）
        """
        with self._diag_lock:
            diag = self._diag_data
        if diag is None:
            return None
        for slot in diag.slots:
            if slot.robot_id == robot_id and (slot.cov_xx > 0 or slot.cov_yy > 0):
                return (slot.cov_xx, slot.cov_yy)
        return None

    def _draw_cov_ellipse(self, img, cx, cy, cov_xx, cov_yy, color, thickness=2):
        """在图像上绘制协方差椭圆 + 中心精确位置点。

        Args:
            img: OpenCV 图像
            cx, cy: 中心像素坐标
            cov_xx: X 方向方差 (m²)
            cov_yy: Y 方向方差 (m²)
            color: BGR 颜色元组
            thickness: 椭圆线条粗细
        """
        # 3σ 椭圆半轴，方差→标准差→像素
        axis_x = int(_COV_SIGMA * math.sqrt(max(cov_xx, 0)) * _M_TO_PX)
        axis_y = int(_COV_SIGMA * math.sqrt(max(cov_yy, 0)) * _M_TO_PX)
        # 限制范围
        axis_x = max(_COV_MIN_PX, min(axis_x, _COV_MAX_PX))
        axis_y = max(_COV_MIN_PX, min(axis_y, _COV_MAX_PX))
        # 绘制椭圆
        cv2.ellipse(img, (cx, cy), (axis_x, axis_y), 0, 0, 360, color, thickness)
        # 中心精确位置点
        cv2.circle(img, (cx, cy), 5, color, -1)

    def _draw_robot_marker(self, img, xx, yy, robot_id, color, is_air, z=0.0):
        """绘制机器人标记：地面用协方差椭圆/圆圈，空中用菱形。"""
        if is_air:
            # 空中机器人用菱形标记
            pts = np.array([[xx, yy-50], [xx+50, yy], [xx, yy+50], [xx-50, yy]], np.int32)
            cv2.polylines(img, [pts], True, color, 3)
            cv2.circle(img, (xx, yy), 5, color, -1)
            cv2.putText(img, f"h={z:.1f}m", (xx + 55, yy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            # 地面机器人：尝试用协方差椭圆，否则回退到固定圆圈
            cov = self._get_slot_cov(robot_id)
            if cov is not None:
                self._draw_cov_ellipse(img, xx, yy, cov[0], cov[1], color)
            else:
                cv2.circle(img, (xx, yy), 60, color, 4)

    def display_locations(self):
        show_map = self.map.copy()

        cur_locations = []
        with self.lock:
            cur_locations = self.locations

        for i in range(len(cur_locations.locs)):
            location = cur_locations.locs[i]
            x = round(location.x, 2)
            y = round(location.y, 2)
            z = round(location.z, 2)
            
            xx = int(x * 100)
            yy = int(self._map_px_h - y * 100)
                
            # 判断是否为空中机器人：正式ID(6/106) + 调试多目标ID段(600~699/1600~1699)
            # 注意：camera_detector 中 NULL 标签使用 9000+track_id 作为临时 ID，不应视为空中机器人
            is_air_robot = location.id in (6, 106) or (600 <= location.id < 700) or (1600 <= location.id < 1700)
            air_suffix = " UAV" if is_air_robot else ""
            
            if location.label == 'Red':
                x = self._field_w - x
                y = self._field_h - y
                xx = self._map_px_w - xx
                yy = self._map_px_h - yy
                color = (0, 0, 255)  # 红色
                cv2.putText(show_map, str(location.id) + air_suffix, (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 4)
                cv2.putText(show_map, str((x)) + ',' + str((y)) + ',' + str(z), (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
                self._draw_robot_marker(show_map, xx, yy, location.id, color, is_air_robot, z)
            elif location.label == 'Blue':
                color = (255, 0, 0)  # 蓝色
                cv2.putText(show_map, str(location.id) + air_suffix, (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 4)
                cv2.putText(show_map, str((x)) + ',' + str((y)) + ',' + str(z), (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
                self._draw_robot_marker(show_map, xx, yy, location.id, color, is_air_robot, z)
            else:
                # 颜色标签为 null/未知的机器人，灰色显示，不做坐标对称变换
                cv2.putText(show_map, 'null', (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, (128, 128, 128), 4)
                cv2.putText(show_map, str((x)) + ',' + str((y)) + ',' + str(z), (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
                cv2.circle(show_map, (xx, yy), 60, (128, 128, 128), 4)
        cv2.imshow('map', show_map)
        cv2.waitKey(16)

        # 发布小地图到 /map_view 话题（降频 + 缩小，减轻 Foxglove WebSocket 带宽压力）
        self._map_pub_counter += 1
        if self._map_pub_counter >= self._MAP_PUB_EVERY_N:
            self._map_pub_counter = 0
            try:
                # 缩小到一半分辨率以降低带宽（2800×1500 → 1400×750，约 3.15MB/帧）
                small = cv2.resize(show_map, (show_map.shape[1] // 2, show_map.shape[0] // 2),
                                   interpolation=cv2.INTER_AREA)
                img_msg = self._cv_bridge.cv2_to_imgmsg(small, encoding='bgr8')
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'map'
                self.pub_map_view.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f'发布小地图失败: {e}', throttle_duration_sec=5.0)
        
    def location_callback(self, msg):
        # 更新locations
        with self.lock:
            self.locations = msg

    def _diagnostics_callback(self, msg):
        """接收 EKF 诊断数据，用于协方差椭圆绘制。"""
        with self._diag_lock:
            self._diag_data = msg

def spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    disPanel = DisplayPanel()
    
    # 创建一个线程来读取节点内容
    thread = threading.Thread(target=spin_thread, args=(disPanel,))
    thread.start()
    
    try:
        while rclpy.ok():
            disPanel.display_locations()
            time.sleep(1/15)  # 控制显示频率
    except KeyboardInterrupt:
        pass
    
    disPanel.destroy_node()
    rclpy.shutdown()
    thread.join()

if __name__ == '__main__':
    main()

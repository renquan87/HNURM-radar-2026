"""
display_panel.py — 小地图可视化面板节点

功能：
  订阅 EKF 滤波后的机器人位置信息（/ekf_location_filtered），
  在标准赛场地图（std_map.png）上实时绘制敌我双方机器人位置，
  以 ~15fps 刷新 OpenCV 窗口显示，并通过 /map_view 话题发布渲染结果
  供 Foxglove / 远程 RViz 查看。

  显示规则：
    - 红色机器人 → 红色圆圈
    - 蓝色机器人 → 蓝色圆圈
    - 空中机器人（id 6/106 等）→ 绿色菱形
    - 坐标文本保留 1 位小数

坐标映射：
  赛场坐标 (m) → 像素坐标：x_px = x * 100, y_px = MAP_PX_H - y * 100
  配准初始位姿已根据 my_color 正确设置，输出即为裁判系统坐标，无需翻转。

订阅话题：
  - /ekf_location_filtered (Locations) — EKF 滤波后的机器人位置

发布话题：
  - /map_view (sensor_msgs/Image) — 渲染后的小地图画面（BGR8）

依赖：
  - map/std_map.png — 标准赛场底图（FIELD_WIDTH*100 × FIELD_HEIGHT*100 像素）
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from detect_result.msg import Location, Locations
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
import threading
import time
import cv2
import numpy as np


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

    def _draw_robot_marker(self, img, xx, yy, color, is_air, z=0.0):
        """绘制机器人标记：地面用圆圈，空中用绿色菱形。"""
        if is_air:
            # 空中机器人统一用绿色菱形标记
            air_color = (0, 255, 0)  # 绿色
            pts = np.array([[xx, yy-50], [xx+50, yy], [xx, yy+50], [xx-50, yy]], np.int32)
            cv2.polylines(img, [pts], True, air_color, 3)
            cv2.circle(img, (xx, yy), 5, air_color, -1)
            cv2.putText(img, f"h={z:.1f}m", (xx + 55, yy + 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        else:
            # 地面机器人用简单圆圈
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

            # 配准初始位姿已根据 my_color 正确设置，坐标直接就是裁判系统坐标，
            # Red/Blue 统一走同一坐标映射路径，无需翻转。
            if location.label == 'Red':
                color = (0, 0, 255)  # 红色 BGR
            elif location.label == 'Blue':
                color = (255, 0, 0)  # 蓝色 BGR
            else:
                color = (128, 128, 128)  # 灰色

            # 坐标文本保留 1 位小数
            coord_text = f"{x:.1f},{y:.1f}"
            label_text = str(location.id) + air_suffix if location.label in ('Red', 'Blue') else 'null'
            cv2.putText(show_map, label_text, (xx - 15, yy + 10), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 4)
            cv2.putText(show_map, coord_text, (xx, yy - 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 4)
            if location.label in ('Red', 'Blue'):
                self._draw_robot_marker(show_map, xx, yy, color, is_air_robot, z)
            else:
                cv2.circle(show_map, (xx, yy), 60, color, 4)
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

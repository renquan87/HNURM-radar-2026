#!/usr/bin/env python3
"""
ekf_node.py — 扩展卡尔曼滤波（EKF）节点

功能：
  订阅检测节点发布的敌方机器人赛场坐标（location 话题），
  对每辆敌方机器人独立运行 EKF 滤波，平滑位置并估计速度，
  将滤波后的坐标通过 ekf_location_filtered 话题发布。
  同时发布 EKF 诊断指标（检测频率、新息、jitter 等）供调试使用。

数据流：
  detector_node → [location] → ekf_node → [ekf_location_filtered] → judge_messager
                                        → [ekf_diagnostics]        → Foxglove / display_panel

订阅话题：
  - location (Locations) — 原始检测坐标

发布话题：
  - ekf_location_filtered (Locations) — 滤波后坐标
  - ekf_diagnostics (EkfDiagnosticsArray) — EKF 诊断指标

核心逻辑：
  - 维护 7 个独立的 RobotEKF 实例（对应敌方 7 辆机器人）
  - 每 50ms 定时触发一次滤波步进
  - 状态向量 [x, vx, y, vy]，观测向量同维
  - 通过 RobotInfo 计算帧间速度和加速度，作为过程模型输入
  - 诊断指标在每次滤波步进后计算并发布

依赖：
  - ekf.RobotEKF — 基于 tinyekf 的 EKF 实现
  - detect_result.msg — Location / Locations / EkfDiagnostics 自定义消息
"""

import math
import os
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from detect_result.msg import Location, Locations
from detect_result.msg import DetectResult
from detect_result.msg import Robots
from detect_result.msg import EkfDiagnostics, EkfDiagnosticsArray
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ekf.RobotEKF import RobotEKF

# ---------- 读取 main_config.yaml ----------
# 优先从 hnurm_radar 的统一路径管理模块获取配置文件路径；
# 若跨包导入失败（如单独测试 ekf 包），则回退到 __file__ 拼接。
try:
    from hnurm_radar.shared.paths import MAIN_CONFIG_PATH as _MAIN_CONFIG
except ImportError:
    _THIS_DIR = os.path.dirname(os.path.abspath(__file__))
    _PROJECT_ROOT = os.path.normpath(os.path.join(_THIS_DIR, '..', '..', '..'))
    _MAIN_CONFIG = os.path.join(_PROJECT_ROOT, 'configs', 'main_config.yaml')

def _load_debug_flag() -> bool:
    """从 main_config.yaml 读取 global.debug_coordinate_publish 配置。
    缺省值 False（赛场安全）。"""
    try:
        import yaml
        with open(_MAIN_CONFIG, 'r', encoding='utf-8') as f:
            cfg = yaml.safe_load(f)
        return bool(cfg.get('global', {}).get('debug_coordinate_publish', False))
    except Exception:
        return False

# 机器人信息类
class RobotInfo(object):

    def __init__(self, time, x, y, z):
        self.x, self.y, self.z = x, y, z
        self.time = time
        self.v_x = 0
        self.v_y = 0
        self.last_v_x = 0 # 上一次的速度
        self.last_v_y = 0 # 上一次的速度
        self.a_x = 0   # V1 = V0 + at   -> a = (v1-v0)/t
        self.a_y = 0

    def __str__(self):
        return '%4d %4d %4d %4d' % (self.time, self.x, self.y, self.z)
    
    # 计算速度和加速度
    def calculateInfo(self, last_info):

        if last_info.x > 0 and last_info.y > 0:
            t = (self.time - last_info.time) # 转s

            if abs(t) < 1e-6:  # 避免除零
                return

            self.v_x = (self.x - last_info.x) / t
            self.v_y = (self.y - last_info.y) / t

            self.a_x = (self.v_x - last_info.v_x) / t
            self.a_y = (self.v_y - last_info.v_y) / t


# ── 诊断相关常量 ──
NUM_SLOTS = 14                   # 14 个 EKF 滤波器 slot（7 红方 + 7 蓝方）
DETECTION_WINDOW_SEC = 2.0       # 检测频率计算的滑动窗口 (秒)
DETECTION_WINDOW_MS = DETECTION_WINDOW_SEC * 1000.0


# EKF节点类
class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # QoS 设置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 记录敌方颜色
        self.eneny_color = 'NULL'
        # ── 调试坐标发布开关（从 main_config.yaml 读取）──
        # True  = 调试: 所有检测坐标都发布（空中ID映射到6/106, 未知ID透传）
        # False = 赛场: 仅标准ID (1-7/101-107) 经EKF发布, 其余丢弃
        self.debug_coordinate_publish = _load_debug_flag()
        self.get_logger().info(
            f'📋 debug_coordinate_publish = {self.debug_coordinate_publish} '
            f'{"(调试模式: 全部坐标发布)" if self.debug_coordinate_publish else "(赛场模式: 仅标准ID)"}')
        # 机器人ID映射到数组下标（1-based）
        # [测试] 红方和蓝方使用独立 slot，避免己方/敌方同时进入时 slot 冲突
        # 红方 slot 1-7:  地面 1-5 → slot 1-5, 哨兵 7 → slot 6, 空中 6 → slot 7
        # 蓝方 slot 8-14: 地面 101-105 → slot 8-12, 哨兵 107 → slot 13, 空中 106 → slot 14
        self.transform_to_th = {
            1: 1,
            2: 2,
            3: 3,
            4: 4,
            5: 5,
            6: 7,       # 红方空中机器人 → slot 7
            7: 6,       # 红方哨兵 → slot 6
            101: 8,
            102: 9,
            103: 10,
            104: 11,
            105: 12,
            106: 14,    # 蓝方空中机器人 → slot 14
            107: 13,    # 蓝方哨兵 → slot 13
        }
        # slot 索引到 (robot_id, label) 的反向映射
        self._slot_to_robot = {
            0: (1, 'Red'), 1: (2, 'Red'), 2: (3, 'Red'), 3: (4, 'Red'), 4: (5, 'Red'),
            5: (7, 'Red'), 6: (6, 'Red'),
            7: (101, 'Blue'), 8: (102, 'Blue'), 9: (103, 'Blue'), 10: (104, 'Blue'), 11: (105, 'Blue'),
            12: (107, 'Blue'), 13: (106, 'Blue'),
        }
        # 初始化14个机器人的EKF滤波器（红蓝各7个），修改噪声参数q，r减小突变对数据的影响
        self.kalfilt = [
            RobotEKF(4, 4, pval=0.001, qval=1e-4, rval=0.0005, interval=0.05) for _ in range(NUM_SLOTS)
        ]

        # 创建发布者和订阅者
        self.pub_location_filtered = self.create_publisher(Locations, 'ekf_location_filtered', qos_profile)
        self.sub_location = self.create_subscription(Locations, "location", self.location_callback, qos_profile)

        # ── 诊断话题发布者（使用 RELIABLE QoS 确保 Foxglove 能接收）──
        diag_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        self.pub_diagnostics = self.create_publisher(
            EkfDiagnosticsArray, 'ekf_diagnostics', diag_qos)

        # 初始化变量
        self.recv_location = Locations()
        self.last_location = Locations()
        # 创建定时器
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50 ms
        
        # 初始化位置队列，保存上次和当前的机器人位置信息 [0]是上次的， [1]是当前的
        self.locations_queue = [
            [RobotInfo(0, 0, 0, 0) for _ in range(NUM_SLOTS)]
            for _ in range(2)
        ]

        # ── 诊断状态初始化 ──
        # 每个 slot 记录最近检测时间戳（用于频率计算）
        self._det_timestamps: list[deque] = [
            deque(maxlen=100) for _ in range(NUM_SLOTS)
        ]
        # 上次滤波位置（用于 jitter 计算）
        self._last_filtered_pos: list[tuple] = [
            (0.0, 0.0) for _ in range(NUM_SLOTS)
        ]
        # 上次检测时间（用于 time_since_last_det）
        self._last_det_time_ms: list[float] = [0.0] * NUM_SLOTS
        # 当前帧各 slot 是否有新检测（在 timer_callback 中标记）
        self._slot_has_detection: list[bool] = [False] * NUM_SLOTS
        # 当前帧各 slot 的原始检测坐标
        self._slot_raw_xy: list[tuple] = [(0.0, 0.0)] * NUM_SLOTS
        # slot → robot_id 的映射（动态更新）
        self._slot_robot_id: list[int] = [0] * NUM_SLOTS

    # 获取当前时间的毫秒数    
    def get_current_time_ms(self):
        current_time = self.get_clock().now()
        time_msg = current_time.to_msg()
        milliseconds = time_msg.sec * 1000 + time_msg.nanosec / 1e6
        return milliseconds

    # 定时器回调函数    
    def timer_callback(self):
        self.get_logger().info(f"Timestamp: {self.get_clock().now()}, Location: {self.recv_location.__str__()}")
        locations = self.recv_location
        now_ms = self.get_current_time_ms()

        # 重置每帧的检测标记
        for i in range(NUM_SLOTS):
            self._slot_has_detection[i] = False

        # 更新测量值
        for i in range(len(locations.locs)):
            location = locations.locs[i]
            robot_id = location.id
            # air_scheme 非严格模式产生的 ID 范围映射
            if 600 <= robot_id < 1000:
                if not self.debug_coordinate_publish:
                    continue            # 赛场模式: 丢弃, 不发裁判系统
                robot_id = 6            # 调试模式: 映射为红方空中标准 ID
            elif 1600 <= robot_id < 2000:
                if not self.debug_coordinate_publish:
                    continue            # 赛场模式: 丢弃
                robot_id = 106          # 调试模式: 映射为蓝方空中标准 ID
            # camera_scheme NULL 机器人: 9000+ ID → 调试模式下映射到对应颜色的哨兵 slot
            elif 9000 <= robot_id < 10000:
                if not self.debug_coordinate_publish:
                    continue            # 赛场模式: 丢弃
                # 根据 label 判断颜色, 映射到对应哨兵 ID (7/107) 以便经过 EKF 滤波
                if location.label == 'Blue':
                    robot_id = 107
                elif location.label == 'Red':
                    robot_id = 7
                else:
                    continue
            # 跳过 ID 不在映射表中的机器人
            if robot_id not in self.transform_to_th:
                continue
            slot_idx = self.transform_to_th[robot_id] - 1  # 转为 0-based
            x = location.x
            y = location.y
            time = self.get_current_time_ms()
            if self.eneny_color == "NULL" and location.label != 'NULL':
                self.eneny_color = location.label
            self.locations_queue[1][slot_idx].x = x
            self.locations_queue[1][slot_idx].y = y
            self.locations_queue[1][slot_idx].time = time
            self.locations_queue[1][slot_idx].z = location.z

            # ── 诊断: 记录检测事件 ──
            self._slot_has_detection[slot_idx] = True
            self._slot_raw_xy[slot_idx] = (x, y)
            self._det_timestamps[slot_idx].append(now_ms)
            self._last_det_time_ms[slot_idx] = now_ms
            self._slot_robot_id[slot_idx] = robot_id

        estimated_locations = [[0, 0, 0, 0] for _ in range(NUM_SLOTS)]
        # 对每个机器人进行卡尔曼滤波
        for i in range(len(self.locations_queue[0])):
            # 如果没有新的数据,则跳过
            if self.locations_queue[1][i].time == 0:
                continue
            # 计算加速度
            self.locations_queue[1][i].calculateInfo(self.locations_queue[0][i])
            self.kalfilt[i].update_acceleration(self.locations_queue[1][i].a_x, self.locations_queue[1][i].a_y)
            estimated_locations[i] = (self.kalfilt[i].step((self.locations_queue[1][i].x, self.locations_queue[1][i].v_x, self.locations_queue[1][i].y, self.locations_queue[1][i].v_y)))
            
        ## 保存历史位置（每帧更新，用于下一帧速度/加速度计算） 等测试完成之后再考虑优化
        for i in range(len(self.locations_queue[0])):
            # if self.locations_queue[1][i].time > 0:
            if self.locations_queue[1][i].time > 0:
                self.locations_queue[0][i].time = self.locations_queue[1][i].time
                self.locations_queue[0][i].x = self.locations_queue[1][i].x
                self.locations_queue[0][i].y = self.locations_queue[1][i].y
                self.locations_queue[0][i].z = self.locations_queue[1][i].z
                # self.locations_queue[0][i].v_x = self.locations_queue[1][i].v_x # 等测试完成之后再考虑优化
                # self.locations_queue[0][i].v_y = self.locations_queue[1][i].v_y # 等测试完成之后再考虑优化
        self.get_logger().info(f"Estimated locations: {estimated_locations}")
        
        # 发布滤波后的位置信息
        # 使用 _slot_to_robot 反向映射，直接从 slot 索引获取 (robot_id, label)
        locations = Locations()
        for i in range(len(estimated_locations)):
            location = Location()
            if estimated_locations[i][0] == 0 or estimated_locations[i][2] == 0:
                continue
            if i not in self._slot_to_robot:
                continue
            robot_id, label = self._slot_to_robot[i]
            location.id = robot_id
            location.label = label
            location.x = float(estimated_locations[i][0])
            location.y = float(estimated_locations[i][2])
            location.z = float(self.locations_queue[1][i].z)
            locations.locs.append(location)
        # 非标准 ID 处理：仅调试模式下透传, 赛场模式全部丢弃
        if self.debug_coordinate_publish:
            for loc in self.recv_location.locs:
                if loc.id not in self.transform_to_th:
                    # 空中 ID 已在上方映射处理, 此处跳过避免重复
                    if 600 <= loc.id < 1000 or 1600 <= loc.id < 2000:
                        continue
                    locations.locs.append(loc)
        # 通过话题发布
        self.pub_location_filtered.publish(locations)

        # ── 计算并发布诊断指标 ──
        self._publish_diagnostics(estimated_locations, now_ms)

                
    # 订阅位置信息的回调函数
    def location_callback(self, msg):
        # self.get_logger().info(f"Received location: {msg}")
        self.recv_location = msg

    def _publish_diagnostics(self, estimated_locations, now_ms: float):
        """计算并发布所有 slot 的诊断指标。"""
        diag_msg = EkfDiagnosticsArray()
        diag_msg.stamp = self.get_clock().now().to_msg()

        for i in range(NUM_SLOTS):
            d = EkfDiagnostics()
            d.slot_id = i
            # 使用 _slot_to_robot 反向映射获取 robot_id
            if i in self._slot_to_robot:
                d.robot_id = self._slot_to_robot[i][0]
            else:
                d.robot_id = self._slot_robot_id[i]

            # ── 检测频率 ──
            # 清理滑动窗口中过期的时间戳
            window_start = now_ms - DETECTION_WINDOW_MS
            while self._det_timestamps[i] and self._det_timestamps[i][0] < window_start:
                self._det_timestamps[i].popleft()
            count = len(self._det_timestamps[i])
            d.detection_rate_hz = float(count) / DETECTION_WINDOW_SEC if count > 0 else 0.0

            # ── 距上次检测时间 ──
            if self._last_det_time_ms[i] > 0:
                d.time_since_last_det_ms = float(now_ms - self._last_det_time_ms[i])
            else:
                d.time_since_last_det_ms = -1.0  # 从未检测过

            # ── EKF 内部指标（仅在 slot 曾经激活时有效）──
            est = estimated_locations[i]
            has_estimate = not (est[0] == 0 and est[2] == 0)

            if has_estimate:
                filtered_x = float(est[0])
                filtered_y = float(est[2])

                # Jitter: 当前滤波位置与上次滤波位置的距离
                last_fx, last_fy = self._last_filtered_pos[i]
                if last_fx != 0.0 or last_fy != 0.0:
                    d.jitter = float(math.sqrt(
                        (filtered_x - last_fx) ** 2 + (filtered_y - last_fy) ** 2))
                else:
                    d.jitter = 0.0
                self._last_filtered_pos[i] = (filtered_x, filtered_y)

                # Innovation (新息)
                if hasattr(self.kalfilt[i], 'innovation') and self.kalfilt[i].innovation is not None:
                    innov = self.kalfilt[i].innovation
                    # innovation 向量是 [x, vx, y, vy]，取位置分量
                    d.innovation_x = float(innov[0])
                    d.innovation_y = float(innov[2])
                    d.innovation_norm = float(math.sqrt(innov[0] ** 2 + innov[2] ** 2))

                # Raw vs Filtered 距离
                if self._slot_has_detection[i]:
                    raw_x, raw_y = self._slot_raw_xy[i]
                    d.raw_vs_filtered_dist = float(math.sqrt(
                        (raw_x - filtered_x) ** 2 + (raw_y - filtered_y) ** 2))

                # 协方差矩阵
                P = self.kalfilt[i].P_result
                if P is not None and hasattr(P, 'shape'):
                    d.covariance_trace = float(np.trace(P))
                    d.cov_xx = float(P[0, 0])
                    d.cov_yy = float(P[2, 2])

            diag_msg.slots.append(d)

        self.pub_diagnostics.publish(diag_msg)
    

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

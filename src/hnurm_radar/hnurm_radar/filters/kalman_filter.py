"""
kalman_filter.py — 增强型卡尔曼滤波器，用于赛场坐标平滑
==========================================================
为每个被跟踪的机器人维护一个独立的卡尔曼滤波器实例，
对透视变换得到的赛场坐标 (x, y) 进行时序平滑和异常值剔除。

状态向量: [x, y, vx, vy]  (位置 + 速度)
观测向量: [x, y]          (透视变换输出)

特性:
  - 动态过程噪声: 位移越大，速度噪声越大
  - 异常值检测: 单帧跳变超过阈值时拒绝更新
  - 速度约束: 观测隐含速度超过最大速度时拒绝
  - 连续异常检测: 连续异常帧过多时重置滤波器
  - 超时清理: 长时间未更新的滤波器自动移除
  - 首帧直接初始化（不滤波）
"""

import cv2
import numpy as np
import time


class EnhancedKalmanFilter:
    """
    单目标增强型卡尔曼滤波器 (4 状态, 2 观测)

    参数:
        process_noise:      过程噪声系数 (默认 1e-2)
        measurement_noise:  观测噪声系数 (默认 1e-1)
        jump_threshold:     单帧跳变阈值 (m)，超过则拒绝更新 (默认 1.0)
        max_velocity:       最大移动速度 (m/s)，超过则拒绝观测 (默认 5.0)
    """

    def __init__(self, process_noise=1e-2, measurement_noise=1e-1,
                 jump_threshold=1.0, max_velocity=5.0):
        self.kf = cv2.KalmanFilter(4, 2)  # 4 状态, 2 观测

        # 状态转移矩阵 (匀速模型, dt=1)
        self.kf.transitionMatrix = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)

        # 观测矩阵
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
        ], dtype=np.float32)

        # 过程噪声
        self.base_process_noise = process_noise
        self.kf.processNoiseCov = np.eye(4, dtype=np.float32) * process_noise

        # 观测噪声
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * measurement_noise

        # 初始协方差
        self.kf.errorCovPost = np.eye(4, dtype=np.float32)

        self.jump_threshold = jump_threshold
        self.max_velocity = max_velocity
        self.initialized = False
        self.last_update_time = time.time()
        self.last_position = None

        # 连续异常计数器
        self.consecutive_anomalies = 0
        self.max_consecutive_anomalies = 5  # 连续5帧异常则重置

    def init_state(self, x, y):
        """首帧初始化：直接设定位置，速度为 0"""
        self.kf.statePost = np.array([[x], [y], [0], [0]], dtype=np.float32)
        self.kf.errorCovPost = np.eye(4, dtype=np.float32) * 0.1
        self.initialized = True
        self.last_update_time = time.time()
        self.last_position = np.array([x, y])
        self.consecutive_anomalies = 0

    def reset_state(self, x, y):
        """重置滤波器状态"""
        self.init_state(x, y)

    def update(self, x, y):
        """
        输入新观测值，返回滤波后的 (x, y)。
        如果检测到异常跳变，返回预测值而非观测值。
        连续异常过多时重置滤波器。

        返回:
            (filtered_x, filtered_y): 滤波后坐标
        """
        if not self.initialized:
            self.init_state(x, y)
            return x, y

        now = time.time()
        dt = now - self.last_update_time
        dt = max(dt, 0.01)  # 下限保护
        dt = min(dt, 2.0)   # 上限保护（长时间未更新）

        # ★ 速度约束检测：观测隐含速度是否超过最大速度
        if self.last_position is not None:
            obs_velocity = np.sqrt((x - self.last_position[0]) ** 2 +
                                   (y - self.last_position[1]) ** 2) / dt
            if obs_velocity > self.max_velocity:
                # 速度异常 → 拒绝观测，使用预测值
                self.consecutive_anomalies += 1
                if self.consecutive_anomalies >= self.max_consecutive_anomalies:
                    # 连续异常过多，重置滤波器
                    self.init_state(x, y)
                    return x, y
                # 更新状态转移矩阵并预测
                self.kf.transitionMatrix[0, 2] = dt
                self.kf.transitionMatrix[1, 3] = dt
                predicted = self.kf.predict()
                self.last_update_time = now
                return float(predicted[0, 0]), float(predicted[1, 0])

        # 更新状态转移矩阵中的 dt
        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

        # 预测
        predicted = self.kf.predict()
        pred_x, pred_y = predicted[0, 0], predicted[1, 0]

        # 异常值检测
        measurement = np.array([[np.float32(x)], [np.float32(y)]])
        distance = np.sqrt((x - pred_x) ** 2 + (y - pred_y) ** 2)

        if distance > self.jump_threshold:
            # 跳变过大 → 仅使用预测值，不修正
            self.consecutive_anomalies += 1
            if self.consecutive_anomalies >= self.max_consecutive_anomalies:
                # 连续异常过多，重置滤波器
                self.init_state(x, y)
                return x, y
            self.last_update_time = now
            return float(pred_x), float(pred_y)

        # 观测正常，重置异常计数器
        self.consecutive_anomalies = 0

        # 动态调整过程噪声（位移大 → 速度分量噪声更大）
        displacement = 0.0
        if self.last_position is not None:
            displacement = np.sqrt((x - self.last_position[0]) ** 2 +
                                   (y - self.last_position[1]) ** 2)
        dynamic_noise = self.base_process_noise * (1.0 + displacement * 5.0)
        self.kf.processNoiseCov[2, 2] = dynamic_noise
        self.kf.processNoiseCov[3, 3] = dynamic_noise

        # 修正
        corrected = self.kf.correct(measurement)
        self.last_update_time = now
        self.last_position = np.array([x, y])

        return float(corrected[0, 0]), float(corrected[1, 0])

    def predict_only(self):
        """
        仅预测（无新观测），用于目标短暂丢失时的轨迹外推。

        返回:
            (pred_x, pred_y) 或 None（未初始化时）
        """
        if not self.initialized:
            return None

        now = time.time()
        dt = now - self.last_update_time
        dt = max(dt, 0.01)
        dt = min(dt, 2.0)

        self.kf.transitionMatrix[0, 2] = dt
        self.kf.transitionMatrix[1, 3] = dt

        predicted = self.kf.predict()
        return float(predicted[0, 0]), float(predicted[1, 0])

    def time_since_update(self):
        """返回距上次更新的秒数"""
        return time.time() - self.last_update_time


class KalmanFilterWrapper:
    """
    多目标卡尔曼滤波器管理器
    ─────────────────────────
    为每个 car_id 自动创建和管理独立的卡尔曼滤波器。

    参数:
        process_noise:      过程噪声系数
        measurement_noise:  观测噪声系数
        jump_threshold:     单帧跳变阈值 (m)
        max_velocity:       最大移动速度 (m/s)
        max_inactive_time:  滤波器最长不活跃时间 (s)，超过则移除
    """

    def __init__(self, process_noise=1e-2, measurement_noise=1e-1,
                 jump_threshold=1.0, max_velocity=5.0, max_inactive_time=3.0):
        self.filters = {}  # car_id → EnhancedKalmanFilter
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.jump_threshold = jump_threshold
        self.max_velocity = max_velocity
        self.max_inactive_time = max_inactive_time

    def update(self, car_id, x, y):
        """
        更新指定 car_id 的坐标。
        如果该 car_id 没有滤波器则自动创建。

        参数:
            car_id: 机器人 ID (如 1, 2, ..., 101, 102, ...)
            x, y:   透视变换得到的赛场坐标 (m)
        返回:
            (filtered_x, filtered_y): 滤波后坐标
        """
        if car_id not in self.filters:
            self.filters[car_id] = EnhancedKalmanFilter(
                process_noise=self.process_noise,
                measurement_noise=self.measurement_noise,
                jump_threshold=self.jump_threshold,
                max_velocity=self.max_velocity
            )
        return self.filters[car_id].update(x, y)

    def predict(self, car_id):
        """
        仅预测指定 car_id 的坐标（用于目标暂时丢失时）。

        返回:
            (pred_x, pred_y) 或 None
        """
        if car_id in self.filters:
            return self.filters[car_id].predict_only()
        return None

    def cleanup(self):
        """清理超时的滤波器"""
        expired = [
            cid for cid, kf in self.filters.items()
            if kf.time_since_update() > self.max_inactive_time
        ]
        for cid in expired:
            del self.filters[cid]

    def reset(self, car_id=None):
        """重置指定 car_id 的滤波器，或重置全部"""
        if car_id is not None:
            self.filters.pop(car_id, None)
        else:
            self.filters.clear()

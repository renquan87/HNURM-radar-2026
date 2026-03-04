"""
air_kalman_filter.py — 空中目标卡尔曼滤波器

适配自 air_target_radar（深度参考 HITS-radar-2024 实现）。
状态变量: [x, y, vx, vy]
观测变量: [x, y, vx_obs, vy_obs]

优化点（来自 HITS）：
1. cov_factor: 根据速度动态调整过程噪声，运动目标不确定性更大
2. stop_p_time: 丢失超过一定帧数后停止协方差增长，防止 P 矩阵爆炸
3. 观测速度: 用位置差 / 丢失帧数估计观测速度
"""

import numpy as np


class AirKalmanFilter:
    """空中目标 2D 卡尔曼滤波器"""

    def __init__(self, x0: np.ndarray,
                 q_pos=0.1, q_vel=0.5, q_pv=0.05,
                 r_pos=0.2, r_vel=1.0,
                 decay_rate=0.1, max_velocity=5.0,
                 cov_factor=0.1, stop_p_time=30):
        """
        Args:
            x0: 初始位置 [x, y]
            q_pos/q_vel/q_pv: 过程噪声参数
            r_pos/r_vel: 观测噪声参数
            decay_rate: 速度衰减率
            max_velocity: 最大速度限制 (m/s)
            cov_factor: 速度相关协方差因子（HITS 特性）
            stop_p_time: 丢失多少帧后停止协方差增长（HITS 特性）
        """
        self.X = np.array([x0[0], x0[1], 0.0, 0.0])
        self.P = np.diag([q_pos, q_pos, q_vel, q_vel])
        self.X_ = self.X.copy()
        self.P_ = self.P.copy()

        self.A = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1 - decay_rate, 0],
            [0, 0, 0, 1 - decay_rate],
        ])

        self.Q = np.array([
            [q_pos, 0, q_pv, 0],
            [0, q_pos, 0, q_pv],
            [q_pv, 0, q_vel, 0],
            [0, q_pv, 0, q_vel],
        ])

        self.R = np.diag([r_pos, r_pos, r_vel, r_vel])

        self.max_velocity = max_velocity
        self.cov_factor = cov_factor
        self.stop_p_time = stop_p_time
        self.lost_time = 0

    @property
    def pos(self) -> np.ndarray:
        return self.X[:2]

    @property
    def velocity(self) -> np.ndarray:
        return self.X[2:4]

    def mahalanobis_distance(self, pt: np.ndarray) -> float:
        """计算马氏距离（xy 平面）"""
        cov = self.P[:2, :2]
        d = pt[:2] - self.pos
        try:
            return float(d @ np.linalg.inv(cov) @ d)
        except np.linalg.LinAlgError:
            return float(np.sum(d ** 2))

    def _predict(self):
        """预测步 — 含速度相关动态过程噪声"""
        self.X_ = self.A @ self.X
        self.X_[2:4] = np.clip(self.X_[2:4], -self.max_velocity, self.max_velocity)

        if self.lost_time > self.stop_p_time:
            return

        real_Q = self.Q.copy()
        vel = self.X_[2:4]
        vel_cov = np.outer(vel, vel) * self.cov_factor
        real_Q[:2, :2] += vel_cov

        self.P_ = self.A @ self.P @ self.A.T + real_Q

    def update_no_observation(self):
        """无观测更新（仅预测）"""
        self._predict()
        self.X = self.X_.copy()
        self.P = self.P_.copy()
        self.lost_time += 1

    def update(self, z: np.ndarray):
        """有观测更新"""
        self._predict()

        v_obs = (z[:2] - self.X[:2]) / max(self.lost_time + 1, 1)
        Z = np.array([z[0], z[1], v_obs[0], v_obs[1]])

        K = self.P_ @ np.linalg.inv(self.P_ + self.R)
        self.X = self.X_ + K @ (Z - self.X_)
        self.P = (np.eye(4) - K) @ self.P_
        self.lost_time = 0

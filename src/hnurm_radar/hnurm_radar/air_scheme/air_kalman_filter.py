"""
air_kalman_filter.py — 空中目标卡尔曼滤波器

紧密对齐 HITS-radar-2024 KalmanFilter.cpp 实现。

状态: [x, y, vx, vy]
观测: [x, y, vx_obs, vy_obs]

HITS 关键特性（全部实现）：
1. init P = Q × p_times（初值大，方便首帧快速收敛到真实位置）
2. cov_factor：速度越大，过程噪声越大（动态 Q）
3. stop_p_time：丢失超过此帧数后停止 P 增长，防矩阵爆炸
4. 速度衰减 decay_rate：模仿空气阻力
5. 观测速度由位置差/丢失帧数估算
6. 速度限幅 max_velocity

马氏距离修正：分母使用 P_[:2,:2] 而非 P[:2,:2]，
即用预测协方差（而非初始协方差）做马氏匹配，与 HITS 一致。
"""

import numpy as np


class AirKalmanFilter:
    """空中目标 2D 卡尔曼滤波器（严格对齐 HITS KalmanFilter.cpp）"""

    # ------------------------------------------------------------------ #
    # 全局参数（类变量，由外部 read_params 设置，与 HITS C++ 用法一致）
    # ------------------------------------------------------------------ #
    _Kf_Q: np.ndarray = None
    _Kf_R: np.ndarray = None
    _Kf_A: np.ndarray = None
    _Kf_speed_limit: float = 10.0
    _cov_factor: float = 2.5
    _stop_p_time: int = 40
    _init_p_times: int = 20     # HITS: P0 = Q * init_lost

    @classmethod
    def set_global_params(
        cls,
        q_pos: float = 1e-7, q_vel: float = 5e-6, q_pv: float = 5e-6,
        r_pos: float = 5e-2, r_vel: float = 5e-2,
        decay_rate: float = 1e-4,
        max_velocity: float = 10.0,
        cov_factor: float = 2.5,
        stop_p_time: int = 40,
        init_p_times: int = 20,
    ):
        """设置全局卡尔曼参数（与 HITS read_params 对应）。

        需在首次创建 AirKalmanFilter 之前调用一次。
        """
        cls._Kf_Q = np.array([
            [q_pos,  0,      q_pv,   0     ],
            [0,      q_pos,  0,      q_pv  ],
            [q_pv,   0,      q_vel,  0     ],
            [0,      q_pv,   0,      q_vel ],
        ])
        cls._Kf_R = np.diag([r_pos, r_pos, r_vel, r_vel])
        cls._Kf_A = np.array([
            [1, 0, 1, 0],
            [0, 1, 0, 1],
            [0, 0, 1 - decay_rate, 0],
            [0, 0, 0, 1 - decay_rate],
        ])
        cls._Kf_speed_limit = max_velocity
        cls._cov_factor     = cov_factor
        cls._stop_p_time    = stop_p_time
        cls._init_p_times   = init_p_times

    def __init__(self, x0: np.ndarray, p_times: int = None, **kwargs):
        """
        Args:
            x0: 初始位置 [x, y]
            p_times: 初始协方差倍数 P = Q * p_times；None → 使用类变量 _init_p_times

        旧接口兼容：若 set_global_params 尚未调用，则用 kwargs 中的单实例参数
        初始化（退化为实例级参数模式）。
        """
        # ---- 兼容旧的逐实例参数调用方式（GPT 生成版本遗留） ----
        if self.__class__._Kf_Q is None or kwargs:
            q_pos        = kwargs.get("q_pos",        1e-7)
            q_vel        = kwargs.get("q_vel",        5e-6)
            q_pv         = kwargs.get("q_pv",         5e-6)
            r_pos        = kwargs.get("r_pos",        5e-2)
            r_vel        = kwargs.get("r_vel",        5e-2)
            decay_rate   = kwargs.get("decay_rate",   1e-4)
            max_velocity = kwargs.get("max_velocity", 10.0)
            cov_factor   = kwargs.get("cov_factor",   2.5)
            stop_p_time  = kwargs.get("stop_p_time",  40)
            init_p_times = kwargs.get("init_p_times", 20)

            self._Q = np.array([
                [q_pos, 0,     q_pv,  0    ],
                [0,     q_pos, 0,     q_pv ],
                [q_pv,  0,     q_vel, 0    ],
                [0,     q_pv,  0,     q_vel],
            ])
            self._R = np.diag([r_pos, r_pos, r_vel, r_vel])
            self._A = np.array([
                [1, 0, 1, 0],
                [0, 1, 0, 1],
                [0, 0, 1 - decay_rate, 0],
                [0, 0, 0, 1 - decay_rate],
            ])
            self._speed_limit  = max_velocity
            self._cov_f        = cov_factor
            self._stop_p       = stop_p_time
            self._p_times_init = p_times if p_times is not None else init_p_times
            self._use_instance = True
        else:
            self._use_instance = False
            self._p_times_init = p_times if p_times is not None else self.__class__._init_p_times

        # ---- 状态 & 协方差初始化 ----
        _Q = self._Q if self._use_instance else self.__class__._Kf_Q
        self.X  = np.array([float(x0[0]), float(x0[1]), 0.0, 0.0])
        self.P  = _Q * self._p_times_init          # HITS: P0 = Q * p_times
        self.X_ = self.X.copy()
        self.P_ = self.P.copy()
        self.lost_time = 0

    # ------------------------------------------------------------------ #
    # 内部辅助
    # ------------------------------------------------------------------ #
    @property
    def _Q_(self) -> np.ndarray:
        return self._Q if self._use_instance else self.__class__._Kf_Q

    @property
    def _R_(self) -> np.ndarray:
        return self._R if self._use_instance else self.__class__._Kf_R

    @property
    def _A_(self) -> np.ndarray:
        return self._A if self._use_instance else self.__class__._Kf_A

    @property
    def _speed_limit_(self) -> float:
        return self._speed_limit if self._use_instance else self.__class__._Kf_speed_limit

    @property
    def _cov_factor_(self) -> float:
        return self._cov_f if self._use_instance else self.__class__._cov_factor

    @property
    def _stop_p_time_(self) -> int:
        return self._stop_p if self._use_instance else self.__class__._stop_p_time

    # ------------------------------------------------------------------ #
    # 公开属性
    # ------------------------------------------------------------------ #
    @property
    def pos(self) -> np.ndarray:
        """卡尔曼估计位置 [x, y]"""
        return self.X[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        """卡尔曼估计速度 [vx, vy]"""
        return self.X[2:4].copy()

    # ------------------------------------------------------------------ #
    # 马氏距离（用预测协方差 P_ 的 (0:2, 0:2) 块，与 HITS 一致）
    # ------------------------------------------------------------------ #
    def mahalanobis_distance(self, pt: np.ndarray) -> float:
        """计算 xy 平面马氏距离（平方值，与 HITS mah_dis 接口一致）

        使用预测协方差 P_ 的位置块。P_ 在首帧 update 之前等于初始 P，
        初始 P 已被放大 p_times 倍，确保距离量纲合理。
        """
        # 若尚未做过预测，使用 P（=P_初始化）
        cov = self.P_[:2, :2] if np.any(self.P_ != self.P) else self.P[:2, :2]
        d = np.asarray(pt[:2], dtype=np.float64) - self.X_[:2]
        try:
            inv_cov = np.linalg.inv(cov + np.eye(2) * 1e-9)
            return float(d @ inv_cov @ d)
        except np.linalg.LinAlgError:
            return float(np.sum(d ** 2))

    # ------------------------------------------------------------------ #
    # 预测步（HITS predict()）
    # ------------------------------------------------------------------ #
    def _predict(self):
        A = self._A_
        self.X_ = A @ self.X
        # 速度限幅
        self.X_[2:4] = np.clip(self.X_[2:4], -self._speed_limit_, self._speed_limit_)

        # stop_p_time 后停止 P 增长（防止长期丢失导致矩阵爆炸）
        if self.lost_time > self._stop_p_time_:
            return

        real_Q = self._Q_.copy()
        vel = self.X_[2:4]
        # 速度越大 → 过程噪声越大（HITS cov_factor 机制）
        real_Q[:2, :2] += np.outer(vel, vel) * self._cov_factor_
        self.P_ = A @ self.P @ A.T + real_Q

    # ------------------------------------------------------------------ #
    # 更新步（无观测，HITS update()无参版本）
    # ------------------------------------------------------------------ #
    def update_no_observation(self):
        """仅预测，不融合观测。"""
        self._predict()
        self.X = self.X_.copy()
        self.P = self.P_.copy()
        self.lost_time += 1

    # ------------------------------------------------------------------ #
    # 更新步（有观测，HITS update(Z)版本）
    # ------------------------------------------------------------------ #
    def update(self, z: np.ndarray):
        """融合观测 z=[x, y] 的卡尔曼更新。"""
        self._predict()

        # 用位置差估计观测速度（HITS 原文方式）
        v_obs = (np.asarray(z[:2], dtype=np.float64) - self.X[:2]) / float(
            max(self.lost_time + 1, 1)
        )
        Z = np.array([z[0], z[1], v_obs[0], v_obs[1]])

        # 卡尔曼增益 K = P_ (P_ + R)^{-1}  （H=I 时的化简形式）
        S = self.P_ + self._R_
        try:
            K = self.P_ @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            K = self.P_ @ np.linalg.pinv(S)

        self.X = self.X_ + K @ (Z - self.X_)
        self.P = (np.eye(4) - K) @ self.P_
        self.lost_time = 0

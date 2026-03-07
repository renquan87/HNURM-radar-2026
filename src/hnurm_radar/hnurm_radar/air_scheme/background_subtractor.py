"""
background_subtractor.py — 空中目标背景减除模块

适配自 air_target_radar（参考 HITS-radar-2024 VoxelGrid 实现）。
通过体素网格记录静态背景点，过滤已知静态点，只保留动态目标。

坐标系说明（雷达坐标系 / livox 帧，右手系）：
  X = 正前方，Y = 左方，Z = 上方。LiDAR 位于原点 (0,0,0)。
  更负的 Z = 更低 = 更接近地面（例：地面 z ≈ -1.05）。
  更接近 0 的 Z = 更高 = 更接近 LiDAR（例：无人机顶 z ≈ -0.5）。
  空中目标 Z ∈ [z_min, z_max]（如 [-1.0, -0.5]），地面 Z < z_min。

飞行区域排除机制：
  flight_z_min / flight_z_max 定义无人机飞行区域的 Z 范围。
  仅在线学习（learn()）时跳过此区域，避免将悬停无人机学入背景。
  filter() 对所有高度的点做背景检查（PCD 中已知的静态结构即使在飞行高度也会被移除）。
  无人机本身不在 PCD 中，因此不会被误删。

  正确调用：flight_z_min = height_filter.z_min（飞行区域下界，如 -1.0）
            flight_z_max = height_filter.z_max（飞行区域上界，如 -0.5）

性能优化：
  filter() 使用 numpy 向量化 + searchsorted 二分查找，避免 Python for 循环。
  背景查找表带 XY 邻域扩展（±1 体素），容忍 registration TF 的小偏差（±voxel_size）。
  查找表在 BG 模型更新后惰性重建并缓存，不随帧重复构建。
"""

import numpy as np
import open3d as o3d
from typing import Optional


class AirBackgroundSubtractor:
    """基于体素网格的背景减除器（HITS VoxelGrid 语义对齐版）"""

    def __init__(
        self,
        voxel_size:       float = 0.15,
        occupy_threshold: int   = 5,
        learning_frames:  int   = 0,
        flight_z_min:     float = None,   # 飞行区域 Z 下界（更负=更接近地面）
        flight_z_max:     float = None,   # 飞行区域 Z 上界（更接近0=更高）
    ):
        """
        Args:
            voxel_size:       体素大小 (m)
            occupy_threshold: 占据阈值，超过此次数的体素视为背景
            learning_frames:  学习帧数（前 N 帧建立背景，0 = 持续学习）
            flight_z_min:     飞行区域 Z 下界；z < 此值 = 地面区域（学为背景）
            flight_z_max:     飞行区域 Z 上界；z > 此值 = LiDAR上方区域（学为背景）
                              [flight_z_min, flight_z_max] 内的点在在线学习时跳过，
                              避免将悬停无人机学入背景模型。
        """
        self.voxel_size       = voxel_size
        self.occupy_threshold = occupy_threshold
        self.learning_frames  = learning_frames
        self.flight_z_min     = flight_z_min
        self.flight_z_max     = flight_z_max
        self.voxel_counts: dict = {}
        self.frame_count  = 0
        self.is_ready     = (learning_frames == 0)
        # 缓存：扩展 BG 查找表（排序后的 int64 编码数组，用于 searchsorted）
        self._bg_cache: Optional[np.ndarray] = None

    def _invalidate_cache(self):
        """标记 BG 缓存失效（voxel_counts 变更后调用）"""
        self._bg_cache = None

    def _get_expanded_bg(self) -> Optional[np.ndarray]:
        """获取扩展背景体素编码（带 XY 邻域扩展 + 缓存）。

        每个背景体素向 X/Y 方向各扩展 ±1 个体素（共 5 个位置：自身+上下左右），
        有效覆盖范围在 XY 平面上扩大 ±voxel_size（0.15m）。
        **不在 Z 方向扩展**，避免地面 BG 侵蚀 Z 方向仅差 1 个体素的无人机底部。

        作用：容忍 registration TF 在水平面上的小偏差（±0.15m）。
        LiDAR 固定安装，Z 方向由重力对齐，Z 偏差远小于 XY。

        Returns:
            排序后的 int64 编码数组（用于 np.searchsorted），或 None（无 BG）
        """
        if self._bg_cache is not None:
            return self._bg_cache

        # 提取达到占据阈值的背景体素坐标
        bg_list = [(k[0], k[1], k[2]) for k, c in self.voxel_counts.items()
                   if c >= self.occupy_threshold]
        if not bg_list:
            return None

        bg_voxels = np.array(bg_list, dtype=np.int64)  # (N, 3)

        # XY 方向 ±1 扩展（全 3×3 方阵含对角线，不扩展 Z）
        # 十字模式(5)只覆盖正交方向的 TF 偏差，
        # 3×3 方阵(9)覆盖所有方向（含对角线），对 TF 在 XY 平面的任意偏差更鲁棒
        offsets = np.array([
            [dx, dy, 0]
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
        ], dtype=np.int64)

        # 广播扩展：(N, 1, 3) + (1, 9, 3) → (N, 9, 3) → (N*9, 3)
        expanded = bg_voxels[:, np.newaxis, :] + offsets[np.newaxis, :, :]
        expanded = expanded.reshape(-1, 3)

        # 编码 + 去重 + 排序（np.unique 自动排序）
        bg_encoded = (expanded[:, 0] * 1000003 +
                      expanded[:, 1] * 1000033 +
                      expanded[:, 2])
        bg_sorted = np.unique(bg_encoded)

        self._bg_cache = bg_sorted
        return bg_sorted

    def learn(self, pcd: o3d.geometry.PointCloud):
        """学习背景帧 — 跳过飞行区域，累积其余静态点。

        跳过 [flight_z_min, flight_z_max] 范围内的点（无人机飞行区域），
        学习地面（z < flight_z_min）和上方（z > flight_z_max）的静态结构，
        避免将悬停无人机学入背景模型。
        """
        # 学习阶段结束后，不再更新背景模型
        if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
            self.is_ready = True
            return

        points = np.asarray(pcd.points)
        if len(points) == 0:
            self.frame_count += 1
            if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
                self.is_ready = True
            return

        # 跳过飞行区域 [flight_z_min, flight_z_max]，学习其余所有点
        if self.flight_z_min is not None and self.flight_z_max is not None:
            flight_mask  = (points[:, 2] >= self.flight_z_min) & (points[:, 2] <= self.flight_z_max)
            learn_points = points[~flight_mask]  # 地面 + LiDAR上方 = 背景
        else:
            learn_points = points

        if len(learn_points) > 0:
            voxel_keys = (learn_points / self.voxel_size).astype(np.int32)
            # 向量化去重：将三维体素坐标编码为单个int64键，避免逐点Python循环
            encoded = (voxel_keys[:, 0].astype(np.int64) * 1000003 +
                       voxel_keys[:, 1].astype(np.int64) * 1000033 +
                       voxel_keys[:, 2].astype(np.int64))
            unique_encoded, unique_idx = np.unique(encoded, return_index=True)
            unique_voxels = voxel_keys[unique_idx]
            for vk in unique_voxels:
                key = (int(vk[0]), int(vk[1]), int(vk[2]))
                self.voxel_counts[key] = self.voxel_counts.get(key, 0) + 1
            self._invalidate_cache()  # BG 模型已变更

        self.frame_count += 1
        if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
            self.is_ready = True

    def filter(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """过滤背景点，只保留前景（动态/空中）点。

        策略：
          - 背景模型未就绪时直接返回原始点云（不过滤）
          - 所有点均通过体素计数判断是否为背景
          - 带 XY 邻域扩展（±1 体素），容忍 TF 在水平面的小偏差

        实现：
          向量化 numpy + searchsorted 二分查找，避免 Python for 循环。
          55k 点的过滤从 ~100ms 降至 ~5ms。

        设计说明：
          learn() 跳过飞行区域，保证悬停无人机不会被学成背景。
          但 filter() 必须检查所有高度的点，因为 PCD 预加载的背景
          可能包含飞行高度范围内的静态结构（桌角、立柱等），这些必须被移除。
          无人机本身不在 PCD 中，所以不会被误删。
        """
        if not self.is_ready or not self.voxel_counts:
            return pcd

        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd

        # 获取扩展 BG 查找表（带缓存）
        bg_sorted = self._get_expanded_bg()
        if bg_sorted is None or len(bg_sorted) == 0:
            return pcd

        # 向量化编码实时点体素坐标
        voxel_keys = (points / self.voxel_size).astype(np.int32)
        encoded = (voxel_keys[:, 0].astype(np.int64) * 1000003 +
                   voxel_keys[:, 1].astype(np.int64) * 1000033 +
                   voxel_keys[:, 2].astype(np.int64))

        # 二分查找：判断每个点是否落入扩展背景体素
        idx = np.searchsorted(bg_sorted, encoded)
        idx_clipped = np.clip(idx, 0, len(bg_sorted) - 1)
        is_bg = bg_sorted[idx_clipped] == encoded
        mask = ~is_bg

        foreground = o3d.geometry.PointCloud()
        foreground.points = o3d.utility.Vector3dVector(points[mask])
        return foreground

    # ------------------------------------------------------------------ #
    # 预建 PCD 加载（替代在线学习）
    # ------------------------------------------------------------------ #
    def load_from_pcd(self, pcd: o3d.geometry.PointCloud,
                      use_all_points: bool = True) -> int:
        """从预建点云地图加载背景模型，立即就绪，无需在线学习。

        调用此方法后 is_ready = True，后续 learn() 调用将被跳过。

        关键设计：
          预建 PCD 是在无人机起飞前采集的纯静态场景，不含动态目标。
          因此默认 use_all_points=True，将 PCD 中所有高度的点都作为
          背景模型（包括地面、墙壁、天花板、灯具等）。

          flight_z_min/flight_z_max 仅限制 **在线学习**（learn() 方法），
          因为在线学习时无人机可能已在飞行，需要排除飞行区域。

        Args:
            pcd: Open3D 点云对象（应已处于 livox 帧坐标系）。
            use_all_points: True=使用PCD全部高度点（推荐）；
                            False=排除飞行区域（与在线学习一致）

        Returns:
            新增背景体素数量
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return 0

        # 默认加载全部 PCD 点作为背景（PCD 是纯静态场景，不含无人机）
        # 仅当显式指定 use_all_points=False 时才排除飞行区域
        if not use_all_points and self.flight_z_min is not None and self.flight_z_max is not None:
            flight_mask = (points[:, 2] >= self.flight_z_min) & (points[:, 2] <= self.flight_z_max)
            points = points[~flight_mask]

        if len(points) == 0:
            return 0

        # 体素化 → 每个体素直接设为占据阈值（确认为背景）
        voxel_keys = (points / self.voxel_size).astype(np.int32)
        # 向量化去重，避免逐点Python循环
        encoded = (voxel_keys[:, 0].astype(np.int64) * 1000003 +
                   voxel_keys[:, 1].astype(np.int64) * 1000033 +
                   voxel_keys[:, 2].astype(np.int64))
        _, unique_idx = np.unique(encoded, return_index=True)
        unique_voxels = voxel_keys[unique_idx]
        added = 0
        for vk in unique_voxels:
            key = (int(vk[0]), int(vk[1]), int(vk[2]))
            old = self.voxel_counts.get(key, 0)
            self.voxel_counts[key] = max(old, self.occupy_threshold)
            if old < self.occupy_threshold:
                added += 1

        # 标记背景就绪，跳过后续在线学习
        self.is_ready = True
        self._invalidate_cache()  # BG 模型已变更
        if self.learning_frames > 0:
            self.frame_count = self.learning_frames  # 阻止后续 learn() 覆写
        return added

    @property
    def background_voxel_count(self) -> int:
        """已确认为背景的体素数量"""
        return sum(1 for c in self.voxel_counts.values()
                   if c >= self.occupy_threshold)

    def reset(self):
        """重置背景模型（场景切换时调用）"""
        self.voxel_counts = {}
        self.frame_count  = 0
        self.is_ready     = (self.learning_frames == 0)
        self._invalidate_cache()

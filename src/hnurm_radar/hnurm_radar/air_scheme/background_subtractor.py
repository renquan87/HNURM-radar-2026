"""
background_subtractor.py — 空中目标背景减除模块

适配自 air_target_radar（参考 HITS-radar-2024 VoxelGrid 实现）。
通过体素网格记录静态背景点，过滤已知静态点，只保留动态目标。

关键设计：learn_z_max 限制只学习地面区域的点作为背景模型，
这样即使空中机器人悬停不动，也不会被误标为背景。
"""

import numpy as np
import open3d as o3d


class AirBackgroundSubtractor:
    """基于体素网格的背景减除器"""

    def __init__(self, voxel_size: float = 0.15,
                 occupy_threshold: int = 5,
                 learning_frames: int = 0,
                 learn_z_max: float = None):
        """
        Args:
            voxel_size: 体素大小 (m)
            occupy_threshold: 占据阈值，超过此次数的体素视为背景
            learning_frames: 学习帧数 (前 N 帧用于建立背景模型)
            learn_z_max: 只学习 Z <= 此值的点作为背景
                         设置为高度过滤的 z_min 可避免空中悬停目标被误标为背景
        """
        self.voxel_size = voxel_size
        self.occupy_threshold = occupy_threshold
        self.learning_frames = learning_frames
        self.learn_z_max = learn_z_max
        self.voxel_counts: dict = {}
        self.frame_count = 0
        self.is_ready = learning_frames == 0

    def learn(self, pcd: o3d.geometry.PointCloud):
        """学习背景帧 — 只用地面区域的点累积体素占据计数"""
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return

        # 只用地面区域的点学习背景
        if self.learn_z_max is not None:
            ground_mask = points[:, 2] <= self.learn_z_max
            learn_points = points[ground_mask]
        else:
            learn_points = points

        if len(learn_points) == 0:
            self.frame_count += 1
            if self.frame_count >= self.learning_frames:
                self.is_ready = True
            return

        voxel_keys = (learn_points / self.voxel_size).astype(int)
        seen_this_frame = set()

        for vk in voxel_keys:
            key = tuple(vk)
            if key not in seen_this_frame:
                seen_this_frame.add(key)
                self.voxel_counts[key] = self.voxel_counts.get(key, 0) + 1

        self.frame_count += 1
        if self.frame_count >= self.learning_frames:
            self.is_ready = True

    def filter(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """过滤背景点，只保留前景（动态）点"""
        if not self.is_ready or not self.voxel_counts:
            return pcd

        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd

        voxel_keys = (points / self.voxel_size).astype(int)
        mask = np.zeros(len(points), dtype=bool)

        for i, vk in enumerate(voxel_keys):
            key = tuple(vk)
            count = self.voxel_counts.get(key, 0)
            if count < self.occupy_threshold:
                mask[i] = True

        foreground = o3d.geometry.PointCloud()
        foreground.points = o3d.utility.Vector3dVector(points[mask])
        return foreground

    @property
    def background_voxel_count(self) -> int:
        """背景体素数量"""
        return sum(1 for c in self.voxel_counts.values()
                   if c >= self.occupy_threshold)

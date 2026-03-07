"""
point_cloud_processor.py — 空中目标点云预处理模块

适配自 air_target_radar，用于 hnurm_radar 项目。
功能：ROI 裁剪、体素降采样、高度过滤（提取空中区域点云）
"""

import numpy as np
import open3d as o3d
from .air_config import AirPreprocessConfig


class AirPointCloudProcessor:
    """空中目标点云预处理器"""

    def __init__(self, config: AirPreprocessConfig):
        self.config = config

    @staticmethod
    def _masked_select(pcd: o3d.geometry.PointCloud, mask: np.ndarray) -> o3d.geometry.PointCloud:
        if mask is None or len(mask) == 0:
            return pcd
        idx = np.flatnonzero(mask)
        if len(idx) == 0:
            return o3d.geometry.PointCloud()
        return pcd.select_by_index(idx)

    def roi_height_mask(self, points: np.ndarray,
                        z_min: float = None,
                        z_max: float = None) -> np.ndarray:
        """一次性计算 ROI+高度掩码，减少中间拷贝"""
        roi = self.config.roi
        hf = self.config.height_filter
        z_min = hf.z_min if z_min is None else z_min
        z_max = hf.z_max if z_max is None else z_max
        return (
            (points[:, 0] >= roi.x_min) & (points[:, 0] <= roi.x_max) &
            (points[:, 1] >= roi.y_min) & (points[:, 1] <= roi.y_max) &
            (points[:, 2] >= roi.z_min) & (points[:, 2] <= roi.z_max) &
            (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        )

    def crop_roi(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """ROI 区域裁剪"""
        roi = self.config.roi
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd
        mask = (
            (points[:, 0] >= roi.x_min) & (points[:, 0] <= roi.x_max) &
            (points[:, 1] >= roi.y_min) & (points[:, 1] <= roi.y_max) &
            (points[:, 2] >= roi.z_min) & (points[:, 2] <= roi.z_max)
        )
        return self._masked_select(pcd, mask)

    def downsample(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """体素降采样"""
        if self.config.voxel_size > 0:
            return pcd.voxel_down_sample(self.config.voxel_size)
        return pcd

    def get_adaptive_height_range(self, tracked_heights=None):
        """根据已跟踪目标高度动态调整过滤区间"""
        hf = self.config.height_filter
        ah = self.config.adaptive_height
        if not ah.enabled or not tracked_heights:
            return hf.z_min, hf.z_max

        hs = np.asarray(tracked_heights, dtype=np.float64)
        hs = hs[np.isfinite(hs)]
        if len(hs) == 0:
            return hf.z_min, hf.z_max

        z_min = float(np.min(hs) - ah.margin)
        z_max = float(np.max(hs) + ah.margin)
        span = z_max - z_min
        if span < ah.min_span:
            mid = 0.5 * (z_min + z_max)
            z_min = mid - ah.min_span * 0.5
            z_max = mid + ah.min_span * 0.5
        if span > ah.max_span:
            mid = 0.5 * (z_min + z_max)
            z_min = mid - ah.max_span * 0.5
            z_max = mid + ah.max_span * 0.5

        z_min = max(z_min, hf.z_min)
        z_max = min(z_max, hf.z_max)
        if z_min >= z_max:
            return hf.z_min, hf.z_max
        return z_min, z_max

    def filter_height(self, pcd: o3d.geometry.PointCloud,
                      tracked_heights=None) -> o3d.geometry.PointCloud:
        """高度过滤 — 提取空中区域点云

        雷达坐标系中 LiDAR 在 Z=0，Z 轴向上为正。
        更负的 Z = 更低 = 更接近地面（如地面 z ≈ -1.05）
        更接近 0 的 Z = 更高 = 更接近 LiDAR（如无人机顶 z ≈ -0.5）
        z_min = 飞行区域下界（接近地面侧），z_max = 飞行区域上界（接近 LiDAR 侧）
        """
        z_min, z_max = self.get_adaptive_height_range(tracked_heights)
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        return self._masked_select(pcd, mask)

    def process(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """完整预处理流程：ROI → 降采样 → 高度过滤"""
        pcd = self.crop_roi(pcd)
        pcd = self.downsample(pcd)
        pcd = self.filter_height(pcd)
        return pcd

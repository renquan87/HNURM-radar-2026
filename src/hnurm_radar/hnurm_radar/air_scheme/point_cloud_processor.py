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
        return pcd.select_by_index(np.where(mask)[0])

    def downsample(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """体素降采样"""
        if self.config.voxel_size > 0:
            return pcd.voxel_down_sample(self.config.voxel_size)
        return pcd

    def filter_height(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """高度过滤 — 提取空中区域点云

        雷达坐标系中 Z 轴向下为负，空中目标 Z 值在 z_min ~ z_max 之间。
        z_min 更负 = 离雷达更远 = 离地面更高
        z_max 更接近 0 = 离雷达更近 = 离地面更低
        """
        hf = self.config.height_filter
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd
        mask = (points[:, 2] >= hf.z_min) & (points[:, 2] <= hf.z_max)
        return pcd.select_by_index(np.where(mask)[0])

    def process(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """完整预处理流程：ROI → 降采样 → 高度过滤"""
        pcd = self.crop_roi(pcd)
        pcd = self.downsample(pcd)
        pcd = self.filter_height(pcd)
        return pcd

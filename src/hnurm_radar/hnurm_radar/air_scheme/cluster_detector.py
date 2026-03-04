"""
cluster_detector.py — 空中目标 DBSCAN 聚类检测模块

适配自 air_target_radar，用于 hnurm_radar 项目。
功能：Z 轴压缩 DBSCAN 聚类 + 目标筛选（点数/尺寸过滤）
"""

import numpy as np
import open3d as o3d
from sklearn.cluster import DBSCAN
from dataclasses import dataclass
from typing import List
from .air_config import AirClusterConfig, AirTargetFilterConfig


@dataclass
class AirTarget:
    """空中目标数据"""
    center: np.ndarray          # 中心坐标 [x, y, z]（雷达坐标系）
    bbox_min: np.ndarray        # 包围盒最小点
    bbox_max: np.ndarray        # 包围盒最大点
    num_points: int             # 点数
    size: np.ndarray            # 尺寸 [dx, dy, dz]
    points: np.ndarray          # 原始点云数组


class AirClusterDetector:
    """空中目标 DBSCAN 聚类检测器"""

    def __init__(self, cluster_config: AirClusterConfig,
                 filter_config: AirTargetFilterConfig):
        self.cluster_config = cluster_config
        self.filter_config = filter_config
        self.z_zip = cluster_config.z_zip

    def cluster(self, pcd: o3d.geometry.PointCloud) -> List[np.ndarray]:
        """DBSCAN 密度聚类（含 Z 轴压缩）

        HITS 特性：聚类前将 Z 轴乘以 z_zip (< 1)，使水平距离权重更大，
        避免高度差导致同一空中目标被分成多个簇。聚类后使用原始坐标。
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return []

        # Z 轴压缩：聚类用压缩坐标，结果用原始坐标
        points_zip = points.copy()
        points_zip[:, 2] *= self.z_zip

        db = DBSCAN(
            eps=self.cluster_config.eps,
            min_samples=self.cluster_config.min_samples,
        )
        labels = db.fit_predict(points_zip)

        clusters = []
        unique_labels = set(labels)
        unique_labels.discard(-1)  # 去除噪声点

        for label in unique_labels:
            cluster_points = points[labels == label]
            clusters.append(cluster_points)

        return clusters

    def filter_targets(self, clusters: List[np.ndarray]) -> List[AirTarget]:
        """筛选空中目标 — 根据点数和尺寸过滤"""
        targets = []
        fc = self.filter_config

        for cluster_points in clusters:
            num_points = len(cluster_points)

            # 点数过滤
            if num_points < fc.min_points or num_points > fc.max_points:
                continue

            bbox_min = cluster_points.min(axis=0)
            bbox_max = cluster_points.max(axis=0)
            size = bbox_max - bbox_min

            # 尺寸过滤（取最大维度）
            max_dim = size.max()
            if max_dim < fc.min_size or max_dim > fc.max_size:
                continue

            center = cluster_points.mean(axis=0)

            target = AirTarget(
                center=center,
                bbox_min=bbox_min,
                bbox_max=bbox_max,
                num_points=num_points,
                size=size,
                points=cluster_points,
            )
            targets.append(target)

        return targets

    def detect(self, pcd: o3d.geometry.PointCloud) -> List[AirTarget]:
        """完整检测流程：聚类 → 筛选"""
        clusters = self.cluster(pcd)
        targets = self.filter_targets(clusters)
        return targets

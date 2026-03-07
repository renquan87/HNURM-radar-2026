"""
cluster_detector.py — 空中目标 DBSCAN 聚类检测模块

适配自 air_target_radar，用于 hnurm_radar 项目。
功能：Z 轴压缩 DBSCAN 聚类 + 目标筛选（点数/尺寸过滤）
"""

import numpy as np
import open3d as o3d
from dataclasses import dataclass
from typing import List, Optional

try:
    from sklearn.cluster import DBSCAN
except Exception:  # pragma: no cover
    DBSCAN = None
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
    confidence: float = 0.0
    source: str = "cluster"


class AirClusterDetector:
    """空中目标 DBSCAN 聚类检测器"""

    def __init__(self, cluster_config: AirClusterConfig,
                 filter_config: AirTargetFilterConfig):
        self.cluster_config = cluster_config
        self.filter_config = filter_config
        self.z_zip = cluster_config.z_zip

    def _cluster_open3d(self, points: np.ndarray) -> np.ndarray:
        pcd_zip = o3d.geometry.PointCloud()
        pcd_zip.points = o3d.utility.Vector3dVector(points)
        labels = np.asarray(
            pcd_zip.cluster_dbscan(
                eps=self.cluster_config.eps,
                min_points=self.cluster_config.min_samples,
                print_progress=False,
            )
        )
        return labels

    def _cluster_sklearn(self, points: np.ndarray) -> np.ndarray:
        if DBSCAN is None:
            return self._cluster_open3d(points)
        db = DBSCAN(
            eps=self.cluster_config.eps,
            min_samples=self.cluster_config.min_samples,
        )
        return db.fit_predict(points)

    @staticmethod
    def _split_by_projection_gap(cluster_points: np.ndarray,
                                 min_gap: float,
                                 min_points: int) -> Optional[List[np.ndarray]]:
        if len(cluster_points) < max(min_points * 2, 8):
            return None

        xy = cluster_points[:, :2]
        centered = xy - xy.mean(axis=0)
        cov = centered.T @ centered / max(len(centered) - 1, 1)
        eigvals, eigvecs = np.linalg.eigh(cov)
        principal = eigvecs[:, int(np.argmax(eigvals))]
        proj = centered @ principal
        order = np.argsort(proj)
        proj_sorted = proj[order]
        gaps = np.diff(proj_sorted)
        if len(gaps) == 0:
            return None
        cut_idx = int(np.argmax(gaps))
        max_gap = float(gaps[cut_idx])
        if max_gap < min_gap:
            return None
        left_idx = order[:cut_idx + 1]
        right_idx = order[cut_idx + 1:]
        if len(left_idx) < min_points or len(right_idx) < min_points:
            return None
        return [cluster_points[left_idx], cluster_points[right_idx]]

    def _maybe_split_cluster(self, cluster_points: np.ndarray) -> List[np.ndarray]:
        cc = self.cluster_config
        if (not cc.split_merged) or len(cluster_points) < cc.split_min_points * 2:
            return [cluster_points]
        size = cluster_points.max(axis=0) - cluster_points.min(axis=0)
        if max(size[0], size[1]) < cc.split_size_xy:
            return [cluster_points]

        split = self._split_by_projection_gap(
            cluster_points,
            min_gap=cc.split_min_gap,
            min_points=cc.split_min_points,
        )
        return split if split else [cluster_points]

    def _compute_confidence(self, num_points: int, size: np.ndarray) -> float:
        """计算目标置信度（对数点数评分 + 尺寸评分）。

        设计目标：
          - 点数评分使用对数尺度，让 400+ 点的真目标远远领先 50-150 点的假阳性
          - 尺寸评分权重降低到 0.2，避免"尺寸恰好合适"的假阳性获得过高分
          - 典型结果：无人机(480pt) → 0.75+，假阳性(150pt) → 0.35，碎片(30pt) → 0.15
        """
        fc = self.filter_config

        # 对数点数评分：log(n)/log(max_points) — 让大点数远离小点数
        # 例：min=30, max=1500 → n=30得0.28, n=100得0.38, n=300得0.47, n=500得0.51, n=1000得0.57
        # 再归一化到 [0, 1]：减去 log(min)/log(max) 的基线
        if num_points <= fc.min_points:
            pts_score = 0.0
        else:
            log_max = np.log(max(fc.max_points, fc.min_points + 1))
            log_min = np.log(max(fc.min_points, 1))
            log_n = np.log(num_points)
            pts_score = float(np.clip((log_n - log_min) / max(log_max - log_min, 1e-6), 0.0, 1.0))

        # 尺寸评分：最大维度在 [min_size, max_size] 中间时最高
        max_dim = float(np.max(size))
        if max_dim <= fc.min_size:
            size_score = 0.0
        elif max_dim >= fc.max_size:
            size_score = 0.0
        else:
            span = max(fc.max_size - fc.min_size, 1e-6)
            center = (max_dim - fc.min_size) / span
            size_score = float(1.0 - abs(center - 0.5) * 2.0)
            size_score = max(size_score, 0.0)

        # 权重：点数 0.8 + 尺寸 0.2（点数是最可靠的区分特征）
        return 0.8 * pts_score + 0.2 * size_score

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

        if self.cluster_config.backend.lower() == "sklearn":
            labels = self._cluster_sklearn(points_zip)
        else:
            labels = self._cluster_open3d(points_zip)

        clusters = []
        unique_labels = set(labels)
        unique_labels.discard(-1)  # 去除噪声点

        for label in unique_labels:
            cluster_points = points[labels == label]
            clusters.extend(self._maybe_split_cluster(cluster_points))

        return clusters

    def filter_targets(self, clusters: List[np.ndarray]) -> List[AirTarget]:
        """筛选空中目标 — 根据点数和尺寸过滤"""
        targets = []
        rejected = []   # 诊断用：记录被拒绝的簇
        fc = self.filter_config

        for cluster_points in clusters:
            num_points = len(cluster_points)
            center = cluster_points.mean(axis=0)
            bbox_min = cluster_points.min(axis=0)
            bbox_max = cluster_points.max(axis=0)
            size = bbox_max - bbox_min
            max_dim = float(size.max())

            # 点数过滤
            if num_points < fc.min_points:
                rejected.append(f'n={num_points} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.2f}) REJECT:too_few(<{fc.min_points})')
                continue
            if num_points > fc.max_points:
                rejected.append(f'n={num_points} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.2f}) REJECT:too_many(>{fc.max_points})')
                continue

            # 尺寸过滤（取最大维度）
            if max_dim < fc.min_size:
                rejected.append(f'n={num_points} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.2f}) REJECT:too_small({max_dim:.2f}<{fc.min_size})')
                continue
            if max_dim > fc.max_size:
                rejected.append(f'n={num_points} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.2f}) size=({size[0]:.2f},{size[1]:.2f},{size[2]:.2f}) REJECT:too_big({max_dim:.2f}>{fc.max_size})')
                continue

            confidence = self._compute_confidence(num_points, size)
            if confidence < fc.confidence_threshold:
                rejected.append(f'n={num_points} center=({center[0]:.1f},{center[1]:.1f},{center[2]:.2f}) REJECT:low_conf({confidence:.2f}<{fc.confidence_threshold})')
                continue

            target = AirTarget(
                center=center,
                bbox_min=bbox_min,
                bbox_max=bbox_max,
                num_points=num_points,
                size=size,
                points=cluster_points,
                confidence=confidence,
            )
            targets.append(target)

        # 将拒绝信息附加到类属性，供上层诊断日志使用
        self._last_rejected = rejected
        return targets

    def detect(self, pcd: o3d.geometry.PointCloud) -> List[AirTarget]:
        """完整检测流程：聚类 → 筛选"""
        clusters = self.cluster(pcd)
        targets = self.filter_targets(clusters)
        return targets

    def loose_query(self, pcd: o3d.geometry.PointCloud,
                    center: np.ndarray,
                    expand_xy: float,
                    expand_z: float,
                    eps_scale: float = 1.4,
                    min_samples_scale: float = 0.6) -> List[AirTarget]:
        """在丢失目标附近做松弛查询"""
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return []

        c = np.asarray(center, dtype=np.float64)
        mask = (
            (points[:, 0] >= c[0] - expand_xy) & (points[:, 0] <= c[0] + expand_xy) &
            (points[:, 1] >= c[1] - expand_xy) & (points[:, 1] <= c[1] + expand_xy) &
            (points[:, 2] >= c[2] - expand_z) & (points[:, 2] <= c[2] + expand_z)
        )
        idx = np.flatnonzero(mask)
        if len(idx) < max(4, int(self.cluster_config.min_samples * min_samples_scale)):
            return []

        local = o3d.geometry.PointCloud()
        local.points = o3d.utility.Vector3dVector(points[idx])

        old_eps = self.cluster_config.eps
        old_ms = self.cluster_config.min_samples
        try:
            self.cluster_config.eps = old_eps * eps_scale
            self.cluster_config.min_samples = max(3, int(old_ms * min_samples_scale))
            targets = self.detect(local)
            for t in targets:
                t.source = "loose_query"
            return targets
        finally:
            self.cluster_config.eps = old_eps
            self.cluster_config.min_samples = old_ms

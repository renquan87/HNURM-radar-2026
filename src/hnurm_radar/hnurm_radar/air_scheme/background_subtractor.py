"""
background_subtractor.py — 空中目标背景减除模块（点云地图版）

基于点云地图（map 坐标系）的背景减除方案。

核心思路：
  点云地图（registration/pcds.pcd / lab_pcds.pcd）天然在 map 坐标系下，是静态场景的精确描述。
  实时点云通过 ICP/KISS-ICP 提供的 livox→map TF 变换到 map 坐标系后，
  与点云地图做最近邻匹配——距离 < 阈值的点即为背景，删除之。

  **不再需要赛前录制背景 PCD**，直接复用已有的点云地图。

优点（相比旧版 livox 帧体素方案）：
  1. 背景模型在 map 帧下一次性建立，TF 漂移不影响背景模型本身
  2. ICP 持续修正 TF，实时点云 → map 的变换始终准确
  3. 不需要"膨胀体素"来吸收震动/漂移
  4. 无需赛前额外录制背景，减少上场流程

坐标系说明：
  map 帧 = 赛场/实验室世界坐标系（X 沿赛场长边，Y 沿短边，Z 向上）
  livox 帧 = 雷达本体坐标系（X=前方, Y=左方, Z=上方）
  TF (livox→map) 由 registration 节点通过 ICP 配准持续提供

性能：
  使用 scipy.spatial.cKDTree 做最近邻查找，O(N log M) 复杂度
  对 ~50k 点查询 ~100k 背景点的 KDTree，耗时 ~3-5ms
"""

import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from typing import Optional
import logging

logger = logging.getLogger(__name__)


class AirBackgroundSubtractor:
    """基于点云地图（map 坐标系）的背景减除器

    工作流：
      1. 初始化时加载点云地图 → 构建 KDTree（一次性，map 帧）
      2. 每帧 filter() 时：
         a. 将实时点云通过 TF 变换到 map 帧
         b. KDTree 最近邻查询，距离 < threshold 的标记为背景
         c. 保留前景点（仍在 livox 帧，不修改原始坐标）
    """

    def __init__(
        self,
        bg_threshold:     float = 0.15,
        voxel_size:       float = 0.10,
        flight_z_min:     float = None,   # map 帧下飞行区域 Z 下界（可选，用于在线学习兼容）
        flight_z_max:     float = None,   # map 帧下飞行区域 Z 上界（可选）
    ):
        """
        Args:
            bg_threshold:  背景判定距离阈值 (m)；实时点到地图最近点距离 < 此值 → 视为背景
                           推荐 0.10~0.20m；越小越精准但对 TF 精度要求越高
            voxel_size:    地图加载时的降采样体素大小 (m)；越小地图越精细但 KDTree 越大
                           推荐 0.05~0.15m
            flight_z_min:  保留参数，供在线学习模式兼容（新方案中通常不使用）
            flight_z_max:  保留参数，供在线学习模式兼容
        """
        self.bg_threshold = bg_threshold
        self.voxel_size = voxel_size
        self.flight_z_min = flight_z_min
        self.flight_z_max = flight_z_max

        # 背景模型（map 坐标系下）
        self._bg_tree: Optional[cKDTree] = None
        self._bg_points: Optional[np.ndarray] = None
        self._map_point_count: int = 0

        # 状态
        self.is_ready = False

        # ---- 在线学习兼容（旧接口，新方案中不推荐使用） ----
        self.voxel_counts: dict = {}
        self.frame_count = 0
        self._bg_cache: Optional[np.ndarray] = None
        # 在线学习模式下的占据阈值（load_from_map 后不会使用）
        self.occupy_threshold = 5
        self.learning_frames = 0

    # ------------------------------------------------------------------ #
    # 从点云地图加载背景（推荐入口）
    # ------------------------------------------------------------------ #
    def load_from_map(self, pcd: o3d.geometry.PointCloud,
                      roi_min: np.ndarray = None,
                      roi_max: np.ndarray = None) -> int:
        """从点云地图加载背景模型（map 坐标系），构建 KDTree。

        点云地图本身就在 map 坐标系下，无需任何坐标变换。
        调用此方法后 is_ready = True。

        Args:
            pcd:      Open3D 点云对象（map 坐标系下的点云地图）
            roi_min:  可选 ROI 下界 [x_min, y_min, z_min]（map 帧，裁剪减少内存）
            roi_max:  可选 ROI 上界 [x_max, y_max, z_max]（map 帧）

        Returns:
            加载的背景点数量
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            logger.warning("点云地图为空，背景减除将不可用")
            return 0

        # 可选 ROI 裁剪（在 map 帧下，减少 KDTree 大小）
        if roi_min is not None and roi_max is not None:
            roi_min = np.asarray(roi_min, dtype=np.float64)
            roi_max = np.asarray(roi_max, dtype=np.float64)
            mask = np.all((points >= roi_min) & (points <= roi_max), axis=1)
            points = points[mask]
            if len(points) == 0:
                logger.warning("ROI 裁剪后点云地图为空")
                return 0

        # 降采样（减少 KDTree 查询开销）
        if self.voxel_size > 0:
            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector(points)
            temp_pcd = temp_pcd.voxel_down_sample(self.voxel_size)
            points = np.asarray(temp_pcd.points)

        if len(points) == 0:
            logger.warning("降采样后点云地图为空")
            return 0

        # 构建 KDTree
        self._bg_points = points.astype(np.float64)
        self._bg_tree = cKDTree(self._bg_points)
        self._map_point_count = len(points)
        self.is_ready = True

        logger.info(
            f"点云地图背景加载完成: {self._map_point_count} 点, "
            f"voxel={self.voxel_size}m, threshold={self.bg_threshold}m"
        )
        return self._map_point_count

    # ------------------------------------------------------------------ #
    # 核心：在 map 帧下做背景减除
    # ------------------------------------------------------------------ #
    def filter(self, pcd: o3d.geometry.PointCloud,
               radar_to_field: np.ndarray = None) -> o3d.geometry.PointCloud:
        """过滤背景点，只保留前景（动态/空中）点。

        新方案核心流程：
          1. 将 livox 帧实时点云通过 TF 变换到 map 帧
          2. 在 map 帧用 KDTree 查询最近邻距离
          3. 距离 < threshold 的标记为背景
          4. 返回 livox 帧下的前景点云（坐标不变）

        Args:
            pcd:             livox 帧下的实时点云
            radar_to_field:  4x4 齐次变换矩阵 (livox → map)，来自 TF
                             如果为 None，回退到旧版 livox 帧体素方案

        Returns:
            livox 帧下的前景点云
        """
        if not self.is_ready:
            return pcd

        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd

        # ---- 新方案：map 帧 KDTree 背景减除 ----
        if self._bg_tree is not None and radar_to_field is not None:
            # 将实时点云变换到 map 帧
            ones = np.ones((len(points), 1), dtype=np.float64)
            pts_homo = np.hstack([points, ones])  # (N, 4)
            pts_map = (radar_to_field @ pts_homo.T).T[:, :3]  # (N, 3)

            # KDTree 最近邻查询
            distances, _ = self._bg_tree.query(pts_map, k=1)

            # 距离 > 阈值 = 前景（非背景）
            mask = distances > self.bg_threshold

            foreground = o3d.geometry.PointCloud()
            foreground.points = o3d.utility.Vector3dVector(points[mask])
            return foreground

        # ---- 旧方案回退：livox 帧体素查表 ----
        return self._filter_voxel_legacy(pcd)

    # ------------------------------------------------------------------ #
    # 旧版 livox 帧体素方案（兼容接口）
    # ------------------------------------------------------------------ #
    def _filter_voxel_legacy(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        """旧版体素背景减除（livox 帧，仅在无 TF 时回退使用）"""
        if not self.voxel_counts:
            return pcd

        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd

        bg_sorted = self._get_expanded_bg()
        if bg_sorted is None or len(bg_sorted) == 0:
            return pcd

        voxel_keys = (points / (self.bg_threshold if self.bg_threshold > 0 else 0.15)).astype(np.int32)
        encoded = (voxel_keys[:, 0].astype(np.int64) * 1000003 +
                   voxel_keys[:, 1].astype(np.int64) * 1000033 +
                   voxel_keys[:, 2].astype(np.int64))

        idx = np.searchsorted(bg_sorted, encoded)
        idx_clipped = np.clip(idx, 0, len(bg_sorted) - 1)
        is_bg = bg_sorted[idx_clipped] == encoded
        mask = ~is_bg

        foreground = o3d.geometry.PointCloud()
        foreground.points = o3d.utility.Vector3dVector(points[mask])
        return foreground

    def _invalidate_cache(self):
        """标记 BG 缓存失效"""
        self._bg_cache = None

    def _get_expanded_bg(self) -> Optional[np.ndarray]:
        """获取扩展背景体素编码（旧版兼容）"""
        if self._bg_cache is not None:
            return self._bg_cache

        bg_list = [(k[0], k[1], k[2]) for k, c in self.voxel_counts.items()
                   if c >= self.occupy_threshold]
        if not bg_list:
            return None

        bg_voxels = np.array(bg_list, dtype=np.int64)
        offsets = np.array([
            [dx, dy, 0]
            for dx in [-1, 0, 1]
            for dy in [-1, 0, 1]
        ], dtype=np.int64)

        expanded = bg_voxels[:, np.newaxis, :] + offsets[np.newaxis, :, :]
        expanded = expanded.reshape(-1, 3)

        bg_encoded = (expanded[:, 0] * 1000003 +
                      expanded[:, 1] * 1000033 +
                      expanded[:, 2])
        bg_sorted = np.unique(bg_encoded)

        self._bg_cache = bg_sorted
        return bg_sorted

    # ------------------------------------------------------------------ #
    # 在线学习（旧接口兼容，新方案中不推荐使用）
    # ------------------------------------------------------------------ #
    def learn(self, pcd: o3d.geometry.PointCloud):
        """在线学习背景帧（旧版兼容接口）。

        新方案中此方法为空操作（背景模型由点云地图提供）。
        仅在未加载点云地图时才启用在线学习。
        """
        if self._bg_tree is not None:
            # 已加载点云地图，不需要在线学习
            return

        if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
            self.is_ready = True
            return

        points = np.asarray(pcd.points)
        if len(points) == 0:
            self.frame_count += 1
            if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
                self.is_ready = True
            return

        if self.flight_z_min is not None and self.flight_z_max is not None:
            flight_mask = (points[:, 2] >= self.flight_z_min) & (points[:, 2] <= self.flight_z_max)
            learn_points = points[~flight_mask]
        else:
            learn_points = points

        if len(learn_points) > 0:
            vs = self.bg_threshold if self.bg_threshold > 0 else 0.15
            voxel_keys = (learn_points / vs).astype(np.int32)
            encoded = (voxel_keys[:, 0].astype(np.int64) * 1000003 +
                       voxel_keys[:, 1].astype(np.int64) * 1000033 +
                       voxel_keys[:, 2].astype(np.int64))
            unique_encoded, unique_idx = np.unique(encoded, return_index=True)
            unique_voxels = voxel_keys[unique_idx]
            for vk in unique_voxels:
                key = (int(vk[0]), int(vk[1]), int(vk[2]))
                self.voxel_counts[key] = self.voxel_counts.get(key, 0) + 1
            self._invalidate_cache()

        self.frame_count += 1
        if self.learning_frames > 0 and self.frame_count >= self.learning_frames:
            self.is_ready = True

    # ------------------------------------------------------------------ #
    # 旧版 PCD 加载（保留兼容，不推荐）
    # ------------------------------------------------------------------ #
    def load_from_pcd(self, pcd: o3d.geometry.PointCloud,
                      use_all_points: bool = True) -> int:
        """从预建点云加载背景模型（旧版 livox 帧方案，保留兼容）。

        ⚠ 新方案请使用 load_from_map()（map 帧 KDTree 方案）。

        Returns:
            新增背景体素数量
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return 0

        if not use_all_points and self.flight_z_min is not None and self.flight_z_max is not None:
            flight_mask = (points[:, 2] >= self.flight_z_min) & (points[:, 2] <= self.flight_z_max)
            points = points[~flight_mask]

        if len(points) == 0:
            return 0

        vs = self.bg_threshold if self.bg_threshold > 0 else 0.15
        voxel_keys = (points / vs).astype(np.int32)
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

        self.is_ready = True
        self._invalidate_cache()
        if self.learning_frames > 0:
            self.frame_count = self.learning_frames
        return added

    @property
    def background_voxel_count(self) -> int:
        """已确认为背景的元素数量（地图模式返回地图点数，旧模式返回体素数）"""
        if self._bg_tree is not None:
            return self._map_point_count
        return sum(1 for c in self.voxel_counts.values()
                   if c >= self.occupy_threshold)

    @property
    def mode(self) -> str:
        """当前背景减除模式"""
        if self._bg_tree is not None:
            return "map_kdtree"
        if self.voxel_counts:
            return "voxel_legacy"
        return "none"

    def reset(self):
        """重置背景模型（场景切换时调用）"""
        self.voxel_counts = {}
        self.frame_count = 0
        self._invalidate_cache()
        self._bg_tree = None
        self._bg_points = None
        self._map_point_count = 0
        self.is_ready = False

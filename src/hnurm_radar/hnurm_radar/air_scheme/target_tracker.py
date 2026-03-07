"""
target_tracker.py — 空中目标跟踪器

严格对齐 HITS-radar-2024 TargetMap.cpp 实现。

核心功能（全部实现 HITS 特性）：
1. 卡尔曼滤波平滑（含 cov_factor/stop_p_time，由 AirKalmanFilter 提供）
2. 马氏距离匹配（贪心算法，与 HITS push() 一致）
3. 强制合并 force_combine（HITS force_combine_dist）
4. 目标分裂检测 separate（HITS TM_NEED_SEPERATE → 使用更严格参数重聚类）
5. 松弛查询 loose_query（HITS loose_query，由外部 cluster_detector 支持）
6. 确认窗口 confirm_frames：只有连续观测≥N帧的目标才发布，减少噪声
7. 丢失计数：超过 max_lost_frames 才删除
8. HITS combine_limit / separate_limit 机制

注意：
  separate 逻辑依赖外部传入完整点云（pcd_for_detect），
  当 aabb 内新旧目标重叠时，用更严格的 eps/min_samples 重聚类。
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import open3d as o3d

from .air_kalman_filter import AirKalmanFilter
from .cluster_detector import AirTarget, AirClusterDetector


# ---- 常量（对应 HITS TM_NOISE / TM_NEED_SEPERATE）----
TM_NOISE = -1
TM_NEED_SEPARATE = -2

# 默认严格聚类参数（用于 separate 内部重聚类）
_STRICT_EPS_FACTOR    = 0.5   # strict_eps = normal_eps * 0.5
_STRICT_MIN_PTS_FACTOR = 2.0  # strict_min_pts = normal_min_pts * 2


@dataclass
class TrackedAirTarget:
    """被跟踪的空中目标（对应 HITS Target 结构）"""
    id:              int
    kf:             AirKalmanFilter
    lost_count:     int   = 0
    total_seen:     int   = 0
    last_detection: Optional[AirTarget] = None
    height:         float = 0.0
    num_points:     int   = 0
    aabb_min:       np.ndarray = field(default_factory=lambda: np.zeros(3))
    aabb_max:       np.ndarray = field(default_factory=lambda: np.zeros(3))
    confidence:     float = 0.0

    @property
    def pos(self) -> np.ndarray:
        """卡尔曼估计位置 [x, y]"""
        return self.kf.pos

    @property
    def pos3d(self) -> np.ndarray:
        """带高度的三维位置"""
        return np.array([self.kf.pos[0], self.kf.pos[1], self.height])

    @property
    def velocity(self) -> np.ndarray:
        return self.kf.velocity

    @property
    def is_confirmed(self) -> bool:
        """目标已通过确认窗口（连续观测足够次数）"""
        return self.total_seen >= 2 and self.lost_count == 0

    def update_aabb(self, det: AirTarget):
        if det.bbox_min is not None and det.bbox_max is not None:
            self.aabb_min = det.bbox_min.copy()
            self.aabb_max = det.bbox_max.copy()


class AirTargetTracker:
    """空中目标跟踪器（严格对齐 HITS TargetMap）"""

    def __init__(
        self,
        max_lost_frames:          int   = 120,
        match_distance_threshold: float = 6.0,
        force_combine_dist:       float = 0.5,
        cc_thres:                 float = 0.05,
        combine_limit:            int   = 15,
        separate_limit:           int   = 8,
        confirm_frames:           int   = 2,
        kf_params: Optional[dict] = None,
    ):
        """
        Args:
            max_lost_frames:          丢失超过此帧数才删除（HITS last_frames）
            match_distance_threshold: 马氏距离阈值（HITS dist_thres²，此处直接为距离）
            force_combine_dist:       强制合并距离 (m)
            cc_thres:                 包围盒接触阈值 (m)（HITS cc_thres）
            combine_limit:            跟踪目标被合并前的最大丢失帧数
            separate_limit:           错误合并判定的最大丢失帧数
            confirm_frames:           确认窗口帧数（total_seen >= 此值才发布）
            kf_params:                卡尔曼参数（可 None，使用全局 set_global_params）
        """
        self.targets:         Dict[int, TrackedAirTarget] = {}
        self.next_id          = 0
        self.max_lost_frames  = max_lost_frames
        self.match_threshold  = match_distance_threshold
        self.force_combine_dist = force_combine_dist
        self.cc_thres         = cc_thres
        self.combine_limit    = combine_limit
        self.separate_limit   = separate_limit
        self.confirm_frames   = confirm_frames
        self.kf_params        = kf_params or {}

        # 注册全局卡尔曼参数（只有第一次调用生效）
        if kf_params:
            AirKalmanFilter.set_global_params(**kf_params)

    # ------------------------------------------------------------------ #
    # 辅助方法
    # ------------------------------------------------------------------ #
    def _create_kf(self, pos: np.ndarray) -> AirKalmanFilter:
        """创建卡尔曼滤波器（使用全局参数或实例参数）"""
        if self.kf_params:
            return AirKalmanFilter(pos, **self.kf_params)
        return AirKalmanFilter(pos)

    @staticmethod
    def _aabb_coincide(min1: np.ndarray, max1: np.ndarray,
                       min2: np.ndarray, max2: np.ndarray,
                       thres: float = 0.0) -> bool:
        """检查两个 AABB 是否相交（HITS is_coincide）"""
        return (
            min1[0] <= max2[0] + thres and max1[0] >= min2[0] - thres and
            min1[1] <= max2[1] + thres and max1[1] >= min2[1] - thres
        )

    @staticmethod
    def _aabb_contains_pos(pos: np.ndarray,
                           aabb_min: np.ndarray, aabb_max: np.ndarray) -> bool:
        """检查位置是否在 AABB 内部（HITS is_contain）"""
        return (
            aabb_min[0] <= pos[0] <= aabb_max[0] and
            aabb_min[1] <= pos[1] <= aabb_max[1]
        )

    # ------------------------------------------------------------------ #
    # 强制合并（HITS combine_force）
    # ------------------------------------------------------------------ #
    def _force_combine(self):
        """将距离过近的目标强制合并，保留 total_seen 更多的一个。"""
        ids = list(self.targets.keys())
        to_remove: set = set()
        for i in range(len(ids)):
            if ids[i] in to_remove:
                continue
            for j in range(i + 1, len(ids)):
                if ids[j] in to_remove:
                    continue
                t1 = self.targets[ids[i]]
                t2 = self.targets[ids[j]]
                p1 = np.array([t1.pos[0], t1.pos[1], t1.height])
                p2 = np.array([t2.pos[0], t2.pos[1], t2.height])
                dist = float(np.linalg.norm(p1 - p2))
                if dist < self.force_combine_dist:
                    if t1.total_seen >= t2.total_seen:
                        to_remove.add(ids[j])
                    else:
                        to_remove.add(ids[i])
                        break
        for tid in to_remove:
            del self.targets[tid]

    # ------------------------------------------------------------------ #
    # 分裂检测（HITS seperate）
    # ------------------------------------------------------------------ #
    def _try_separate(
        self,
        aabb_min: np.ndarray,
        aabb_max: np.ndarray,
        pcd: o3d.geometry.PointCloud,
        detector: Optional['AirClusterDetector'],
    ) -> bool:
        """当一个聚类 AABB 与多个已有目标重叠时，尝试分裂聚类。

        使用更严格的 DBSCAN 参数（eps × 0.5，min_pts × 2）在包围盒内重聚类。
        返回 True 表示进行了分裂处理。
        """
        if detector is None or pcd is None or len(pcd.points) == 0:
            return False

        points = np.asarray(pcd.points)
        mask = (
            (points[:, 0] >= aabb_min[0]) & (points[:, 0] <= aabb_max[0]) &
            (points[:, 1] >= aabb_min[1]) & (points[:, 1] <= aabb_max[1])
        )
        local_pts = points[mask]
        if len(local_pts) < detector.cluster_config.min_samples * 2:
            return False

        pcd_local = o3d.geometry.PointCloud()
        pcd_local.points = o3d.utility.Vector3dVector(local_pts)

        # 临时收紧聚类参数
        old_eps = detector.cluster_config.eps
        old_ms  = detector.cluster_config.min_samples
        try:
            detector.cluster_config.eps        = old_eps * _STRICT_EPS_FACTOR
            detector.cluster_config.min_samples = max(
                old_ms, int(old_ms * _STRICT_MIN_PTS_FACTOR)
            )
            sub_targets = detector.detect(pcd_local)
        finally:
            detector.cluster_config.eps        = old_eps
            detector.cluster_config.min_samples = old_ms

        if len(sub_targets) <= 1:
            return False   # 分裂失败

        # 将分裂出来的子目标推入跟踪器（no_strict 模式）
        for sub in sub_targets:
            self._push_single(sub, no_strict=True, pcd=pcd, detector=detector)
        return True

    # ------------------------------------------------------------------ #
    # 匹配单个检测目标到已有跟踪目标（HITS push）
    # ------------------------------------------------------------------ #
    def _push_single(
        self,
        det: AirTarget,
        no_strict: bool = False,
        pcd: Optional[o3d.geometry.PointCloud] = None,
        detector: Optional['AirClusterDetector'] = None,
    ) -> int:
        """匹配一个检测目标到已有跟踪目标，或创建新目标。

        Returns:
            匹配到的 track_id，或 TM_NEED_SEPARATE / TM_NOISE
        """
        grav2d = det.center[:2]
        det_min = det.bbox_min[:2] if det.bbox_min is not None else grav2d - 0.5
        det_max = det.bbox_max[:2] if det.bbox_max is not None else grav2d + 0.5

        min_id: int  = -1
        min_dis: float = -1.0
        combine_force_ids: List[int] = []
        contain_ids:       List[int] = []   # 与检测 AABB 包含关系的跟踪目标

        for tid, target in self.targets.items():
            # 检测 AABB 是否包含该跟踪目标位置
            if self._aabb_contains_pos(target.pos, det_min, det_max):
                if target.lost_count < self.separate_limit:
                    contain_ids.append(tid)

            mah = target.kf.mahalanobis_distance(grav2d)

            # 过远且不重叠 → 不考虑
            aabb_overlap = self._aabb_coincide(
                det_min, det_max, target.aabb_min[:2], target.aabb_max[:2],
                thres=self.cc_thres
            )
            if mah > self.match_threshold ** 2 and not aabb_overlap:
                continue

            # 强制合并候选
            dist_m = float(np.linalg.norm(target.pos - grav2d))
            if dist_m <= self.force_combine_dist:
                combine_force_ids.append(tid)

            if mah < min_dis or min_id == -1:
                min_id  = tid
                min_dis = mah

        # 检测到多个已有目标被包含 → 需要分裂
        if len(contain_ids) > 1 and not no_strict:
            if self._try_separate(det_min, det_max, pcd, detector):
                return TM_NEED_SEPARATE

        if min_id != -1:
            # 强制合并（保留 min_id，删除其他）
            if len(combine_force_ids) > 1:
                for tid in combine_force_ids:
                    if tid != min_id and tid in self.targets:
                        del self.targets[tid]
            # 更新目标
            t = self.targets[min_id]
            t.kf.update(grav2d)
            t.lost_count  = 0
            t.total_seen  += 1
            t.last_detection = det
            t.height      = det.center[2]
            t.num_points  = det.num_points
            t.confidence  = det.confidence
            t.update_aabb(det)
            return min_id

        # 无匹配 → 创建新目标
        return self._new_target(det)

    def _new_target(self, det: AirTarget) -> int:
        new_id = self.next_id
        self.next_id += 1
        kf = self._create_kf(det.center[:2])
        t  = TrackedAirTarget(
            id=new_id, kf=kf, lost_count=0, total_seen=1,
            last_detection=det,
            height=det.center[2],
            num_points=det.num_points,
            confidence=det.confidence,
        )
        t.update_aabb(det)
        self.targets[new_id] = t
        return new_id

    # ------------------------------------------------------------------ #
    # 松弛查询（HITS loose_query）
    # ------------------------------------------------------------------ #
    def _do_loose_query(
        self,
        pcd: o3d.geometry.PointCloud,
        detector: 'AirClusterDetector',
        expand_xy: float,
        expand_z:  float,
        eps_scale:        float,
        min_samples_scale: float,
    ):
        """对丢失时间较短的目标在其周围做宽松聚类搜索（HITS loose_query）"""
        for tid, target in list(self.targets.items()):
            if target.lost_count == 0 or target.lost_count > self.combine_limit:
                continue
            center = np.array([target.pos[0], target.pos[1], target.height])
            extra = detector.loose_query(
                pcd, center, expand_xy, expand_z, eps_scale, min_samples_scale
            )
            if extra:
                # 取置信度最高的那个候选，直接更新目标
                best = max(extra, key=lambda x: x.confidence)
                best_pos2d = best.center[:2]
                dist = float(np.linalg.norm(target.pos - best_pos2d))
                if dist <= expand_xy * 1.5:
                    target.kf.update(best_pos2d)
                    target.lost_count  = 0
                    target.total_seen  += 1
                    target.last_detection = best
                    target.height      = best.center[2]
                    target.num_points  = best.num_points
                    target.confidence  = best.confidence
                    target.update_aabb(best)

    # ------------------------------------------------------------------ #
    # 主更新接口
    # ------------------------------------------------------------------ #
    def update(
        self,
        detections: List[AirTarget],
        pcd:      Optional[o3d.geometry.PointCloud]  = None,
        detector: Optional['AirClusterDetector']     = None,
        loose_query_params: Optional[dict]           = None,
    ) -> List[TrackedAirTarget]:
        """更新跟踪器，返回已确认的活跃目标列表。

        Args:
            detections:          当帧检测目标列表
            pcd:                 原始预处理后的 Open3D 点云（供 separate/loose_query 使用）
            detector:            AirClusterDetector 实例（供 separate/loose_query 使用）
            loose_query_params:  松弛查询参数字典（含 expand_xy/expand_z 等），None 则跳过

        Returns:
            confirmed_active: 已确认且本帧（或刚丢失≤1帧）的目标列表
        """
        # 0. 每帧开始前，重置 pt_num（HITS pre_update 等效）
        #    此处跳过，因 total_seen 代替了 pt_num 的语义

        matched_target_ids: set = set()

        # 1. 逐检测目标贪心匹配
        for det in detections:
            ret = self._push_single(det, pcd=pcd, detector=detector)
            if ret >= 0:
                matched_target_ids.add(ret)

        # 2. 对未被匹配到的跟踪目标做松弛查询再尝试恢复
        if pcd is not None and detector is not None and loose_query_params:
            self._do_loose_query(
                pcd, detector,
                expand_xy=loose_query_params.get("expand_xy", 1.0),
                expand_z=loose_query_params.get("expand_z", 0.5),
                eps_scale=loose_query_params.get("eps_scale", 1.4),
                min_samples_scale=loose_query_params.get("min_samples_scale", 0.6),
            )

        # 3. 未匹配的目标做无观测预测更新（HITS post_update）
        for tid, target in list(self.targets.items()):
            if tid not in matched_target_ids:
                target.kf.update_no_observation()
                target.lost_count += 1

        # 4. 强制合并
        self._force_combine()

        # 5. 删除丢失过久的目标（HITS last_frames）
        to_remove = [
            tid for tid, t in self.targets.items()
            if t.lost_count > self.max_lost_frames
        ]
        for tid in to_remove:
            del self.targets[tid]

        # 6. 返回已确认的活跃目标（strict: total_seen >= confirm_frames）
        return [
            t for t in self.targets.values()
            if t.total_seen >= self.confirm_frames and t.lost_count == 0
        ]

    # ------------------------------------------------------------------ #
    # 辅助查询接口
    # ------------------------------------------------------------------ #
    def get_all_targets(self) -> List[TrackedAirTarget]:
        """返回所有跟踪中的目标（含丢失状态）"""
        return list(self.targets.values())

    def get_confirmed_targets(self) -> List[TrackedAirTarget]:
        """返回所有已确认目标（含丢失状态，用于松弛查询候选）"""
        return [t for t in self.targets.values() if t.total_seen >= self.confirm_frames]

    def get_recent_lost_targets(self, max_lost_frames: int = 5) -> List[TrackedAirTarget]:
        """返回近期丢失的目标（供外部松弛查询）"""
        return [
            t for t in self.targets.values()
            if 0 < t.lost_count <= max_lost_frames and t.last_detection is not None
            and t.total_seen >= self.confirm_frames
        ]

    def reset(self):
        self.targets.clear()
        self.next_id = 0

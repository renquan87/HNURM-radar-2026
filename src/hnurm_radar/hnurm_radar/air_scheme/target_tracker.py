"""
target_tracker.py — 空中目标跟踪器

适配自 air_target_radar（深度参考 HITS-radar-2024 TargetMap 实现）。
核心功能：
- 卡尔曼滤波平滑（含 cov_factor, stop_p_time）
- 马氏距离匹配（跨帧关联）
- 目标 ID 持久化
- 丢失计数与自动删除
- 目标强制合并（force_combine_dist，HITS 特性）
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
from .air_kalman_filter import AirKalmanFilter
from .cluster_detector import AirTarget


@dataclass
class TrackedAirTarget:
    """被跟踪的空中目标"""
    id: int
    kf: AirKalmanFilter
    lost_count: int = 0
    total_seen: int = 0
    last_detection: Optional[AirTarget] = None
    height: float = 0.0
    num_points: int = 0
    aabb_min: np.ndarray = field(default_factory=lambda: np.zeros(3))
    aabb_max: np.ndarray = field(default_factory=lambda: np.zeros(3))

    @property
    def pos(self) -> np.ndarray:
        return self.kf.pos

    @property
    def velocity(self) -> np.ndarray:
        return self.kf.velocity

    @property
    def is_confirmed(self) -> bool:
        return self.total_seen >= 2

    def update_aabb(self, det: AirTarget):
        """更新包围盒"""
        if det.bbox_min is not None and det.bbox_max is not None:
            self.aabb_min = det.bbox_min.copy()
            self.aabb_max = det.bbox_max.copy()


class AirTargetTracker:
    """空中目标跟踪器"""

    def __init__(self,
                 max_lost_frames: int = 10,
                 match_distance_threshold: float = 50.0,
                 force_combine_dist: float = 0.5,
                 cc_thres: float = 0.3,
                 kf_params: Optional[dict] = None):
        self.targets: Dict[int, TrackedAirTarget] = {}
        self.next_id = 0
        self.max_lost_frames = max_lost_frames
        self.match_threshold = match_distance_threshold
        self.force_combine_dist = force_combine_dist
        self.cc_thres = cc_thres
        self.kf_params = kf_params or {}

    def _create_kf(self, pos: np.ndarray) -> AirKalmanFilter:
        return AirKalmanFilter(pos, **self.kf_params)

    def _force_combine(self):
        """强制合并距离过近的目标（HITS 特性）"""
        ids = list(self.targets.keys())
        to_remove = set()
        for i in range(len(ids)):
            if ids[i] in to_remove:
                continue
            for j in range(i + 1, len(ids)):
                if ids[j] in to_remove:
                    continue
                t1 = self.targets[ids[i]]
                t2 = self.targets[ids[j]]
                dist = np.linalg.norm(t1.pos - t2.pos)
                if dist < self.force_combine_dist:
                    if t1.total_seen >= t2.total_seen:
                        to_remove.add(ids[j])
                    else:
                        to_remove.add(ids[i])
                        break
        for tid in to_remove:
            del self.targets[tid]

    def _match_detections(self, detections: List[AirTarget]
                          ) -> Tuple[Dict[int, int], List[int]]:
        """贪心匹配：检测结果 ↔ 已有跟踪目标"""
        if not detections or not self.targets:
            return {}, list(range(len(detections)))

        det_centers = [d.center[:2] for d in detections]
        target_ids = list(self.targets.keys())

        # 马氏距离代价矩阵
        cost_matrix = np.zeros((len(detections), len(target_ids)))
        for i, dc in enumerate(det_centers):
            for j, tid in enumerate(target_ids):
                cost_matrix[i, j] = self.targets[tid].kf.mahalanobis_distance(dc)

        # 贪心匹配
        matched = {}
        used_targets = set()
        used_dets = set()

        pairs = []
        for i in range(len(detections)):
            for j in range(len(target_ids)):
                pairs.append((cost_matrix[i, j], i, j))
        pairs.sort(key=lambda x: x[0])

        for cost, di, tj in pairs:
            if di in used_dets or tj in used_targets:
                continue
            if cost > self.match_threshold:
                break
            matched[di] = target_ids[tj]
            used_dets.add(di)
            used_targets.add(tj)

        unmatched = [i for i in range(len(detections)) if i not in matched]
        return matched, unmatched

    def update(self, detections: List[AirTarget]) -> List[TrackedAirTarget]:
        """更新跟踪器，返回活跃目标列表"""
        # 1. 匹配
        matched, unmatched = self._match_detections(detections)

        # 2. 更新已匹配的目标
        updated_ids = set()
        for det_idx, target_id in matched.items():
            det = detections[det_idx]
            target = self.targets[target_id]
            target.kf.update(det.center[:2])
            target.lost_count = 0
            target.total_seen += 1
            target.last_detection = det
            target.height = det.center[2]  # 原始雷达Z坐标，后续TF变换得到实际高度
            target.num_points = det.num_points
            target.update_aabb(det)
            updated_ids.add(target_id)

        # 3. 未匹配的检测 → 新目标
        for det_idx in unmatched:
            det = detections[det_idx]
            new_id = self.next_id
            self.next_id += 1
            kf = self._create_kf(det.center[:2])
            t = TrackedAirTarget(
                id=new_id, kf=kf, lost_count=0, total_seen=1,
                last_detection=det,
                height=det.center[2],  # 原始雷达Z坐标
                num_points=det.num_points,
            )
            t.update_aabb(det)
            self.targets[new_id] = t

        # 4. 未匹配的已有目标 → 仅预测
        for tid, target in self.targets.items():
            if tid not in updated_ids and tid < self.next_id - len(unmatched):
                target.kf.update_no_observation()
                target.lost_count += 1

        # 5. 强制合并
        self._force_combine()

        # 6. 删除丢失过久的目标
        to_remove = [tid for tid, t in self.targets.items()
                     if t.lost_count > self.max_lost_frames]
        for tid in to_remove:
            del self.targets[tid]

        # 7. 返回活跃目标
        active = [t for t in self.targets.values() if t.lost_count == 0]
        return active

    def get_all_targets(self) -> List[TrackedAirTarget]:
        return list(self.targets.values())

    def reset(self):
        self.targets.clear()
        self.next_id = 0

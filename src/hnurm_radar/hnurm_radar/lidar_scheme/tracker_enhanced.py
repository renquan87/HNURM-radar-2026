import time
import numpy as np
from typing import List, Tuple, Dict, Optional
from scipy.optimize import linear_sum_assignment
from ..filters.kalman_filter import EnhancedKalmanFilter

class TrackState:
    INACTIVE = 0
    TENTATIVE = 1
    CONFIRMED = 2
    LOST = 3
    DELETED = 4

class Slot:
    def __init__(self, slot_id: int, car_id: int, class_num: int = 12,
                 process_noise=1e-2, measurement_noise=1e-1,
                 jump_threshold=1.0, max_velocity=5.0):
        self.slot_id = slot_id
        self.car_id = car_id
        self.state = TrackState.INACTIVE
        self.raw_position = None
        self.filtered_position = None
        
        self.kf = EnhancedKalmanFilter(
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            jump_threshold=jump_threshold,
            max_velocity=max_velocity
        )
        
        self.class_dist = np.zeros(class_num, dtype=np.float32)
        self.alpha = 0.7
        self.hits = 0
        self.lost_count = 0
        self.last_update_time = time.time()
        self.has_ever_matched = False
        
        # 猜点计数器（新增）
        self.guess_count = 0
        self.max_guess_frames = 10   # 最大猜点帧数

    def update_appearance(self, class_label: int, confidence: float):
        one_hot = np.zeros_like(self.class_dist)
        one_hot[class_label] = confidence
        self.class_dist = self.alpha * self.class_dist + (1 - self.alpha) * one_hot

    def appearance_similarity(self, class_label: int) -> float:
        if np.sum(self.class_dist) == 0:
            return 0.5
        obs_vec = np.zeros_like(self.class_dist)
        obs_vec[class_label] = 1.0
        sim = np.dot(obs_vec, self.class_dist) / (
            np.linalg.norm(obs_vec) * np.linalg.norm(self.class_dist) + 1e-6)
        return sim

    def update(self, raw_pos: np.ndarray, class_label: int = None,
               confidence: float = 0.0, from_visual: bool = False):
        self.raw_position = raw_pos.copy()
        x, y = raw_pos[0], raw_pos[1]
        filtered_x, filtered_y = self.kf.update(x, y)
        self.filtered_position = np.array([filtered_x, filtered_y, raw_pos[2]])
        
        if from_visual and class_label is not None and class_label >= 0:
            self.update_appearance(class_label, confidence)
            self.has_ever_matched = True
        
        self.hits += 1
        self.lost_count = 0
        self.last_update_time = time.time()
        # 重置猜点计数器（因为收到了新观测）
        self.guess_count = 0
        
        if self.state == TrackState.TENTATIVE and self.hits >= 3:
            self.state = TrackState.CONFIRMED
        elif self.state == TrackState.LOST:
            self.state = TrackState.CONFIRMED

    def predict(self, dt: float):
        """
        仅预测位置，并更新 filtered_position（用于猜点）
        返回预测位置（赛场坐标）
        """
        if self.filtered_position is None:
            return None
        pred = self.kf.predict_only()
        if pred is not None:
            pred_x, pred_y = pred
            # 关键：更新 filtered_position 为预测值（用于猜点）
            self.filtered_position = np.array([pred_x, pred_y, self.filtered_position[2]])
            # 增加猜点计数（只有在 LOST 状态下才累加，但为了统一，每次 predict 都累加）
            # 更精确的做法是在 mark_missed 后调用 predict 才累加，但这里简单处理：
            if self.state == TrackState.LOST:
                self.guess_count += 1
            return self.filtered_position
        else:
            return self.filtered_position

    def mark_missed(self):
        self.lost_count += 1
        if self.state == TrackState.CONFIRMED:
            self.state = TrackState.LOST
            # 进入丢失状态，重置猜点计数器
            self.guess_count = 0
        elif self.state in (TrackState.TENTATIVE, TrackState.LOST) and self.lost_count > 5:
            self.state = TrackState.DELETED
        self.last_update_time = time.time()

    def get_output_position(self) -> Optional[np.ndarray]:
        if self.state == TrackState.CONFIRMED:
            return self.raw_position
        elif self.state == TrackState.LOST:
            # 如果猜点超过最大帧数，则不再输出位置（返回 None）
            if self.guess_count >= self.max_guess_frames:
                return None
            return self.filtered_position   # 此时 filtered_position 已被 predict 更新为外推值
        return None

    def reset(self):
        self.state = TrackState.INACTIVE
        self.hits = 0
        self.lost_count = 0
        self.class_dist.fill(0)
        self.raw_position = None
        self.filtered_position = None
        self.has_ever_matched = False
        self.guess_count = 0
        # 重置卡尔曼滤波器
        self.kf = EnhancedKalmanFilter(
            process_noise=self.kf.base_process_noise,
            measurement_noise=self.kf.kf.measurementNoiseCov[0,0],
            jump_threshold=self.kf.jump_threshold,
            max_velocity=self.kf.max_velocity
        )


class FixedSlotTracker:
    def __init__(self, enemy_car_ids: List[int], class_num: int = 12,
                 max_distance: float = 0.5, w_pos: float = 0.7, w_app: float = 0.3,
                 process_noise=1e-2, measurement_noise=1e-1,
                 jump_threshold=1.0, max_velocity=5.0):
        self.slots = []
        for idx, cid in enumerate(enemy_car_ids):
            self.slots.append(Slot(idx, cid, class_num,
                                   process_noise, measurement_noise,
                                   jump_threshold, max_velocity))
        self.max_distance = max_distance
        self.w_pos = w_pos
        self.w_app = w_app
        self.class_num = class_num
        self.last_update_time = time.time()

    def update(self, detections: List[Tuple[int, np.ndarray, bool, Optional[int], float]],
               current_time: float) -> List[Tuple[int, np.ndarray]]:
        dt = current_time - self.last_update_time
        dt = max(0.01, min(0.2, dt))
        self.last_update_time = current_time

        active_slots = [s for s in self.slots if s.state in (TrackState.TENTATIVE, TrackState.CONFIRMED, TrackState.LOST)]
        if not active_slots and not detections:
            return []

        # 预测所有活跃槽位（同时更新 filtered_position）
        for slot in active_slots:
            slot.predict(dt)   # 这里会更新 filtered_position（包括 LOST 状态的猜点）

        # 构建代价矩阵（使用每个槽位的 filtered_position 作为预测位置）
        n_slots = len(active_slots)
        n_dets = len(detections)
        cost_matrix = np.full((n_slots, n_dets), 1e9, dtype=np.float32)

        for i, slot in enumerate(active_slots):
            pred_pos = slot.filtered_position
            if pred_pos is None:
                continue
            for j, (det_car_id, det_pos, from_visual, class_label, conf) in enumerate(detections):
                pos_cost = np.linalg.norm(pred_pos[:2] - det_pos[:2])
                app_cost = 0.0
                if from_visual and class_label is not None and class_label >= 0:
                    sim = slot.appearance_similarity(class_label)
                    app_cost = 1 - sim
                total_cost = self.w_pos * pos_cost + self.w_app * app_cost
                if total_cost < self.max_distance * 2:
                    cost_matrix[i, j] = total_cost

        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matched_slot_idx = set()
        matched_det_idx = set()
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.max_distance:
                matched_slot_idx.add(r)
                matched_det_idx.add(c)
                slot = active_slots[r]
                det_car_id, det_pos, from_visual, class_label, conf = detections[c]
                slot.update(det_pos, class_label, conf, from_visual)

        # 未匹配的槽位标记丢失
        for i in range(n_slots):
            if i not in matched_slot_idx:
                active_slots[i].mark_missed()

        # 未匹配的视觉观测激活 INACTIVE 槽位
        for j in range(n_dets):
            if j not in matched_det_idx:
                det_car_id, det_pos, from_visual, class_label, conf = detections[j]
                if from_visual and det_car_id != 0:
                    for slot in self.slots:
                        if slot.car_id == det_car_id and slot.state == TrackState.INACTIVE:
                            slot.state = TrackState.TENTATIVE
                            slot.update(det_pos, class_label, conf, from_visual)
                            break

        # 清理 DELETED 槽位
        for slot in self.slots:
            if slot.state == TrackState.DELETED:
                slot.reset()

        # 返回结果（超过最大猜点帧数的 LOST 槽位不再输出）
        result = []
        for slot in self.slots:
            if slot.state in (TrackState.CONFIRMED, TrackState.LOST) and slot.has_ever_matched:
                pos = slot.get_output_position()
                if pos is not None:
                    result.append((slot.car_id, pos))
        return result
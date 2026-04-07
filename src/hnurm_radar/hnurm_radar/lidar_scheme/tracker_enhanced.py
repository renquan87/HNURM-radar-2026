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
        self.car_id = car_id          # 固定的敌方 ID（1~7 或 101~107）
        self.state = TrackState.INACTIVE
        self.raw_position = None      # 原始观测位置（赛场坐标），仅用于发布
        self.filtered_position = None # 卡尔曼滤波后的位置，用于预测和猜点
        
        # 卡尔曼滤波器（仅用于内部预测，不用于发布）
        self.kf = EnhancedKalmanFilter(
            process_noise=process_noise,
            measurement_noise=measurement_noise,
            jump_threshold=jump_threshold,
            max_velocity=max_velocity
        )
        
        # 软外观模型
        self.class_dist = np.zeros(class_num, dtype=np.float32)
        self.alpha = 0.7               # 指数平滑系数
        self.hits = 0                  # 连续匹配成功次数（用于 TENTATIVE -> CONFIRMED）
        self.lost_count = 0            # 连续丢失次数（用于 CONFIRMED -> LOST -> DELETED）
        self.last_update_time = time.time()
        self.has_ever_matched = False  # 是否曾与视觉观测匹配过（有 label）

    def update_appearance(self, class_label: int, confidence: float):
        """更新外观分布（指数滑动平均）"""
        one_hot = np.zeros_like(self.class_dist)
        one_hot[class_label] = confidence
        self.class_dist = self.alpha * self.class_dist + (1 - self.alpha) * one_hot

    def appearance_similarity(self, class_label: int) -> float:
        """计算观测类别与历史分布的余弦相似度（0~1）"""
        if np.sum(self.class_dist) == 0:
            return 0.5   # 无历史时中性
        obs_vec = np.zeros_like(self.class_dist)
        obs_vec[class_label] = 1.0
        sim = np.dot(obs_vec, self.class_dist) / (
            np.linalg.norm(obs_vec) * np.linalg.norm(self.class_dist) + 1e-6)
        return sim

    def update(self, raw_pos: np.ndarray, class_label: int = None,
               confidence: float = 0.0, from_visual: bool = False):
        """
        更新槽位：存储原始观测，更新卡尔曼滤波器，更新外观模型
        raw_pos: 赛场坐标 [x, y, z]（原始值）
        """
        # 存储原始观测（用于发布）
        self.raw_position = raw_pos.copy()
        # 更新卡尔曼滤波器（融合观测，得到滤波后位置）
        x, y = raw_pos[0], raw_pos[1]
        filtered_x, filtered_y = self.kf.update(x, y)   # 注意：update 返回滤波后位置
        self.filtered_position = np.array([filtered_x, filtered_y, raw_pos[2]])
        
        if from_visual and class_label is not None and class_label >= 0:
            self.update_appearance(class_label, confidence)
            self.has_ever_matched = True
        
        self.hits += 1
        self.lost_count = 0
        self.last_update_time = time.time()
        
        # 状态转换
        if self.state == TrackState.TENTATIVE and self.hits >= 3:
            self.state = TrackState.CONFIRMED
        elif self.state == TrackState.LOST:
            self.state = TrackState.CONFIRMED

    def predict(self, dt: float):
        """
        仅预测位置（不更新观测），用于匹配前获取预测位置
        返回预测位置（赛场坐标）
        """
        if self.filtered_position is None:
            return None
        # 注意：EnhancedKalmanFilter 的 predict_only 会内部更新状态转移矩阵的 dt
        pred = self.kf.predict_only()
        if pred is not None:
            pred_x, pred_y = pred
            return np.array([pred_x, pred_y, self.filtered_position[2]])
        else:
            # 未初始化，返回上一次的滤波位置
            return self.filtered_position

    def mark_missed(self):
        """标记一帧丢失"""
        self.lost_count += 1
        if self.state == TrackState.CONFIRMED:
            self.state = TrackState.LOST
        elif self.state in (TrackState.TENTATIVE, TrackState.LOST) and self.lost_count > 5:
            self.state = TrackState.DELETED
        self.last_update_time = time.time()

    def get_output_position(self) -> Optional[np.ndarray]:
        """
        获取对外发布的位置：
        - CONFIRMED 状态：返回原始观测（raw_position）
        - LOST 状态：返回卡尔曼预测位置（猜点）
        - 其他状态：返回 None
        """
        if self.state == TrackState.CONFIRMED:
            return self.raw_position
        elif self.state == TrackState.LOST:
            # 使用最近一次预测（需要调用 predict 后才有最新预测）
            # 注意：在 LOST 状态下，应连续调用 predict_only 外推，此处简单返回上次预测位置
            # 更精确的做法是在主循环中定期调用 predict 并保存预测位置
            if self.filtered_position is not None:
                # 如果没有最新预测，返回上一次的滤波位置（不推荐，但可避免 None）
                return self.filtered_position
        return None

    def reset(self):
        """重置槽位（当 DELETED 时调用）"""
        self.state = TrackState.INACTIVE
        self.hits = 0
        self.lost_count = 0
        self.class_dist.fill(0)
        self.raw_position = None
        self.filtered_position = None
        self.has_ever_matched = False
        # 重置卡尔曼滤波器（重新初始化）
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
        """
        enemy_car_ids: 敌方 car_id 列表（顺序与槽位索引对应）
        """
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

    def predict_all(self, dt: float):
        """对所有活跃槽位进行预测（更新预测位置，用于匹配）"""
        for slot in self.slots:
            if slot.state in (TrackState.CONFIRMED, TrackState.LOST):
                # 调用 predict 会更新卡尔曼预测状态，并返回预测位置
                # 注意：predict 内部会调用 kf.predict_only()，并更新 slot.filtered_position?
                # 为了简单，我们直接在匹配前调用 slot.predict(dt) 获取预测位置，但不需要存储到 slot 中，
                # 匹配时动态获取即可。然而为了效率，可以预先计算并缓存。
                # 我们可以在匹配前对每个槽位调用 predict 并将结果暂存。
                pass

    def update(self, detections: List[Tuple[int, np.ndarray, bool, Optional[int], float]],
               current_time: float) -> List[Tuple[int, np.ndarray]]:
        """
        参数:
            detections: 每个元素为 (car_id, position_xyz, from_visual, class_label, confidence)
                        from_visual 为 True 时 class_label 和 confidence 有效。
            current_time: 当前时间戳（秒），用于计算 dt
        返回:
            确认态且 has_ever_matched 的轨迹列表 [(car_id, position), ...]
        """
        # 计算时间差
        dt = current_time - self.last_update_time
        dt = max(0.01, min(0.2, dt))   # 限制范围
        self.last_update_time = current_time

        # 1. 对每个活跃槽位获取预测位置（用于匹配）
        active_slots = [s for s in self.slots if s.state in (TrackState.TENTATIVE, TrackState.CONFIRMED, TrackState.LOST)]
        if not active_slots and not detections:
            return []

        # 为每个槽位计算预测位置（调用 predict 方法，但注意 predict 会修改内部卡尔曼状态）
        # 注意：EnhancedKalmanFilter.predict_only 会执行一次预测并更新状态，但不改变观测。
        # 为避免影响后续的观测更新，我们应当在匹配前为每个槽位进行一次预测，
        # 匹配成功后再用观测更新（调用 kf.update）。
        # 这里我们先保存预测位置到临时列表。
        pred_positions = []
        for slot in active_slots:
            if slot.filtered_position is not None:
                pred = slot.predict(dt)   # predict 内部会调用 kf.predict_only 并更新状态
                if pred is not None:
                    pred_positions.append(pred)
                else:
                    # 未初始化，使用原始观测（如果有）
                    pred_positions.append(slot.raw_position if slot.raw_position is not None else None)
            else:
                pred_positions.append(None)

        # 2. 构建代价矩阵
        n_slots = len(active_slots)
        n_dets = len(detections)
        cost_matrix = np.full((n_slots, n_dets), 1e9, dtype=np.float32)

        for i, slot in enumerate(active_slots):
            pred_pos = pred_positions[i]
            if pred_pos is None:
                continue
            for j, (det_car_id, det_pos, from_visual, class_label, conf) in enumerate(detections):
                # 位置代价（欧氏距离）
                pos_cost = np.linalg.norm(pred_pos[:2] - det_pos[:2])
                # 外观代价（仅当视觉观测且类别有效）
                app_cost = 0.0
                if from_visual and class_label is not None and class_label >= 0:
                    sim = slot.appearance_similarity(class_label)
                    app_cost = 1 - sim
                total_cost = self.w_pos * pos_cost + self.w_app * app_cost
                if total_cost < self.max_distance * 2:   # 宽松阈值
                    cost_matrix[i, j] = total_cost

        # 3. 匈牙利匹配
        row_ind, col_ind = linear_sum_assignment(cost_matrix)
        matched_slot_idx = set()
        matched_det_idx = set()
        for r, c in zip(row_ind, col_ind):
            if cost_matrix[r, c] < self.max_distance:
                matched_slot_idx.add(r)
                matched_det_idx.add(c)
                slot = active_slots[r]
                det_car_id, det_pos, from_visual, class_label, conf = detections[c]
                # 注意：这里调用 slot.update 会更新卡尔曼滤波器（correct），
                # 同时存储原始观测（raw_position）和滤波后位置（filtered_position）
                slot.update(det_pos, class_label, conf, from_visual)

        # 4. 未匹配的活跃槽位标记丢失
        for i in range(n_slots):
            if i not in matched_slot_idx:
                active_slots[i].mark_missed()

        # 5. 未匹配的视觉观测尝试激活 INACTIVE 槽位
        for j in range(n_dets):
            if j not in matched_det_idx:
                det_car_id, det_pos, from_visual, class_label, conf = detections[j]
                if from_visual and det_car_id != 0:
                    # 寻找相同 car_id 的 INACTIVE 槽位
                    for slot in self.slots:
                        if slot.car_id == det_car_id and slot.state == TrackState.INACTIVE:
                            slot.state = TrackState.TENTATIVE
                            slot.update(det_pos, class_label, conf, from_visual)
                            break

        # 6. 清理 DELETED 槽位（回归 INACTIVE）
        for slot in self.slots:
            if slot.state == TrackState.DELETED:
                slot.reset()

        # 7. 返回需要发布的轨迹（CONFIRMED 或 LOST 且 has_ever_matched）
        result = []
        for slot in self.slots:
            if slot.state in (TrackState.CONFIRMED, TrackState.LOST) and slot.has_ever_matched:
                pos = slot.get_output_position()
                if pos is not None:
                    result.append((slot.car_id, pos))
        return result
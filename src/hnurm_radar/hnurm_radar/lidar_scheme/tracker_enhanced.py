#!/usr/bin/env python3
"""
tracker_enhanced.py — 统一追踪模块（含TDT风格视觉身份融合）
===============================================================
包含：
  - SimpleKalmanFilter   : 轻量级卡尔曼滤波器
  - LidarTrack           : 单条点云轨迹（含历史点云与身份投票）
  - LidarTracker         : 点云轨迹追踪器（匈牙利匹配+视觉身份融合）
  - VisualTracker        : 纯视觉轨迹追踪器
  - FixedSlotTracker     : 融合轨迹固定槽位追踪器（保留）
"""

import time
import numpy as np
from typing import List, Tuple, Dict, Optional
from scipy.optimize import linear_sum_assignment
from ..filters.kalman_filter import EnhancedKalmanFilter


# ===================== 轻量级卡尔曼滤波器 =====================
class SimpleKalmanFilter:
    def __init__(self, process_noise=0.01, measurement_noise=0.1, dt=0.1):
        self.dt = dt
        self.F = np.array([[1, 0, dt, 0],
                           [0, 1, 0, dt],
                           [0, 0, 1, 0],
                           [0, 0, 0, 1]], dtype=np.float32)
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=np.float32)
        self.Q = np.eye(4, dtype=np.float32) * process_noise
        self.R = np.eye(2, dtype=np.float32) * measurement_noise
        self.x = np.zeros((4, 1), dtype=np.float32)
        self.P = np.eye(4, dtype=np.float32)
        self.initialized = False

    def init(self, x, y):
        self.x[0, 0] = x
        self.x[1, 0] = y
        self.x[2, 0] = 0.0
        self.x[3, 0] = 0.0
        self.initialized = True

    def predict(self, dt):
        self.F[0, 2] = dt
        self.F[1, 3] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[0, 0], self.x[1, 0]

    def update(self, z):
        y = z.reshape(2, 1) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x[0, 0], self.x[1, 0]

    def get_state(self):
        return self.x[0, 0], self.x[1, 0], self.x[2, 0], self.x[3, 0]


# ===================== 点云轨迹（含C++风格身份融合） =====================
class LidarTrack:
    def __init__(self, track_id, kf, init_pos, init_time):
        self.id = track_id
        self.kf = kf
        self.predict_point = init_pos
        self.hits = 1
        self.lost_count = 0
        self.last_time = init_time
        self.timer = time.time()
        self.color = None
        self.number = -1
        self.has_ever_matched_visual = False

        # C++风格历史点云与身份投票
        self.history = []          # 每项: (timestamp, x, y)
        self.detect_history = []   # 每项: (color, number)  color:0=蓝,2=红
        self.max_history = 20

    def predict(self, dt):
        self.kf.predict(dt)
        self.predict_point = self.kf.get_state()[:2]

    def update(self, obs_pos, current_time):
        z = np.array([obs_pos[0], obs_pos[1]])
        self.kf.update(z)
        self.predict_point = self.kf.get_state()[:2]
        self.last_time = current_time
        self.lost_count = 0
        self.hits += 1
        # 保存历史点云
        self.history.append((current_time, obs_pos[0], obs_pos[1]))
        if len(self.history) > self.max_history:
            self.history.pop(0)

    def distance(self, point):
        return np.hypot(self.predict_point[0] - point[0], self.predict_point[1] - point[1])

    # ---------- C++ camera_match 移植 ----------
    def camera_match(self, visual_time, visual_point, color, number):
        TIME_THRESHOLD = 1.0   # 时间窗口 1 秒
        detect_r = 1.0         # 空间距离阈值

        # 寻找历史点云中时间最接近的点
        best_diff = float('inf')
        best_point = None
        for ts, x, y in self.history:
            diff = abs(ts - visual_time)
            if diff < best_diff:
                best_diff = diff
                best_point = (x, y)

        if best_diff > TIME_THRESHOLD or best_point is None:
            return

        # 空间距离校验
        dist = np.hypot(best_point[0] - visual_point[0], best_point[1] - visual_point[1])
        if dist < detect_r:
            self.detect_history.append((color, number))
            if len(self.detect_history) > self.max_history:
                self.detect_history.pop(0)
            self.has_ever_matched_visual = True

    def get_color(self):
        """多数投票决定颜色 (0=蓝, 2=红, 1=未定)"""
        if not self.detect_history:
            return 1
        blue = sum(1 for c, _ in self.detect_history if c == 0)
        red = sum(1 for c, _ in self.detect_history if c == 2)
        if red > blue:
            return 2
        elif blue > red:
            return 0
        else:
            return 1

    def get_number(self):
        """在确定颜色后，统计该颜色下各编号出现次数，返回多数编号"""
        color = self.get_color()
        if color not in (0, 2):
            return -1
        number_counts = {}
        for c, n in self.detect_history:
            if c == color:
                number_counts[n] = number_counts.get(n, 0) + 1
        if not number_counts:
            return -1
        return max(number_counts, key=number_counts.get)

    def get_identity(self):
        """返回最终身份: (color, number)"""
        return self.get_color(), self.get_number()


# ===================== 点云轨迹追踪器（含身份融合） =====================
class LidarTracker:
    def __init__(self, max_distance=1.0, max_lost=5, min_hits=3,
                 car_max_speed=2.5, detect_r=1.0):
        self.tracks: List[LidarTrack] = []
        self.next_id = 0
        self.max_distance = max_distance
        self.max_lost = max_lost
        self.min_hits = min_hits
        self.car_max_speed = car_max_speed
        self.detect_r = detect_r
        self.last_time = time.time()

    def update(self, detections, current_time, visual_dets=None):
        """
        detections: list of (x, y)  点云簇中心
        visual_dets: list of (x, y, color, number, timestamp)  视觉观测，用于身份融合
        """
        dt = max(0.01, min(0.2, current_time - self.last_time))
        self.last_time = current_time

        # 预测
        for tr in self.tracks:
            tr.predict(dt)

        # 匈牙利匹配（点云观测与轨迹）
        n_tracks = len(self.tracks)
        n_dets = len(detections)
        cost = np.full((n_tracks, n_dets), 1e9)
        for i, tr in enumerate(self.tracks):
            for j, det in enumerate(detections):
                dist = tr.distance(det)
                if dist < self.max_distance:
                    cost[i, j] = dist

        row, col = linear_sum_assignment(cost)
        matched_tracks = set()
        matched_dets = set()
        for r, c in zip(row, col):
            if cost[r, c] < self.max_distance:
                matched_tracks.add(r)
                matched_dets.add(c)
                self.tracks[r].update(detections[c], current_time)

        # 未匹配的轨迹丢失计数
        for i in range(n_tracks):
            if i not in matched_tracks:
                self.tracks[i].lost_count += 1

        # 未匹配的点云创建新轨迹
        for j in range(n_dets):
            if j not in matched_dets:
                kf = SimpleKalmanFilter()
                kf.init(detections[j][0], detections[j][1])
                new_tr = LidarTrack(self.next_id, kf, detections[j], current_time)
                self.tracks.append(new_tr)
                self.next_id += 1

        # ---------- 视觉身份融合（C++ camera_match） ----------
        if visual_dets:
            for vx, vy, color, number, vis_time in visual_dets:
                visual_point = (vx, vy)
                for tr in self.tracks:
                    tr.camera_match(vis_time, visual_point, color, number)

        # 删除丢失过久的轨迹
        self.tracks = [tr for tr in self.tracks if tr.lost_count <= self.max_lost]

    def get_confirmed_tracks(self):
        """返回已确认的轨迹 (id, x, y, label, source, color, number)"""
        result = []
        for tr in self.tracks:
            if tr.hits >= self.min_hits:
                x, y = tr.predict_point
                color, number = tr.get_identity()
                if color == 0:
                    label = f"B{number+1}" if number >= 0 else "NULL"
                elif color == 2:
                    label = f"R{number+1}" if number >= 0 else "NULL"
                else:
                    label = "NULL"
                result.append((tr.id, x, y, label, 'LIDAR', color, number))
        return result


# ===================== 视觉轨迹追踪器（保持不变） =====================
class VisualTracker:
    def __init__(self, max_distance=1.0, max_lost=5, min_hits=3):
        self.tracks = []
        self.next_id = 0
        self.max_distance = max_distance
        self.max_lost = max_lost
        self.min_hits = min_hits

    def update(self, detections, dt):
        for tr in self.tracks:
            tr['kf'].predict(dt)

        n_tracks = len(self.tracks)
        n_dets = len(detections)
        cost = np.full((n_tracks, n_dets), 1e9)
        for i, tr in enumerate(self.tracks):
            pred_x, pred_y = tr['kf'].get_state()[:2]
            for j, det in enumerate(detections):
                dist = np.hypot(pred_x - det[0], pred_y - det[1])
                if dist < self.max_distance:
                    cost[i, j] = dist

        row, col = linear_sum_assignment(cost)
        matched_tracks = set()
        matched_dets = set()
        for r, c in zip(row, col):
            if cost[r, c] < self.max_distance:
                matched_tracks.add(r)
                matched_dets.add(c)
                tr = self.tracks[r]
                z = np.array([detections[c][0], detections[c][1]])
                tr['kf'].update(z)
                tr['lost'] = 0
                tr['hits'] += 1
                tr['label'] = detections[c][2]

        for i in range(n_tracks):
            if i not in matched_tracks:
                self.tracks[i]['lost'] += 1

        for j in range(n_dets):
            if j not in matched_dets:
                kf = SimpleKalmanFilter()
                kf.init(detections[j][0], detections[j][1])
                self.tracks.append({
                    'id': self.next_id,
                    'kf': kf,
                    'label': detections[j][2],
                    'hits': 1,
                    'lost': 0,
                })
                self.next_id += 1

        self.tracks = [tr for tr in self.tracks if tr['lost'] <= self.max_lost]

    def get_confirmed_tracks(self):
        result = []
        for tr in self.tracks:
            if tr['hits'] >= self.min_hits:
                x, y, _, _ = tr['kf'].get_state()
                result.append((tr['id'], x, y, tr['label'], 'VISUAL'))
        return result


# ===================== 原有固定槽位追踪器（保持不变） =====================
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
        self.guess_count = 0
        self.max_guess_frames = 10

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
        self.guess_count = 0
        
        if self.state == TrackState.TENTATIVE and self.hits >= 3:
            self.state = TrackState.CONFIRMED
        elif self.state == TrackState.LOST:
            self.state = TrackState.CONFIRMED

    def predict(self, dt: float):
        if self.filtered_position is None:
            return None
        pred = self.kf.predict_only()
        if pred is not None:
            pred_x, pred_y = pred
            self.filtered_position = np.array([pred_x, pred_y, self.filtered_position[2]])
            if self.state == TrackState.LOST:
                self.guess_count += 1
            return self.filtered_position
        else:
            return self.filtered_position

    def mark_missed(self):
        self.lost_count += 1
        if self.state == TrackState.CONFIRMED:
            self.state = TrackState.LOST
            self.guess_count = 0
        elif self.state in (TrackState.TENTATIVE, TrackState.LOST) and self.lost_count > 5:
            self.state = TrackState.DELETED
        self.last_update_time = time.time()

    def get_output_position(self) -> Optional[np.ndarray]:
        if self.state == TrackState.CONFIRMED:
            return self.raw_position
        elif self.state == TrackState.LOST:
            if self.guess_count >= self.max_guess_frames:
                return None
            return self.filtered_position
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

        for slot in active_slots:
            slot.predict(dt)

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

        for i in range(n_slots):
            if i not in matched_slot_idx:
                active_slots[i].mark_missed()

        for j in range(n_dets):
            if j not in matched_det_idx:
                det_car_id, det_pos, from_visual, class_label, conf = detections[j]
                if from_visual and det_car_id != 0:
                    for slot in self.slots:
                        if slot.car_id == det_car_id and slot.state == TrackState.INACTIVE:
                            slot.state = TrackState.TENTATIVE
                            slot.update(det_pos, class_label, conf, from_visual)
                            break

        for slot in self.slots:
            if slot.state == TrackState.DELETED:
                slot.reset()

        result = []
        for slot in self.slots:
            if slot.state in (TrackState.CONFIRMED, TrackState.LOST) and slot.has_ever_matched:
                pos = slot.get_output_position()
                if pos is not None:
                    result.append((slot.car_id, pos))
        return result
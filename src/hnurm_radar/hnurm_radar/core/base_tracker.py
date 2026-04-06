"""
base_tracker.py — 跟踪器基类

定义目标跟踪的公共接口。
当前项目有多种跟踪机制：
  - ByteTrack（ultralytics 内置）— 地面机器人多目标跟踪
  - 卡尔曼滤波（air_kalman_filter.py）— 空中目标跟踪
  - EKF（ekf_node.py）— 坐标平滑滤波
  - 投票累积（CarList）— 目标 ID 投票确认

本基类统一跟踪器的生命周期和数据流接口，
便于阶段二优化时替换或组合不同的跟踪算法。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class TrackedTarget:
    """单个被跟踪目标的状态。"""

    track_id: int  # 全局唯一跟踪 ID
    label: str = ""  # 当前最优标签（经投票/累积）
    class_id: int = -1  # 类别数字 ID
    confidence: float = 0.0  # 跟踪置信度
    position: tuple | None = None  # 当前位置 (x, y) 或 (x, y, z)
    velocity: tuple | None = None  # 当前速度估计 (vx, vy) 或 (vx, vy, vz)
    bbox_xyxy: tuple = (0, 0, 0, 0)  # 最新检测框
    age: int = 0  # 已存活帧数
    hits: int = 0  # 成功匹配次数
    time_since_update: int = 0  # 距上次观测的帧数
    is_confirmed: bool = False  # 是否已确认（满足最小连续观测帧数）
    extra: dict[str, Any] = field(default_factory=dict)


class BaseTracker(ABC):
    """跟踪器抽象基类。

    跟踪器的典型工作流：

    1. ``predict()``  — 状态预测（卡尔曼预测步）
    2. ``update(detections)`` — 关联检测结果并更新状态
    3. ``get_tracks()`` — 获取当前所有活跃跟踪目标

    子类需要实现：
      - update(detections) : 核心关联+更新逻辑
      - get_tracks()       : 返回活跃目标列表

    可选覆写：
      - predict()  : 状态预测
      - reset()    : 重置跟踪器状态

    典型使用模式::

        tracker = MyTracker(config)
        for frame_detections in detection_stream:
            tracker.predict()
            tracker.update(frame_detections)
            active = tracker.get_tracks()
            publish(active)
    """

    def __init__(self, config: dict):
        """
        Args:
            config: 跟踪器配置（通常含 max_lost_frames, match_distance 等）
        """
        self._config = config
        self._frame_count = 0

    @property
    def config(self) -> dict:
        """只读访问配置。"""
        return self._config

    @property
    def frame_count(self) -> int:
        """当前已处理帧数。"""
        return self._frame_count

    def predict(self):
        """状态预测步（可选覆写）。

        在 update() 之前调用，执行卡尔曼预测或运动模型外推。
        默认实现为空操作。
        """

    @abstractmethod
    def update(self, detections: list) -> list[TrackedTarget]:
        """使用新一帧的检测结果更新跟踪器。

        核心方法，需要实现：
          1. 检测-跟踪关联（匈牙利/贪心/级联匹配）
          2. 已匹配目标的状态更新
          3. 未匹配检测的新目标初始化
          4. 长时间未匹配的目标删除

        Args:
            detections: 当前帧的检测结果列表
                       （Detection 对象或等效字典/元组）

        Returns:
            当前帧更新后的活跃跟踪目标列表
        """

    @abstractmethod
    def get_tracks(self) -> list[TrackedTarget]:
        """获取所有活跃的跟踪目标。

        Returns:
            当前活跃的 TrackedTarget 列表
            （通常只返回 is_confirmed=True 的目标）
        """

    def reset(self):
        """重置跟踪器状态。

        清除所有跟踪目标，重置帧计数器。
        子类覆写时应调用 super().reset()。
        """
        self._frame_count = 0

    def step(self, detections: list) -> list[TrackedTarget]:
        """执行一个完整的 predict-update 步骤。

        便捷方法，依次调用 predict() 和 update()。
        """
        self._frame_count += 1
        self.predict()
        return self.update(detections)

"""
base_detector.py — 检测器基类

定义所有检测方案节点的公共接口。
当前项目有三种检测方案：
  - camera_scheme/camera_detector.py  — 纯相机透视变换方案
  - lidar_scheme/detector_node.py     — 激光雷达+相机方案
  - air_scheme/air_target_node.py     — 空中目标检测方案（纯点云）

公共模式：
  1. 加载配置文件 → 初始化推理引擎
  2. 传感器帧获取（相机/点云）→ 推理循环
  3. 结果发布到 ROS2 话题

本基类不引入 ROS2 依赖，仅定义数据处理接口。
ROS2 节点生命周期由具体实现类管理（继承 rclpy.node.Node）。
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

# ======================== 数据容器 ========================


@dataclass
class Detection:
    """单个检测结果的标准化数据容器。

    所有检测方案输出统一为此格式，供下游跟踪/定位/发布使用。
    """

    track_id: int = -1  # 跟踪 ID（-1 = 未跟踪）
    label: str = ""  # 分类标签，如 "B1", "R3", "G5"
    class_id: int = -1  # 类别数字 ID
    confidence: float = 0.0  # 检测/分类置信度 [0, 1]
    bbox_xyxy: tuple = (0, 0, 0, 0)  # 像素坐标边界框 (x1, y1, x2, y2)
    field_xy: tuple | None = None  # 赛场坐标 (x, y)，若已定位
    field_xyz: tuple | None = None  # 赛场 3D 坐标 (x, y, z)，若有
    extra: dict[str, Any] = field(default_factory=dict)  # 扩展字段


@dataclass
class FrameResult:
    """单帧检测结果的标准化容器。"""

    timestamp: float = 0.0  # 时间戳（秒）
    detections: list[Detection] = field(default_factory=list)
    frame: Any = None  # 原始帧数据（numpy array 或 PointCloud）
    annotated_frame: Any = None  # 标注后的帧（用于可视化）
    inference_ms: float = 0.0  # 推理耗时（毫秒）


# ======================== 检测器基类 ========================


class BaseDetector(ABC):
    """检测器抽象基类。

    子类需要实现：
      - setup()        : 初始化模型、传感器等资源
      - detect(frame)  : 对单帧执行检测，返回 FrameResult
      - cleanup()      : 释放资源

    可选覆写：
      - preprocess(frame)   : 帧预处理（缩放、裁剪等）
      - postprocess(result) : 后处理（NMS、过滤等）

    典型使用模式::

        class MyDetector(BaseDetector):
            def setup(self):
                self.pipeline = YoloPipeline(cfg, ...)

            def detect(self, frame):
                raw = self.pipeline.infer(frame)
                return self._to_frame_result(raw)

            def cleanup(self):
                del self.pipeline
    """

    def __init__(self, config: dict):
        """
        Args:
            config: 检测器配置字典（通常从 detector_config.yaml 加载）
        """
        self._config = config
        self._is_setup = False

    @property
    def config(self) -> dict:
        """只读访问配置。"""
        return self._config

    def setup(self):
        """初始化检测器资源（模型加载、传感器连接等）。

        子类覆写此方法时应在末尾调用 super().setup()。
        """
        self._is_setup = True

    @abstractmethod
    def detect(self, frame) -> FrameResult:
        """对单帧数据执行检测。

        Args:
            frame: 输入帧数据（numpy.ndarray 图像或点云数据）

        Returns:
            FrameResult 包含本帧所有检测结果
        """

    def preprocess(self, frame):
        """帧预处理（可选覆写）。

        默认直接返回原始帧。子类可覆写以添加缩放、裁剪、增强等。
        """
        return frame

    def postprocess(self, result: FrameResult) -> FrameResult:
        """结果后处理（可选覆写）。

        默认直接返回原始结果。子类可覆写以添加过滤、NMS、坐标转换等。
        """
        return result

    def cleanup(self):
        """释放检测器资源。

        子类覆写此方法时应在末尾调用 super().cleanup()。
        """
        self._is_setup = False

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.cleanup()
        return False

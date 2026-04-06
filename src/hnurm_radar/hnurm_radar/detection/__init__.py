"""
detection — 三阶段 YOLO 推理管线模块

从 detector_node.py 和 camera_detector.py 中提取的公共检测逻辑。
包含：
  - label_mappings  : 灰色装甲板标签映射常量
  - yolo_pipeline   : 三阶段 YOLO 推理引擎类（不含 ROS 依赖）

典型用法（在 ROS 节点中）::

    from ..detection.yolo_pipeline import YoloPipeline
    self.pipeline = YoloPipeline(det_cfg, logger=self.get_logger())
"""

from .label_mappings import (
    Gray2Blue, Gray2Red, gray2gray, Blue2Gray, Red2Gray,
)

__all__ = [
    'Gray2Blue', 'Gray2Red', 'gray2gray', 'Blue2Gray', 'Red2Gray',
]

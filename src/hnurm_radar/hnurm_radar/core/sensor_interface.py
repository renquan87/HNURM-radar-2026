"""
sensor_interface.py — 传感器抽象接口

定义相机和激光雷达等传感器的统一接口，为阶段四的
多传感器融合提供抽象基础。

当前项目中的传感器：
  - Camera/HKCam.py       — 海康工业相机（SDK 直连，拉模式）
  - lidar_scheme/lidar_node.py — Livox HAP 激光雷达（ROS2 话题回调，推模式）
  - （未来）USB 相机、深度相机、毫米波雷达等

设计考量：
  1. 兼容拉模式（poll / grab）和推模式（callback）两种采集方式
  2. 不引入 ROS2 依赖，保持纯 Python 接口
  3. 提供上下文管理器（with 语法）保证资源释放
  4. 传感器状态可查询（已连接/已打开/错误）

典型使用模式::

    # 拉模式（相机）
    class HKCamera(BaseSensor):
        def open(self):
            self._cam = MvCamera()
            ...
        def read(self):
            frame = self._cam.MV_CC_GetImageBuffer(...)
            return SensorFrame(data=frame, ...)

    # 推模式（激光雷达，在 ROS2 回调中填充）
    class LidarSensor(BaseSensor):
        def open(self):
            # 订阅在 ROS2 Node 中处理
            self._latest = None
        def on_data(self, points):
            self._latest = SensorFrame(data=points, ...)
        def read(self):
            return self._latest
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Dict, Optional

# ======================== 传感器状态 ========================


class SensorState(Enum):
    """传感器运行状态。"""

    DISCONNECTED = auto()  # 未连接
    CONNECTED = auto()  # 已连接，未开始采集
    STREAMING = auto()  # 正在采集数据
    ERROR = auto()  # 出错


# ======================== 传感器类型 ========================


class SensorType(Enum):
    """传感器类型标识。"""

    CAMERA = "camera"  # 2D 相机（工业 / USB / 网络）
    LIDAR = "lidar"  # 3D 激光雷达
    DEPTH_CAMERA = "depth"  # 深度相机（RGBD）
    OTHER = "other"


# ======================== 数据帧容器 ========================


@dataclass
class SensorFrame:
    """传感器采集的单帧数据。

    通用容器，可承载图像（numpy HxWxC）、点云（numpy Nx3）等。
    """

    data: Any = None  # 帧数据（numpy array）
    timestamp: float = 0.0  # 采集时间戳（秒）
    sequence: int = 0  # 帧序号
    sensor_type: SensorType = SensorType.OTHER
    width: int = 0  # 图像宽度（仅相机）
    height: int = 0  # 图像高度（仅相机）
    num_points: int = 0  # 点数量（仅激光雷达）
    extra: dict[str, Any] = field(default_factory=dict)

    @property
    def is_valid(self) -> bool:
        """数据是否有效（非 None）。"""
        return self.data is not None


# ======================== 传感器基类 ========================


class BaseSensor(ABC):
    """传感器抽象基类。

    子类需要实现：
      - open()           : 连接/初始化传感器硬件
      - read()           : 获取最新一帧数据
      - close()          : 释放传感器资源

    可选覆写：
      - configure(params): 运行时参数调节
      - get_info()       : 获取传感器信息（型号、分辨率等）

    生命周期::

        sensor = MyCamera(config)
        sensor.open()           # DISCONNECTED → CONNECTED → STREAMING
        frame = sensor.read()   # 获取数据
        sensor.close()          # STREAMING → DISCONNECTED

    或使用 with 语法::

        with MyCamera(config) as cam:
            frame = cam.read()
    """

    def __init__(self, config: dict, sensor_type: SensorType = SensorType.OTHER):
        """
        Args:
            config: 传感器配置字典（通常从 main_config.yaml 加载）
            sensor_type: 传感器类型标识
        """
        self._config = config
        self._sensor_type = sensor_type
        self._state = SensorState.DISCONNECTED
        self._frame_count = 0
        self._error_msg: str | None = None

    # ---------- 属性 ----------

    @property
    def config(self) -> dict:
        """只读访问配置。"""
        return self._config

    @property
    def sensor_type(self) -> SensorType:
        """传感器类型。"""
        return self._sensor_type

    @property
    def state(self) -> SensorState:
        """当前状态。"""
        return self._state

    @property
    def is_opened(self) -> bool:
        """传感器是否已打开并可用。"""
        return self._state in (SensorState.CONNECTED, SensorState.STREAMING)

    @property
    def frame_count(self) -> int:
        """已读取的帧数。"""
        return self._frame_count

    @property
    def error_message(self) -> str | None:
        """最近一次错误信息。"""
        return self._error_msg

    # ---------- 核心接口 ----------

    @abstractmethod
    def open(self):
        """连接并初始化传感器。

        实现应在成功后将 _state 设置为 CONNECTED 或 STREAMING。
        失败时应设置 _state = ERROR 并填充 _error_msg。
        """

    @abstractmethod
    def read(self) -> SensorFrame | None:
        """读取最新一帧数据。

        Returns:
            SensorFrame: 成功时返回帧数据
            None: 暂无数据或读取失败
        """

    @abstractmethod
    def close(self):
        """释放传感器资源。

        实现应在末尾将 _state 设置为 DISCONNECTED。
        """

    # ---------- 可选接口 ----------

    def configure(self, params: dict):
        """运行时参数调节（可选覆写）。

        例如调节相机曝光、增益，或激光雷达扫描模式。

        Args:
            params: 参数键值对
        """

    def get_info(self) -> dict[str, Any]:
        """获取传感器信息（可选覆写）。

        Returns:
            包含型号、分辨率、固件版本等信息的字典
        """
        return {
            "sensor_type": self._sensor_type.value,
            "state": self._state.name,
            "frame_count": self._frame_count,
        }

    # ---------- 辅助方法 ----------

    def _set_state(self, state: SensorState, error_msg: str | None = None):
        """安全地更新状态（供子类调用）。"""
        self._state = state
        if state == SensorState.ERROR:
            self._error_msg = error_msg
        elif error_msg is None:
            self._error_msg = None

    def _increment_frame_count(self):
        """帧计数器自增（供子类在 read() 成功后调用）。"""
        self._frame_count += 1

    # ---------- 上下文管理 ----------

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()
        return False

    def __repr__(self) -> str:
        return (
            f"<{self.__class__.__name__} "
            f"type={self._sensor_type.value} "
            f"state={self._state.name} "
            f"frames={self._frame_count}>"
        )

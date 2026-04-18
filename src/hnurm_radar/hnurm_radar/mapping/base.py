"""
base.py — 坐标映射器抽象基类
========================================
定义像素→赛场坐标映射的统一接口，供 Homography 和 Raycast 两种实现遵循。
"""
from abc import ABC, abstractmethod
from typing import Optional, Tuple


class CoordinateMapper(ABC):
    """坐标映射器抽象基类。
    输入：图像像素坐标 (px, py)
    输出：赛场坐标 (field_x, field_y, field_z)
    """

    @abstractmethod
    def pixel_to_field(self, px: float, py: float) -> Optional[Tuple[float, float, float]]:
        """将像素坐标映射到赛场坐标。

        参数:
            px: 原始分辨率下的像素 x 坐标
            py: 原始分辨率下的像素 y 坐标
        返回:
            (field_x, field_y, field_z): 赛场坐标，单位 m。
            field_x ∈ [0, 28], field_y ∈ [0, 15], field_z 为高度。
            映射失败返回 None。
        """
        ...

    @abstractmethod
    def is_ready(self) -> bool:
        """映射器是否已完成初始化并可用。"""
        ...

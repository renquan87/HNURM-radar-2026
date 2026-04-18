"""
solidworks_to_field.py — Solidworks 坐标→裁判系统赛场坐标变换
========================================
将 PLY 网格模型的 Solidworks 坐标系交点转换为 RM 裁判系统赛场坐标。

Solidworks 坐标系 (PLY 原生):
    X 轴 → 赛场短轴方向 (宽 15m)
    Y 轴 → 垂直方向 (高度)
    Z 轴 → 赛场长轴方向 (长 28m)

裁判系统赛场坐标:
    field_x ∈ [0, 28]  — 赛场长轴
    field_y ∈ [0, 15]  — 赛场短轴
    field_z            — 物理高度 (用于地面/高地判断)

变换公式 (参考 HKUST solidwork2uwb.py):
    红方: field_x = -sw_z + 14.0,  field_y = -sw_x + 7.5
    蓝方: field_x =  sw_z + 14.0,  field_y =  sw_x + 7.5
    高度: field_z =  sw_y
"""
import numpy as np
from typing import Tuple


def solidworks_to_field(hit_point: np.ndarray, my_color: str) -> Tuple[float, float, float]:
    """将 Solidworks 坐标系的 3D 交点转换为裁判系统赛场坐标。

    参数:
        hit_point: [sw_x, sw_y, sw_z] Solidworks 坐标 (PLY 网格空间)
        my_color: "Red" 或 "Blue"
    返回:
        (field_x, field_y, field_z)
    """
    sw_x, sw_y, sw_z = float(hit_point[0]), float(hit_point[1]), float(hit_point[2])

    if my_color == 'Red':
        field_x = -sw_z + 14.0
        field_y = -sw_x + 7.5
    else:
        field_x = sw_z + 14.0
        field_y = sw_x + 7.5

    field_z = sw_y  # Solidworks Y 轴 = 物理高度
    return field_x, field_y, field_z

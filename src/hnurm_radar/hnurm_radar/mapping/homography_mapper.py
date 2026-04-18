"""
homography_mapper.py — 基于 Homography 透视变换的坐标映射器
========================================
将 CameraDetector 中现有的多层 Homography 逻辑封装为 CoordinateMapper 接口。
HomographyMapper 持有 H 矩阵、掩码图和地图尺寸等状态，独立于 CameraDetector 实例。

输入：原始分辨率下的像素坐标 (px, py)
输出：赛场坐标 (field_x, field_y, 0.0)
      z=0.0 (Homography 方案无高度信息)
"""
import cv2
import numpy as np
from typing import Optional, Tuple

from .base import CoordinateMapper


class HomographyMapper(CoordinateMapper):
    """基于多层 Homography 透视变换的坐标映射器。

    该类将 camera_detector.py 中的透视变换逻辑抽象封装，
    供 CameraDetector 通过统一的 CoordinateMapper 接口调用。
    """

    def __init__(
        self,
        H_ground: Optional[np.ndarray],
        H_highland: Optional[np.ndarray],
        mask_img: Optional[np.ndarray],
        my_color: str,
        calib_map_w: Optional[int] = None,
        calib_map_h: Optional[int] = None,
        calib_map_portrait: bool = False,
        field_width: float = 28.0,
        field_height: float = 15.0,
        logger=None,
    ):
        """
        参数:
            H_ground: 地面层 Homography 矩阵 (3x3)，相机像素→地图像素
            H_highland: 高地层 Homography 矩阵 (3x3)，可选
            mask_img: 分区掩码图，黑色=地面，非黑色=高地
            my_color: "Red" 或 "Blue"
            calib_map_w: 标定地图宽度 (px)
            calib_map_h: 标定地图高度 (px)
            calib_map_portrait: 标定地图是否竖版
            field_width: 赛场宽度 (m)
            field_height: 赛场高度 (m)
            logger: 可选的 ROS2 logger
        """
        self.H_ground = H_ground
        self.H_highland = H_highland
        self.mask_img = mask_img
        self.my_color = my_color
        self.calib_map_w = calib_map_w
        self.calib_map_h = calib_map_h
        self.calib_map_portrait = calib_map_portrait
        self.field_width = field_width
        self.field_height = field_height
        self._logger = logger

        # 掩码方向
        self.mask_is_portrait = False
        if mask_img is not None:
            h, w = mask_img.shape[:2]
            self.mask_is_portrait = (h > w)

    def _map_pixel_to_field(self, map_px: float, map_py: float) -> Tuple[float, float]:
        """将地图像素坐标转换为赛场米坐标。"""
        if self.calib_map_w is None:
            return map_px, map_py

        if self.calib_map_portrait:
            if self.my_color == 'Red':
                field_x = (self.calib_map_h - map_py) / self.calib_map_h * self.field_width
                field_y = (self.calib_map_w - map_px) / self.calib_map_w * self.field_height
            else:
                field_x = map_py / self.calib_map_h * self.field_width
                field_y = map_px / self.calib_map_w * self.field_height
        else:
            field_x = map_px / self.calib_map_w * self.field_width
            field_y = (self.calib_map_h - map_py) / self.calib_map_h * self.field_height
        return field_x, field_y

    def pixel_to_field(self, px: float, py: float) -> Optional[Tuple[float, float, float]]:
        """将像素坐标通过多层 Homography 变换到赛场坐标。

        参数:
            px, py: 原始分辨率下的像素坐标
        返回:
            (field_x, field_y, 0.0) 或 None
        """
        if self.H_ground is None:
            return None

        pt = np.array([[[px, py]]], dtype=np.float64)

        # Step 1: 地面层变换 → 地图像素坐标
        transformed = cv2.perspectiveTransform(pt, self.H_ground)
        map_x = transformed[0][0][0]
        map_y = transformed[0][0][1]

        # 异常值检测
        margin = 500
        if self.calib_map_w is not None:
            if (map_x < -margin or map_x > self.calib_map_w + margin or
                    map_y < -margin or map_y > self.calib_map_h + margin):
                return None

        # Step 2: 查掩码判定高度层
        if self.mask_img is not None and self.H_highland is not None:
            mask_h, mask_w = self.mask_img.shape[:2]
            if self.calib_map_w is not None:
                mx = int(map_x * mask_w / self.calib_map_w)
                my = int(map_y * mask_h / self.calib_map_h)
            else:
                if self.mask_is_portrait:
                    mx = int(map_y * mask_w / self.field_height)
                    my = int(map_x * mask_h / self.field_width)
                else:
                    mx = int(map_x * mask_w / self.field_width)
                    my = int(mask_h - map_y * mask_h / self.field_height)
            mx = max(0, min(mx, mask_w - 1))
            my = max(0, min(my, mask_h - 1))

            pixel_color = self.mask_img[my, mx]
            is_highland = not (pixel_color[0] == 0 and
                               pixel_color[1] == 0 and
                               pixel_color[2] == 0)

            if is_highland:
                transformed_h = cv2.perspectiveTransform(pt, self.H_highland)
                map_x = transformed_h[0][0][0]
                map_y = transformed_h[0][0][1]
                if self.calib_map_w is not None:
                    if (map_x < -margin or map_x > self.calib_map_w + margin or
                            map_y < -margin or map_y > self.calib_map_h + margin):
                        return None

        # Step 3: 地图像素 → 赛场米坐标
        fx, fy = self._map_pixel_to_field(map_x, map_y)
        return (fx, fy, 0.0)

    def is_ready(self) -> bool:
        return self.H_ground is not None

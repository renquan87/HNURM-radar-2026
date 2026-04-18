"""
raycast_mapper.py — 基于 Open3D 射线-网格求交的坐标映射器
========================================
通过将像素反投影为三维射线，与赛场 PLY 网格模型求交，
获取物理交点后经 Solidworks→赛场坐标变换得到最终赛场坐标。

核心流程:
    1. 像素去畸变
    2. 像素齐次坐标 → 相机方向向量: cam_dir = K_inv @ [u, v, 1]
    3. 相机方向 → 世界方向: world_dir = R^T @ cam_dir
    4. 相机原点 (世界系): origin = -R^T @ T
    5. Open3D RaycastingScene 射线求交
    6. 交点 (Solidworks 坐标) → 赛场坐标

外参约定:
    R, T 为 world-to-camera 外参 (PnP 标准输出)。
    构造射线时通过 R^T 和 -R^T @ T 反求 camera-to-world。

依赖:
    - open3d (pip install open3d)
    - configs/raycast_calib.yaml
    - 赛场 PLY 文件 (Solidworks 坐标系)
"""
import numpy as np
import cv2
import open3d as o3d
from typing import Optional, Tuple

from .base import CoordinateMapper
from .solidworks_to_field import solidworks_to_field


class RaycastMapper(CoordinateMapper):
    """基于 Open3D 射线-网格求交的坐标映射器。"""

    def __init__(self, config_path: str, my_color: str, logger=None):
        """
        参数:
            config_path: raycast_calib.yaml 文件路径
            my_color: "Red" 或 "Blue"
            logger: 可选的 ROS2 logger 实例，用于输出初始化信息
        """
        self.my_color = my_color
        self._logger = logger
        self._ready = False
        self._load_config(config_path)

    def _log_info(self, msg: str):
        if self._logger:
            self._logger.info(msg)
        else:
            print(f"[RaycastMapper] {msg}")

    def _log_warn(self, msg: str):
        if self._logger:
            self._logger.warn(msg)
        else:
            print(f"[RaycastMapper WARN] {msg}")

    def _load_config(self, config_path: str):
        """加载标定配置并初始化射线求交场景。"""
        import yaml
        import os

        try:
            with open(config_path, 'r', encoding='utf-8') as f:
                cfg = yaml.safe_load(f)
        except Exception as e:
            self._log_warn(f"无法加载配置文件 {config_path}: {e}")
            return

        # 相机内参
        self.K = np.array(cfg['K'], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

        # 外参 (world-to-camera)
        self.R = np.array(cfg['R'], dtype=np.float64)
        self.T = np.array(cfg['T'], dtype=np.float64).reshape(3, 1)

        # 畸变系数
        dist = cfg.get('dist_coeffs', [0, 0, 0, 0, 0])
        self.dist_coeffs = np.array(dist, dtype=np.float64)

        # 加载 PLY 网格
        mesh_path = cfg['mesh_path']
        # 支持相对路径 (相对于配置文件所在目录的上级，即项目根目录)
        if not os.path.isabs(mesh_path):
            config_dir = os.path.dirname(os.path.abspath(config_path))
            project_root = os.path.normpath(os.path.join(config_dir, '..'))
            mesh_path = os.path.join(project_root, mesh_path)

        mesh = o3d.io.read_triangle_mesh(mesh_path)
        if mesh.is_empty():
            self._log_warn(f"PLY 网格为空或加载失败: {mesh_path}")
            return

        # 构建 RaycastingScene
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(o3d.t.geometry.TriangleMesh.from_legacy(mesh))

        # 预计算相机在世界坐标系中的位置
        self._cam_origin = (-self.R.T @ self.T).flatten()

        self._ready = True

        # 输出网格范围信息
        vertices = np.asarray(mesh.vertices)
        v_min = np.min(vertices, axis=0)
        v_max = np.max(vertices, axis=0)
        self._log_info(
            f"射线求交映射器初始化完成: mesh={mesh_path}, "
            f"顶点数={len(vertices)}, "
            f"范围 min=({v_min[0]:.2f},{v_min[1]:.2f},{v_min[2]:.2f}) "
            f"max=({v_max[0]:.2f},{v_max[1]:.2f},{v_max[2]:.2f})")

    def pixel_to_field(self, px: float, py: float) -> Optional[Tuple[float, float, float]]:
        """将像素坐标映射到赛场坐标。

        参数:
            px: 原始分辨率下的像素 x 坐标
            py: 原始分辨率下的像素 y 坐标
        返回:
            (field_x, field_y, field_z) 或 None
        """
        if not self._ready:
            return None

        u, v = px, py

        # 畸变校正
        if self.dist_coeffs is not None and not np.all(self.dist_coeffs == 0):
            pts = np.array([[[u, v]]], dtype=np.float32)
            undist = cv2.undistortPoints(pts, self.K, self.dist_coeffs, P=self.K)
            u, v = float(undist[0, 0, 0]), float(undist[0, 0, 1])

        # 像素齐次坐标 → 相机方向向量
        pixel_hom = np.array([u, v, 1.0], dtype=np.float64)
        cam_dir = self.K_inv @ pixel_hom

        # 相机方向 → 世界方向
        world_dir = self.R.T @ cam_dir

        # 射线求交
        rays = o3d.core.Tensor(
            [[*self._cam_origin, *world_dir]], dtype=o3d.core.Dtype.Float32)
        result = self.scene.cast_rays(rays)
        t_hit = result['t_hit'].numpy()[0]

        if t_hit >= float('inf'):
            return None

        # 交点 (Solidworks 坐标)
        hit_point = self._cam_origin + t_hit * world_dir

        # Solidworks → 赛场坐标
        return solidworks_to_field(hit_point, self.my_color)

    def pixel_to_field_batch(self, pixels: np.ndarray) -> np.ndarray:
        """批量像素→赛场坐标映射。

        参数:
            pixels: Nx2 数组，每行 [px, py]
        返回:
            Nx4 数组，每行 [field_x, field_y, field_z, valid]
            valid=1.0 表示射线命中，valid=0.0 表示未命中
        """
        if not self._ready or len(pixels) == 0:
            return np.zeros((len(pixels), 4), dtype=np.float64)

        N = len(pixels)
        result_arr = np.zeros((N, 4), dtype=np.float64)

        # 畸变校正
        uv = pixels.astype(np.float32)
        if self.dist_coeffs is not None and not np.all(self.dist_coeffs == 0):
            pts = uv.reshape(-1, 1, 2)
            undist = cv2.undistortPoints(pts, self.K, self.dist_coeffs, P=self.K)
            uv = undist.reshape(-1, 2)

        # 批量构造射线
        pixel_hom = np.hstack([uv, np.ones((N, 1), dtype=np.float32)])  # Nx3
        cam_dirs = (self.K_inv @ pixel_hom.T).T  # Nx3
        world_dirs = (self.R.T @ cam_dirs.T).T   # Nx3
        origins = np.tile(self._cam_origin, (N, 1))  # Nx3
        rays = np.hstack([origins, world_dirs])  # Nx6

        rays_tensor = o3d.core.Tensor(rays.astype(np.float32), dtype=o3d.core.Dtype.Float32)
        cast_result = self.scene.cast_rays(rays_tensor)
        t_hits = cast_result['t_hit'].numpy()  # N

        for i in range(N):
            if t_hits[i] < float('inf'):
                hit_point = self._cam_origin + t_hits[i] * world_dirs[i]
                fx, fy, fz = solidworks_to_field(hit_point, self.my_color)
                result_arr[i] = [fx, fy, fz, 1.0]

        return result_arr

    def is_ready(self) -> bool:
        return self._ready

"""
transforms.py — 坐标变换工具函数

提供纯数学的坐标系变换功能，无 ROS2 依赖。
从 radar.py 和 lidar_node.py 中提取的公共逻辑。
"""

import numpy as np
from scipy.spatial.transform import Rotation


def quaternion_to_rotation_matrix(q):
    """四元数 (x, y, z, w) → 3×3 旋转矩阵。"""
    x, y, z, w = q
    return Rotation.from_quat([x, y, z, w]).as_matrix()


def tf_to_matrix(translation, rotation):
    """ROS2 TF 的 translation + rotation → 4×4 齐次变换矩阵。

    Args:
        translation: 具有 .x, .y, .z 属性的对象（geometry_msgs Translation）
        rotation: 具有 .x, .y, .z, .w 属性的对象（geometry_msgs Quaternion）
    """
    q = [rotation.x, rotation.y, rotation.z, rotation.w]
    R = quaternion_to_rotation_matrix(q)

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [translation.x, translation.y, translation.z]
    return T


def transform_points_homogeneous(points, matrix):
    """用 4×4 齐次变换矩阵批量变换 3D 点。

    Args:
        points: (N, 3) numpy array
        matrix: (4, 4) 变换矩阵

    Returns:
        (N, 3) 变换后的坐标
    """
    ones = np.ones((points.shape[0], 1))
    pts_h = np.hstack((points, ones))
    transformed = pts_h @ matrix.T
    return transformed[:, :3]

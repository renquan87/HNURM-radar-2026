#!/usr/bin/env python3
"""
从 T-DT 的 solvePnP 参数 + rosbag 点云 GICP 配准推导 lidar→camera 外参。

数学原理：
  T_lidar→camera = T_world→camera × T_lidar→world

  - T_world→camera: T-DT calibrate 节点用 solvePnP 标定的 rvec/tvec
                     (out_matrix.yaml)
  - T_lidar→world:  对 rosbag 中的原始点云与 PCD 场地地图做 GICP 配准
                     (模仿 T-DT localization 节点的行为)

用法：
  python3 scripts/derive_lidar_camera_extrinsic.py

依赖：
  pip install open3d opencv-python numpy
  需要 ROS2 humble (rclpy, sensor_msgs)

输出：
  打印推导出的 R (3×3) 和 T (3×1)，可直接填入 converter_config_rosbag.yaml
"""

import struct
import sqlite3
import sys

import cv2
import numpy as np
import open3d as o3d

# ──────────────────────────────────────────────────────────────
# 配置区 — 按实际路径修改
# ──────────────────────────────────────────────────────────────
ROSBAG_DB3 = (
    "/home/rm/rq/projects/radar/lidar-rosbag/competition/"
    "全明星赛第一局bag/全明星赛第一局/rosbag.db3"
)
PCD_MAP = (
    "/home/rm/rq/projects/radar/open-source-project-of-other-team/"
    "T-DT_Radar/config/RM2025.pcd"
)
# T-DT out_matrix.yaml 中的 solvePnP 结果
RVEC = np.array([1.4991050282713083, -1.2765999291002665, 0.96939795426133368])
TVEC = np.array([-10.019945443997486, 2.8500528283672280, 3.3869325006721294])
# T-DT camera_params.yaml
CAMERA_MATRIX = np.array([
    [3617.422853, 0.0, 2046.972383],
    [0.0, 3617.561167, 1515.242535],
    [0.0, 0.0, 1.0],
])
# 累积帧数（T-DT localization 用 20 帧）
ACCUMULATE_FRAMES = 20
# T-DT localization 的过滤条件
FILTER_X = (5, 30)
FILTER_Y = (-10, 8)
FILTER_Z_MAX = 7
# GICP 参数
VOXEL_SIZE = 0.1
GICP_THRESHOLD = 0.5


def main():
    # ── 第一步：构建 T_world→camera ──
    R_w2c, _ = cv2.Rodrigues(RVEC)
    T_w2c = np.eye(4)
    T_w2c[:3, :3] = R_w2c
    T_w2c[:3, 3] = TVEC
    print("=== T_world→camera (solvePnP) ===")
    print(T_w2c)

    # ── 第二步：从 rosbag 读取点云 ──
    sys.path.insert(0, "/opt/ros/humble/lib/python3.10/dist-packages")
    from rclpy.serialization import deserialize_message  # noqa: E402
    from sensor_msgs.msg import PointCloud2  # noqa: E402

    db = sqlite3.connect(ROSBAG_DB3)
    cur = db.cursor()
    # 找到 /livox/lidar 的 topic_id
    cur.execute(
        "SELECT id FROM topics WHERE name = '/livox/lidar' "
        "AND type = 'sensor_msgs/msg/PointCloud2'"
    )
    row = cur.fetchone()
    if row is None:
        raise RuntimeError("rosbag 中找不到 /livox/lidar topic")
    topic_id = row[0]

    cur.execute(
        f"SELECT data FROM messages WHERE topic_id = {topic_id} "
        f"ORDER BY timestamp LIMIT {ACCUMULATE_FRAMES}"
    )
    rows = cur.fetchall()
    db.close()
    print(f"\n读取了 {len(rows)} 帧点云")

    all_pts: list[list[float]] = []
    for (raw,) in rows:
        msg = deserialize_message(raw, PointCloud2)
        step = msg.point_step
        data = bytes(msg.data)
        n = len(data) // step
        for i in range(n):
            off = i * step
            x, y, z = struct.unpack_from("fff", data, off)
            if FILTER_X[0] < x < FILTER_X[1] and FILTER_Y[0] < y < FILTER_Y[1] and z < FILTER_Z_MAX:
                all_pts.append([x, y, z])
    print(f"累积过滤后点数: {len(all_pts)}")

    source = o3d.geometry.PointCloud()
    source.points = o3d.utility.Vector3dVector(np.array(all_pts))

    # ── 第三步：加载 PCD 地图 ──
    target = o3d.io.read_point_cloud(PCD_MAP)
    print(f"PCD 地图点数: {len(target.points)}")

    # 下采样
    target_d = target.voxel_down_sample(VOXEL_SIZE)
    source_d = source.voxel_down_sample(VOXEL_SIZE)
    print(f"下采样后: target={len(target_d.points)}, source={len(source_d.points)}")

    # 法向量
    sp = o3d.geometry.KDTreeSearchParamHybrid(radius=0.3, max_nn=30)
    target_d.estimate_normals(search_param=sp)
    source_d.estimate_normals(search_param=sp)

    # ── 第四步：GICP 配准 ──
    reg = o3d.pipelines.registration.registration_generalized_icp(
        source_d,
        target_d,
        GICP_THRESHOLD,
        np.eye(4),
        o3d.pipelines.registration.TransformationEstimationForGeneralizedICP(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
    )
    T_l2w = reg.transformation
    print(f"\n=== GICP 结果 ===")
    print(f"Fitness: {reg.fitness:.4f}")
    print(f"RMSE:    {reg.inlier_rmse:.4f}")
    print(f"T_lidar→world:\n{T_l2w}")

    # ── 第五步：推导 T_lidar→camera ──
    T_l2c = T_w2c @ T_l2w
    R_l2c = T_l2c[:3, :3]
    t_l2c = T_l2c[:3, 3]

    # SVD 清理确保严格正交
    U, _, Vt = np.linalg.svd(R_l2c)
    R_clean = U @ Vt
    if np.linalg.det(R_clean) < 0:
        R_clean = -R_clean

    print(f"\n{'=' * 60}")
    print(f"=== 推导结果：T_lidar→camera ===")
    print(f"{'=' * 60}")
    R_flat = R_clean.flatten()
    print(f"\nR (填入 converter_config_rosbag.yaml):")
    print(f"  data: [{', '.join(f'{v:.8f}' for v in R_flat)}]")
    print(f"\nT (填入 converter_config_rosbag.yaml):")
    print(f"  data: [{', '.join(f'{v:.8f}' for v in t_l2c)}]")
    print(f"\nR 行列式: {np.linalg.det(R_clean):.6f} (应为 1.0)")

    # ── 验证 ──
    # 用 T-DT calibrate 的 5 个世界标志点验证 solvePnP 投影
    landmarks = np.array([
        [5.471, -7.5, 0.0],       # self_FORTRESS
        [10.936, -11.161, 0.868],  # self_Tower
        [25.49, -7.5, 1.24524],    # enemy_Base
        [16.925, -3.625, 1.745],   # enemy_Tower
        [20.20, -10.8, 0.8],      # enemy_High
    ])
    print(f"\n=== 验证：世界点→图像投影 ===")
    for wp in landmarks:
        p_cam = R_w2c @ wp + TVEC
        p_img = CAMERA_MATRIX @ p_cam
        u, v = p_img[0] / p_img[2], p_img[1] / p_img[2]
        print(f"  ({wp[0]:7.3f}, {wp[1]:7.3f}, {wp[2]:5.3f}) → ({u:7.1f}, {v:7.1f})")


if __name__ == "__main__":
    main()

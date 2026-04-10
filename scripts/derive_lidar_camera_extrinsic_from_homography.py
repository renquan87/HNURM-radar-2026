#!/usr/bin/env python3
"""
derive_lidar_camera_extrinsic_from_homography.py — 从手动标定推导 lidar→camera 外参

单流程脚本：
  1) 复用项目的 CalibrationUI 做手动标定（图像来源改为 /compressed_image 订阅）
  2) 标定保存后在 UI 中显示重投影误差 + 可视化，允许重新标定
  3) 手动关闭标定窗口后，用新标定结果 + TF(map←lidar) 推导 lidar→camera 外参

CalibrationUI 行为与项目 perspective_calibrator.py 完全一致：
  - 点击「开始标定」冻结画面
  - 左图点击特征点，右图点击对应地图位置
  - 点击「保存计算」：计算 Homography、保存 JSON、显示重投影误差
  - 保存后不退出，可继续查看误差或重新标定
  - 手动关闭窗口后进入外参推导阶段

用法：
  # 先启动 rosbag + registration
  ros2 bag play <rosbag_path> --clock
  ros2 launch hnurm_radar registration.launch.py

  # 再运行本脚本
  python3 scripts/derive_lidar_camera_extrinsic_from_homography.py \\
      --map-image data/maps/competition_2026/std_map.png
"""

import argparse
import math
import os
import sys
import threading
import time
from collections import deque
from typing import Dict, Tuple

# ── 解决 PyQt5 与 OpenCV 内置 Qt 插件冲突 ──
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import cv2

os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import numpy as np
import rclpy
import yaml
from PyQt5.QtWidgets import QApplication
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# ======================== 工具函数 ========================

def _load_yaml(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _resolve_path(path_str: str, root_dir: str) -> str:
    if os.path.isabs(path_str):
        return path_str
    return os.path.normpath(os.path.join(root_dir, path_str))


def _map_pixel_to_world_matrix(
    map_w: int,
    map_h: int,
    field_w: float,
    field_h: float,
) -> np.ndarray:
    """地图像素坐标 → 世界坐标 的 3×3 变换矩阵。

    约定：地图左下角 = 世界原点 (0,0)，向右 +X，向上 +Y。
    像素坐标原点在左上，因此 Y 轴需要翻转。
    """
    return np.array(
        [
            [field_w / float(map_w), 0.0, 0.0],
            [0.0, -field_h / float(map_h), field_h],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _decompose_w2c_from_homography(K: np.ndarray, H_w2i: np.ndarray) -> np.ndarray:
    """从 H_world→image 和相机内参 K 分解出 T_world→camera (4×4)。

    H_w2i = K @ [r1 | r2 | t]  （地面 z=0 平面的投影）
    """
    K_inv = np.linalg.inv(K)
    B = K_inv @ H_w2i
    b1, b2, b3 = B[:, 0], B[:, 1], B[:, 2]

    s1 = 1.0 / np.linalg.norm(b1)
    s2 = 1.0 / np.linalg.norm(b2)
    scale = 0.5 * (s1 + s2)

    r1 = scale * b1
    r2 = scale * b2
    r3 = np.cross(r1, r2)

    R_approx = np.stack([r1, r2, r3], axis=1)
    U, _, Vt = np.linalg.svd(R_approx)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt

    t = scale * b3
    T_w2c = np.eye(4, dtype=np.float64)
    T_w2c[:3, :3] = R
    T_w2c[:3, 3] = t
    return T_w2c


def _rpy_from_R(R: np.ndarray) -> tuple[float, float, float]:
    """从旋转矩阵提取 (yaw, pitch, roll)。"""
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-8
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return yaw, pitch, roll


def _tf_to_matrix(tf_msg) -> np.ndarray:
    """TransformStamped → 4×4 齐次变换矩阵。"""
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [t.x, t.y, t.z]
    return T


# ======================== ROS 图像桥接节点 ========================

class CompressedImageBridge(Node):
    """将 /compressed_image 解码后写入 perspective_calibrator.camera_image。"""

    def __init__(self, topic: str, pcal_mod):
        super().__init__("compressed_image_bridge_for_manual_calib")
        self._pcal_mod = pcal_mod
        self.create_subscription(CompressedImage, topic, self._cb, 10)
        self.get_logger().info(f"标定图像源: {topic}")

    def _cb(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self._pcal_mod.camera_image = cv_image
        except Exception as ex:
            self.get_logger().warn(f"压缩图像解码失败: {ex}")


# ======================== TF 推导节点 ========================

class DeriveLidarCameraNode(Node):
    """监听 TF(map←lidar)，结合 T_w2c 计算 T_l2c。"""

    def __init__(self, args, T_w2c: np.ndarray, K: np.ndarray):
        super().__init__("derive_lidar_camera_after_manual_calib")
        self.args = args
        self.T_w2c = T_w2c
        self.K = K

        main_cfg = _load_yaml(args.main_config)
        self.tf_source_frame = args.tf_source_frame or str(
            main_cfg.get("camera", {}).get("tf_source_frame", "livox_frame")
        )
        self.tf_target_frame = args.tf_target_frame or str(
            main_cfg.get("camera", {}).get("tf_target_frame", "map")
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.samples_tx = deque(maxlen=args.window)
        self.samples_ty = deque(maxlen=args.window)
        self.samples_tz = deque(maxlen=args.window)
        self.samples_yaw = deque(maxlen=args.window)
        self.last_print_t = 0.0

        self.timer = self.create_timer(args.tf_period, self._tick)
        self.get_logger().info(
            f"Listening TF {self.tf_target_frame} ← {self.tf_source_frame}, "
            f"period={args.tf_period}s"
        )

    def _tick(self):
        try:
            tf_msg = self.tf_buffer.lookup_transform(
                target_frame=self.tf_target_frame,
                source_frame=self.tf_source_frame,
                time=rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF unavailable: {ex}", throttle_duration_sec=5.0)
            return

        T_l2w = _tf_to_matrix(tf_msg)
        T_l2c = self.T_w2c @ T_l2w

        R = T_l2c[:3, :3]
        t = T_l2c[:3, 3]
        yaw, pitch, roll = _rpy_from_R(R)

        self.samples_tx.append(float(t[0]))
        self.samples_ty.append(float(t[1]))
        self.samples_tz.append(float(t[2]))
        self.samples_yaw.append(float(yaw))

        now = time.time()
        if now - self.last_print_t < self.args.print_period:
            return
        self.last_print_t = now

        tx_m = float(np.median(self.samples_tx)) if self.samples_tx else float(t[0])
        ty_m = float(np.median(self.samples_ty)) if self.samples_ty else float(t[1])
        tz_m = float(np.median(self.samples_tz)) if self.samples_tz else float(t[2])
        yaw_m = float(np.median(self.samples_yaw)) if self.samples_yaw else float(yaw)

        self.get_logger().info("=" * 64)
        self.get_logger().info(
            f"T_l2c instant: t=({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f}), "
            f"ypr=({yaw:.6f}, {pitch:.6f}, {roll:.6f})"
        )
        self.get_logger().info(
            f"T_l2c median[{len(self.samples_tx)}/{self.args.window}]: "
            f"tx={tx_m:.4f}, ty={ty_m:.4f}, tz={tz_m:.4f}, yaw={yaw_m:.6f}"
        )
        self.get_logger().info("R/T yaml snippet:")
        r_flat = R.reshape(-1)
        self.get_logger().info(
            "R.data: [" + ", ".join(f"{v:.8f}" for v in r_flat) + "]"
        )
        self.get_logger().info(
            "T.data: [" + ", ".join(f"{v:.8f}" for v in t) + "]"
        )

        if self.args.print_matrix:
            self.get_logger().info("T_l2c 4x4:")
            self.get_logger().info(
                "\n" + np.array2string(T_l2c, precision=6, suppress_small=False)
            )


# ======================== 标定阶段 ========================

def _run_manual_calibration(args, main_cfg: dict) -> tuple[np.ndarray, np.ndarray]:
    """复用项目 CalibrationUI 执行手动标定。

    返回:
        (T_w2c, K) — 从 homography 分解得到的 T_world→camera 和相机内参矩阵
    """
    # ---- 导入项目的 perspective_calibrator 模块 ----
    project_root = os.path.dirname(
        os.path.dirname(os.path.abspath(args.main_config))
    )
    pkg_root = os.path.join(project_root, "src", "hnurm_radar")
    if pkg_root not in sys.path:
        sys.path.insert(0, pkg_root)

    from hnurm_radar.camera_locator import perspective_calibrator as pcal_mod

    # ---- 覆盖保存路径 ----
    pcal_mod.PERSPECTIVE_CALIB_PATH = args.perspective_calib

    # ---- 覆盖地图路径（默认使用 std_map.png） ----
    if args.map_image:
        map_path = _resolve_path(args.map_image, project_root)
    else:
        # 默认使用 std_map.png
        map_path = _resolve_path(
            "data/maps/competition_2026/std_map.png", project_root
        )
    pcal_mod.PFA_MAP_RED_PATH = map_path
    pcal_mod.PFA_MAP_BLUE_PATH = map_path
    pcal_mod.PFA_MAP_2025_PATH = map_path

    old_mtime = (
        os.path.getmtime(args.perspective_calib)
        if os.path.exists(args.perspective_calib)
        else None
    )

    my_color = str(main_cfg.get("global", {}).get("my_color", "Blue"))
    topic = args.compressed_image_topic or str(
        main_cfg.get("camera", {}).get(
            "compressed_image_topic", "/compressed_image"
        )
    )

    # ---- 启动 ROS 图像桥接 ----
    bridge = CompressedImageBridge(topic, pcal_mod)
    stop_evt = threading.Event()

    def _spin_ros():
        while rclpy.ok() and not stop_evt.is_set():
            rclpy.spin_once(bridge, timeout_sec=0.05)

    spin_thread = threading.Thread(target=_spin_ros, daemon=True)
    spin_thread.start()

    # 等待首帧图像
    print("[标定工具] 等待 /compressed_image 首帧图像 ...")
    for _ in range(200):  # 最多等 10 秒
        if pcal_mod.camera_image is not None:
            break
        time.sleep(0.05)
    if pcal_mod.camera_image is None:
        print("[标定工具] ⚠ 超时！未收到图像，使用空白图像。")
    else:
        h, w = pcal_mod.camera_image.shape[:2]
        print(f"[标定工具] ✔ 收到首帧图像 ({w}×{h})")

    # ---- 启动标定 UI ----
    # CalibrationUI 行为与项目完全一致：
    #   保存后调用 _show_reprojection_error() 显示误差
    #   不会自动退出，用户可继续重新标定
    #   只有手动关闭窗口才退出
    app = QApplication.instance() or QApplication(sys.argv)
    ui = pcal_mod.CalibrationUI(my_color)
    print(
        "[标定工具] 标定窗口已打开。\n"
        "  操作流程:\n"
        "    1. 点击「开始标定」冻结画面\n"
        "    2. 左图点击地面特征点，右图点击对应地图位置（≥4 对）\n"
        "    3. 点击「保存计算」→ 显示重投影误差\n"
        "    4. 误差满意则关闭窗口进入外参推导；不满意可撤销重标\n"
        "  ⚠ 关闭窗口前必须至少点击一次「保存计算」！"
    )
    app.exec_()

    # ---- 清理 ROS 桥接 ----
    stop_evt.set()
    spin_thread.join(timeout=1.0)
    bridge.destroy_node()

    # ---- 验证标定文件已更新 ----
    if not os.path.exists(args.perspective_calib):
        raise RuntimeError(
            f"未生成标定文件: {args.perspective_calib}\n"
            "请在标定界面点击「保存计算」后再关闭窗口。"
        )
    new_mtime = os.path.getmtime(args.perspective_calib)
    if old_mtime is not None and abs(new_mtime - old_mtime) < 1e-6:
        raise RuntimeError(
            "检测到标定文件未更新（仍是旧数据）。\n"
            "请在标定界面点击「保存计算」后再关闭窗口。"
        )

    # ---- 从新标定结果推导 T_w2c ----
    import json

    with open(args.perspective_calib, "r", encoding="utf-8") as f:
        pcal_data = json.load(f)

    if "H_ground" in pcal_data:
        H_i2m = np.array(pcal_data["H_ground"], dtype=np.float64)
    elif "H" in pcal_data:
        H_i2m = np.array(pcal_data["H"], dtype=np.float64)
    else:
        raise ValueError(
            f"标定文件缺少 H_ground/H: {args.perspective_calib}"
        )

    map_w = int(pcal_data.get("map_w", 2800))
    map_h = int(pcal_data.get("map_h", 1500))

    # 读取相机内参
    conv_cfg = _load_yaml(args.converter_config)
    intr = conv_cfg["calib"]["intrinsic"]
    K = np.array(
        [
            [float(intr["fx"]), 0.0, float(intr["cx"])],
            [0.0, float(intr["fy"]), float(intr["cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    # 场景尺寸
    scene_name = str(main_cfg.get("global", {}).get("scene", "competition"))
    scene_cfg = main_cfg.get("scenes", {}).get(scene_name, {})
    field_w = float(scene_cfg.get("field_width", 28.0))
    field_h = float(scene_cfg.get("field_height", 15.0))

    # 地图像素 → 世界坐标 的变换矩阵
    A_m2w = _map_pixel_to_world_matrix(map_w, map_h, field_w, field_h)

    # H_i2w: 图像像素 → 世界坐标
    H_i2w = A_m2w @ H_i2m
    # H_w2i: 世界坐标 → 图像像素
    H_w2i = np.linalg.inv(H_i2w)

    # 分解得到 T_w2c
    T_w2c = _decompose_w2c_from_homography(K, H_w2i)

    yaw, pitch, roll = _rpy_from_R(T_w2c[:3, :3])
    print("\n" + "=" * 64)
    print("[标定结果] T_world→camera (从 homography 分解):")
    print(f"  yaw   = {yaw:.6f} rad ({math.degrees(yaw):.2f}°)")
    print(f"  pitch = {pitch:.6f} rad ({math.degrees(pitch):.2f}°)")
    print(f"  roll  = {roll:.6f} rad ({math.degrees(roll):.2f}°)")
    print(f"  t     = {T_w2c[:3, 3].tolist()}")
    print(f"  det(R)= {np.linalg.det(T_w2c[:3, :3]):.6f}")
    print("=" * 64)

    # 打印标定点对信息
    if "pixel_points_ground" in pcal_data and "field_points_ground" in pcal_data:
        img_pts = pcal_data["pixel_points_ground"]
        map_pts = pcal_data["field_points_ground"]
        print(f"\n[标定点对] {len(img_pts)} 对:")
        for i, (ip, mp) in enumerate(zip(img_pts, map_pts)):
            # 将地图像素转为世界坐标
            mp_homo = np.array([mp[0], mp[1], 1.0])
            wp = A_m2w @ mp_homo
            wp = wp[:2] / wp[2]
            print(
                f"  P{i+1}: 图像({ip[0]}, {ip[1]}) → "
                f"地图像素({mp[0]}, {mp[1]}) → "
                f"世界({wp[0]:.2f}m, {wp[1]:.2f}m)"
            )

    return T_w2c, K


# ======================== 参数解析 ========================

def _build_parser():
    p = argparse.ArgumentParser(
        description=(
            "从手动标定 homography 推导 lidar→camera 外参。\n"
            "流程：手动标定 → T_w2c 分解 → TF 监听 → T_l2c 输出"
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "--converter-config",
        default="configs/converter_config_rosbag.yaml",
        help="converter 配置文件路径（读取相机内参）",
    )
    p.add_argument(
        "--perspective-calib",
        default="configs/perspective_calib.json",
        help="标定结果保存路径",
    )
    p.add_argument(
        "--main-config",
        default="configs/main_config.yaml",
        help="main_config.yaml 路径",
    )
    p.add_argument(
        "--compressed-image-topic",
        default="",
        help="压缩图像 topic（默认从 main_config 读取）",
    )
    p.add_argument(
        "--map-image",
        default="",
        help="标定用地图路径（默认 std_map.png）",
    )
    p.add_argument(
        "--tf-source-frame",
        default="",
        help="TF 源坐标系（默认从 main_config 读取，一般为 livox_frame）",
    )
    p.add_argument(
        "--tf-target-frame",
        default="",
        help="TF 目标坐标系（默认从 main_config 读取，一般为 map）",
    )
    p.add_argument(
        "--tf-period",
        type=float,
        default=0.2,
        help="TF 查询周期(秒)",
    )
    p.add_argument(
        "--print-period",
        type=float,
        default=1.0,
        help="打印周期(秒)",
    )
    p.add_argument(
        "--window",
        type=int,
        default=30,
        help="中值滤波窗口大小",
    )
    p.add_argument(
        "--print-matrix",
        action="store_true",
        help="额外打印完整 4×4 变换矩阵",
    )
    return p


# ======================== 主入口 ========================

def main():
    args = _build_parser().parse_args()
    main_cfg = _load_yaml(args.main_config)

    # 将相对路径解析为绝对路径
    project_root = os.path.dirname(
        os.path.dirname(os.path.abspath(args.main_config))
    )
    args.perspective_calib = _resolve_path(args.perspective_calib, project_root)
    args.converter_config = _resolve_path(args.converter_config, project_root)
    args.main_config = _resolve_path(args.main_config, project_root)

    rclpy.init()
    node = None
    try:
        # ---- 阶段 1：手动标定 ----
        T_w2c, K = _run_manual_calibration(args, main_cfg)

        # ---- 阶段 2：TF 监听推导外参 ----
        print("\n[外参推导] 开始监听 TF，持续输出 T_lidar→camera ...")
        print("[外参推导] 按 Ctrl+C 停止\n")
        node = DeriveLidarCameraNode(args, T_w2c, K)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[外参推导] 已停止。")
    except RuntimeError as e:
        print(f"\n[错误] {e}")
        sys.exit(1)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

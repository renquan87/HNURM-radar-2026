#!/usr/bin/env python3
"""
record_initial_pose.py — 初始位姿记录工具

在 RViz/Foxglove Studio 中可视化 PCD 地图，通过 "2D Pose Estimate" 工具
为红方/蓝方分别设置雷达初始位姿，保存到 configs/main_config.yaml。

脚本会自动启动 foxglove_bridge 节点，用户可直接通过 Foxglove Studio
连接 ws://localhost:8765 来进行可视化操作（比 RViz 更便捷）。
若不需要自动启动 foxglove_bridge，可使用 --no-foxglove 参数跳过。

使用方法:
    # 使用配置文件中的 PCD（根据当前 scene 自动选择）
    python3 scripts/record_initial_pose.py

    # 指定 PCD 文件
    python3 scripts/record_initial_pose.py --pcd data/pointclouds/registration/pcds_downsampled.pcd

    # 跳过自动启动 foxglove_bridge（例如已有在运行的实例）
    python3 scripts/record_initial_pose.py --no-foxglove

流程:
    1. 启动后自动启动 foxglove_bridge（可通过 --no-foxglove 跳过）
    2. 加载并持续发布 PCD 地图到 /map_cloud (frame: map)
    3. 在 Foxglove Studio 或 RViz 中用 "2D Pose Estimate" (Publish) 设置红方初始位姿 → 按 Enter 确认
    4. 同样设置蓝方初始位姿 → 按 Enter 确认
    5. 写入 configs/main_config.yaml 的 initial_pose 字段

依赖: rclpy, open3d, numpy, ruamel.yaml, sensor_msgs_py (或 geometry_msgs)
"""

import argparse
import contextlib
import math
import os
import signal
import subprocess
import sys
import threading

import numpy as np

# ── 项目根目录定位（与 paths.py 保持一致的回溯方式） ──
# 本脚本位于 scripts/record_initial_pose.py，向上 1 层即项目根目录
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))

MAIN_CONFIG_PATH = os.path.join(PROJECT_ROOT, "configs", "main_config.yaml")


# ═══════════════════════════════════════════════════════════
#  Foxglove Bridge 管理
# ═══════════════════════════════════════════════════════════

_foxglove_process: subprocess.Popen | None = None


def start_foxglove_bridge() -> subprocess.Popen | None:
    """启动 foxglove_bridge 子进程，返回 Popen 对象。

    启动失败时打印警告并返回 None，不阻断主流程。
    """
    global _foxglove_process  # noqa: PLW0603
    cmd = ["ros2", "run", "foxglove_bridge", "foxglove_bridge"]
    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,  # 新建进程组，方便整组清理
        )
        _foxglove_process = proc
        print("🦊 foxglove_bridge 已启动 (PID: {})".format(proc.pid))
        print("   👉 请用 Foxglove Studio 连接: ws://localhost:8765")
        return proc
    except FileNotFoundError:
        print("⚠️  未找到 foxglove_bridge，跳过自动启动")
        print("   请确认已安装: sudo apt install ros-${ROS_DISTRO}-foxglove-bridge")
        return None
    except Exception as e:  # noqa: BLE001
        print(f"⚠️  启动 foxglove_bridge 失败: {e}，跳过")
        return None


def stop_foxglove_bridge() -> None:
    """关闭 foxglove_bridge 子进程（及其子进程组）。"""
    global _foxglove_process  # noqa: PLW0603
    if _foxglove_process is None:
        return
    try:
        # 发送 SIGTERM 到整个进程组
        os.killpg(os.getpgid(_foxglove_process.pid), signal.SIGTERM)
        _foxglove_process.wait(timeout=5)
        print("🦊 foxglove_bridge 已关闭")
    except ProcessLookupError:
        pass  # 进程已退出
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(_foxglove_process.pid), signal.SIGKILL)
        print("🦊 foxglove_bridge 已强制关闭")
    except Exception:  # noqa: BLE001
        pass
    finally:
        _foxglove_process = None


# ═══════════════════════════════════════════════════════════
#  辅助函数
# ═══════════════════════════════════════════════════════════

def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """从四元数提取 yaw 角（弧度）。"""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def load_pcd_path_from_config() -> str:
    """从 main_config.yaml 读取当前使用的 PCD 文件路径。

    与 registration_node 使用相同的 PCD 选择逻辑（见 registration.launch.py）：
      1. 若 camera.mode == "rosbag" 且 camera.rosbag_pcd_file 存在 → 使用 rosbag PCD
      2. 否则使用当前 scene 的 downsampled_pcd（优先）或 pcd_file
    """
    from ruamel.yaml import YAML

    yaml = YAML()
    with open(MAIN_CONFIG_PATH, encoding="utf-8") as f:
        cfg = yaml.load(f)

    # ── 检查 rosbag 模式覆盖（与 registration.launch.py 保持一致） ──
    camera_cfg = cfg.get("camera", {})
    camera_mode = camera_cfg.get("mode", "")
    rosbag_pcd_rel = camera_cfg.get("rosbag_pcd_file", "")

    if camera_mode == "rosbag" and rosbag_pcd_rel:
        rosbag_pcd_abs = (
            os.path.join(PROJECT_ROOT, rosbag_pcd_rel)
            if not os.path.isabs(rosbag_pcd_rel)
            else rosbag_pcd_rel
        )
        if os.path.isfile(rosbag_pcd_abs):
            print(f"📦 rosbag 模式: 使用 rosbag_pcd_file = {rosbag_pcd_abs}")
            return rosbag_pcd_abs
        else:
            print(f"⚠️  rosbag_pcd_file 不存在: {rosbag_pcd_abs}，回退到场景 PCD")

    # ── 使用当前 scene 配置的 PCD ──
    scene_name = cfg.get("global", {}).get("scene", "competition")
    scenes = cfg.get("scenes", {})
    scene_cfg = scenes.get(scene_name, {})

    # 优先 downsampled_pcd（文件更小、发布更快）
    pcd_rel = scene_cfg.get("downsampled_pcd") or scene_cfg.get("pcd_file", "")
    if not pcd_rel:
        print("❌ 配置文件中未找到 PCD 文件路径，请使用 --pcd 参数指定")
        sys.exit(1)

    pcd_abs = os.path.join(PROJECT_ROOT, pcd_rel) if not os.path.isabs(pcd_rel) else pcd_rel
    print(f"🗺️  scene={scene_name}, PCD = {pcd_abs}")
    return pcd_abs


def load_pcd_as_numpy(pcd_path: str) -> np.ndarray:
    """用 open3d 读取 PCD 文件，返回 (N, 3) float32 数组。"""
    import open3d as o3d

    if not os.path.isfile(pcd_path):
        print(f"❌ PCD 文件不存在: {pcd_path}")
        sys.exit(1)

    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points, dtype=np.float32)
    print(f"✅ 已加载 PCD 文件: {pcd_path}  ({len(points)} 个点)")
    return points


def build_pointcloud2_msg(points: np.ndarray, stamp):
    """手动构建 sensor_msgs/PointCloud2 消息（不依赖 sensor_msgs_py）。

    Parameters
    ----------
    points : np.ndarray
        形状 (N, 3)，dtype float32
    stamp : rclpy 时间戳

    Returns
    -------
    sensor_msgs.msg.PointCloud2
    """
    from sensor_msgs.msg import PointCloud2, PointField

    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.header.stamp = stamp

    msg.height = 1
    msg.width = len(points)
    msg.is_dense = True
    msg.is_bigendian = False

    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 12  # 3 × float32
    msg.row_step = msg.point_step * msg.width

    # 确保 C-contiguous float32
    pts = np.ascontiguousarray(points[:, :3].astype(np.float32))
    msg.data = pts.tobytes()

    return msg


def save_poses_to_config(red_pose: dict, blue_pose: dict) -> None:
    """将红方/蓝方初始位姿写入 main_config.yaml，保留其他配置不变。"""
    from ruamel.yaml import YAML

    yaml = YAML()
    yaml.preserve_quotes = True

    with open(MAIN_CONFIG_PATH, encoding="utf-8") as f:
        cfg = yaml.load(f)

    # 确保 initial_pose 节点存在
    if "initial_pose" not in cfg:
        cfg["initial_pose"] = {}

    if "red" not in cfg["initial_pose"]:
        cfg["initial_pose"]["red"] = {}
    if "blue" not in cfg["initial_pose"]:
        cfg["initial_pose"]["blue"] = {}

    for key in ("x", "y", "z", "yaw"):
        cfg["initial_pose"]["red"][key] = round(red_pose[key], 5)
        cfg["initial_pose"]["blue"][key] = round(blue_pose[key], 5)

    with open(MAIN_CONFIG_PATH, "w", encoding="utf-8") as f:
        yaml.dump(cfg, f)

    print(f"\n✅ 已保存到 {MAIN_CONFIG_PATH}")


# ═══════════════════════════════════════════════════════════
#  ROS2 节点
# ═══════════════════════════════════════════════════════════

class InitialPoseRecorder:
    """ROS2 节点：发布 PCD 地图 + 监听 /initialpose。"""

    def __init__(self, node, points: np.ndarray):
        self.node = node
        self.points = points
        self.latest_pose: dict | None = None
        self._pose_lock = threading.Lock()

        # 发布者：PCD 地图
        from sensor_msgs.msg import PointCloud2

        self.map_pub = node.create_publisher(PointCloud2, "/map_cloud", 10)

        # 预构建消息（只需更新时间戳）
        self._cloud_msg = build_pointcloud2_msg(
            points, node.get_clock().now().to_msg()
        )

        # 定时发布 PCD（1Hz）
        self.timer = node.create_timer(1.0, self._publish_map)

        # 订阅 /initialpose
        from geometry_msgs.msg import PoseWithCovarianceStamped

        node.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self._initialpose_callback,
            10,
        )

        node.get_logger().info("节点已启动，正在以 1Hz 发布 PCD 地图到 /map_cloud (frame: map)")

    def _publish_map(self):
        """定时发布 PCD 地图。"""
        self._cloud_msg.header.stamp = self.node.get_clock().now().to_msg()
        self.map_pub.publish(self._cloud_msg)

    def _initialpose_callback(self, msg):
        """收到 /initialpose 时提取并暂存位姿。"""
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation

        yaw = quaternion_to_yaw(ori.x, ori.y, ori.z, ori.w)

        pose = {
            "x": pos.x,
            "y": pos.y,
            "z": pos.z,
            "yaw": yaw,
        }

        with self._pose_lock:
            self.latest_pose = pose

        self.node.get_logger().info(
            f"收到 /initialpose: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}, yaw={yaw:.4f} rad ({math.degrees(yaw):.1f}°)"
        )

    def get_and_clear_pose(self) -> dict | None:
        """获取最新位姿并清除（线程安全）。"""
        with self._pose_lock:
            pose = self.latest_pose
            self.latest_pose = None
            return pose

    def peek_pose(self) -> dict | None:
        """查看最新位姿但不清除。"""
        with self._pose_lock:
            return self.latest_pose


# ═══════════════════════════════════════════════════════════
#  交互式流程
# ═══════════════════════════════════════════════════════════

def record_one_side(recorder: InitialPoseRecorder, side_name: str) -> dict:
    """交互式记录一侧的初始位姿。

    Parameters
    ----------
    recorder : InitialPoseRecorder
    side_name : str
        "红方" 或 "蓝方"

    Returns
    -------
    dict : {"x": ..., "y": ..., "z": ..., "yaw": ...}
    """
    # 清除之前可能残留的位姿
    recorder.get_and_clear_pose()

    print(f"\n{'='*60}")
    print(f"  请在 Foxglove Studio 或 RViz 中使用 '2D Pose Estimate' 工具")
    print(f"  设置【{side_name}】雷达初始位姿")
    print("  提示: 在 PCD 地图上点击并拖拽，设置雷达位置和朝向")
    print(f"{'='*60}")

    # 等待用户在 Foxglove Studio / RViz 中设置位姿
    while True:
        pose = recorder.peek_pose()
        if pose is not None:
            print(f"\n📍 已接收到【{side_name}】位姿:")
            print(f"   x = {pose['x']:.4f}")
            print(f"   y = {pose['y']:.4f}")
            print(f"   z = {pose['z']:.4f}")
            print(f"   yaw = {pose['yaw']:.5f} rad ({math.degrees(pose['yaw']):.1f}°)")

            user_input = input("\n按 Enter 确认此位姿，输入 'r' 重新设置 >>> ").strip().lower()
            if user_input == "r":
                recorder.get_and_clear_pose()
                print(f"🔄 已清除，请重新在 Foxglove/RViz 中设置【{side_name}】位姿...")
                continue
            else:
                # 确认
                confirmed_pose = recorder.get_and_clear_pose()
                if confirmed_pose is None:
                    confirmed_pose = pose  # 使用 peek 到的值
                print(f"✅ 【{side_name}】位姿已确认")
                return confirmed_pose
        else:
            with contextlib.suppress(EOFError):
                input("等待 /initialpose 消息... 在 Foxglove/RViz 中设置后按 Enter 检查 >>> ")


def interactive_flow(recorder: InitialPoseRecorder) -> None:
    """主交互流程：依次记录红方和蓝方位姿，然后保存。"""
    print("\n" + "=" * 60)
    print("  初始位姿记录工具")
    print("  PCD 地图已发布到 /map_cloud (frame: map)")
    print("  可视化方式（二选一）:")
    print("    • Foxglove Studio: 连接 ws://localhost:8765，添加 3D 面板")
    print("    • RViz: 添加 PointCloud2 显示，订阅 /map_cloud")
    print("  使用 '2D Pose Estimate' (Publish) 工具发布位姿到 /initialpose")
    print("=" * 60)

    # 记录红方
    red_pose = record_one_side(recorder, "红方")

    # 记录蓝方
    blue_pose = record_one_side(recorder, "蓝方")

    # 汇总显示
    print("\n" + "=" * 60)
    print("  位姿记录汇总")
    print("=" * 60)
    print(f"  红方: x={red_pose['x']:.4f}, y={red_pose['y']:.4f}, "
          f"z={red_pose['z']:.4f}, yaw={red_pose['yaw']:.5f}")
    print(f"  蓝方: x={blue_pose['x']:.4f}, y={blue_pose['y']:.4f}, "
          f"z={blue_pose['z']:.4f}, yaw={blue_pose['yaw']:.5f}")

    confirm = input("\n按 Enter 保存到配置文件，输入 'q' 放弃 >>> ").strip().lower()
    if confirm == "q":
        print("❌ 已放弃，未保存")
        return

    save_poses_to_config(red_pose, blue_pose)

    print("\n📋 配置文件中 initial_pose 内容:")
    print("  initial_pose:")
    print("    red:")
    print(f"      x: {round(red_pose['x'], 5)}")
    print(f"      y: {round(red_pose['y'], 5)}")
    print(f"      z: {round(red_pose['z'], 5)}")
    print(f"      yaw: {round(red_pose['yaw'], 5)}")
    print("    blue:")
    print(f"      x: {round(blue_pose['x'], 5)}")
    print(f"      y: {round(blue_pose['y'], 5)}")
    print(f"      z: {round(blue_pose['z'], 5)}")
    print(f"      yaw: {round(blue_pose['yaw'], 5)}")


# ═══════════════════════════════════════════════════════════
#  入口
# ═══════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="初始位姿记录工具 — 通过 Foxglove Studio 或 RViz 的 2D Pose Estimate 记录红方/蓝方初始位姿",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 scripts/record_initial_pose.py
  python3 scripts/record_initial_pose.py --pcd data/pointclouds/registration/pcds_downsampled.pcd
  python3 scripts/record_initial_pose.py --no-foxglove
        """,
    )
    parser.add_argument(
        "--pcd",
        type=str,
        default=None,
        help="PCD 文件路径（覆盖 main_config.yaml 中的配置）",
    )
    parser.add_argument(
        "--no-foxglove",
        action="store_true",
        default=False,
        help="跳过自动启动 foxglove_bridge（适用于已有运行实例的情况）",
    )
    args = parser.parse_args()

    # ── 启动 Foxglove Bridge ──
    if not args.no_foxglove:
        start_foxglove_bridge()
    else:
        print("ℹ️  已跳过 foxglove_bridge 自动启动（--no-foxglove）")

    # ── 确定 PCD 路径 ──
    if args.pcd:
        pcd_path = args.pcd
        if not os.path.isabs(pcd_path):
            pcd_path = os.path.join(PROJECT_ROOT, pcd_path)
    else:
        pcd_path = load_pcd_path_from_config()

    # ── 加载 PCD ──
    points = load_pcd_as_numpy(pcd_path)

    # ── 初始化 ROS2 ──
    import rclpy

    rclpy.init()
    node = rclpy.create_node("initial_pose_recorder")

    recorder = InitialPoseRecorder(node, points)

    # ── 启动 ROS spin 线程 ──
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ── 主线程运行交互流程 ──
    try:
        interactive_flow(recorder)
    except KeyboardInterrupt:
        print("\n\n⚠️  用户中断，退出")
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001
            pass  # 可能已被 signal handler 调用过
        stop_foxglove_bridge()
        print("节点已关闭")


if __name__ == "__main__":
    main()

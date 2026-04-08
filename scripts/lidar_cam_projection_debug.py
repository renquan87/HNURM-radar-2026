#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_cam_projection_debug.py

用途：将 /lidar_pcds 点云按当前外参投影到图像，快速排查雷达-相机外参。

增强点（针对“整屏糊成一片看不出对齐”的问题）：
1) 支持“仅显示检测框附近点云”(box-only)，默认开启（当 --use-detect-box 时）
2) 支持“每像素仅保留最近深度点”，去除穿透叠影
3) 支持 LiDAR 空间预裁剪（x/y/z），减少无关背景
4) 修复 dt 异常巨大：对无效时间戳做兼容处理
5) Ctrl+C 退出时避免重复 shutdown 报错
"""

import os
import cv2
import yaml
import time
import argparse
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from cv_bridge import CvBridge

try:
    from detect_result.msg import Robots
    HAS_ROBOTS_MSG = True
except Exception:
    HAS_ROBOTS_MSG = False
    Robots = None


def _safe_stamp_to_sec(stamp):
    t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    if t < 1e-3:
        return None
    return t


def _build_parser():
    p = argparse.ArgumentParser(description="雷达点云投影到图像外参排查脚本")

    p.add_argument("--config", default="configs/converter_config.yaml", help="外参/内参配置文件")
    p.add_argument("--pc-topic", default="/lidar_pcds", help="点云话题")
    p.add_argument("--img-topic", default="/detect_view", help="图像话题")
    p.add_argument("--det-topic", default="/detect_result", help="检测框话题")

    p.add_argument("--use-detect-box", action="store_true", help="启用检测框辅助统计")
    p.add_argument("--render-mode", default="auto", choices=["auto", "full", "box-only", "overlay3d"],
                   help="可视化模式: auto|full|box-only|overlay3d")
    p.add_argument("--view-mode", default="both", choices=["overlay", "fusion", "both"],
                   help="显示模式: overlay=原图叠加, fusion=点云视角融合, both=两个窗口都显示")
    p.add_argument("--box-margin", type=int, default=25, help="框过滤扩张像素")

    p.add_argument("--min-depth", type=float, default=0.8, help="相机坐标最小深度(m)")
    p.add_argument("--max-depth", type=float, default=25.0, help="相机坐标最大深度(m)")
    p.add_argument("--sample-step", type=int, default=3, help="点云抽样步长")

    p.add_argument("--x-min", type=float, default=0.0, help="LiDAR坐标x最小(前向)")
    p.add_argument("--x-max", type=float, default=20.0, help="LiDAR坐标x最大(前向)")
    p.add_argument("--y-min", type=float, default=-12.0, help="LiDAR坐标y最小(右侧负)")
    p.add_argument("--y-max", type=float, default=12.0, help="LiDAR坐标y最大(左侧正)")
    p.add_argument("--z-min", type=float, default=-2.5, help="LiDAR坐标z最小")
    p.add_argument("--z-max", type=float, default=2.5, help="LiDAR坐标z最大")

    p.add_argument("--nearest-per-pixel", action="store_true", help="每像素仅保留最近深度")
    p.add_argument("--max-draw-points", type=int, default=30000, help="最多绘制点数")

    p.add_argument("--silhouette-dilate", type=int, default=3, help="轮廓膨胀核")
    p.add_argument("--point-radius", type=int, default=1, help="投影点半径")
    p.add_argument("--fusion-image-alpha", type=float, default=0.35,
                   help="fusion 模式中相机图像透明度(0~1)，越大图像越明显")
    p.add_argument("--overlay3d-alpha", type=float, default=0.5,
                   help="overlay3d 模式中相机图像混合比例(0~1)，按 +/- 可调")
    p.add_argument("--overlay3d-max-width", type=int, default=1920,
                   help="overlay3d 模式显示窗口最大宽度，超过则降采样")

    p.add_argument("--sync-tol", type=float, default=0.2, help="图点最大时间差(s)")
    p.add_argument("--log-interval", type=float, default=1.0, help="日志间隔(s)")

    p.add_argument("--publish-topic", default="/lidar_cam_projection_debug", help="叠加图发布话题")
    p.add_argument("--no-window", action="store_true", help="不弹窗")

    p.add_argument("--save-dir", default="test_output/lidar_cam_projection", help="保存目录")
    p.add_argument("--save-every", type=int, default=0, help="每N帧保存，0不保存")
    return p


class LidarCamProjectionDebugger(Node):
    def __init__(self, args):
        super().__init__("lidar_cam_projection_debugger")
        self.args = args
        self.bridge = CvBridge()

        self._load_calib(args.config)

        qos_img = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        qos_pc = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.sub_pc = self.create_subscription(PointCloud2, args.pc_topic, self._pc_cb, qos_pc)
        self.sub_img = self.create_subscription(Image, args.img_topic, self._img_cb, qos_img)

        self.use_detect_box = bool(args.use_detect_box and HAS_ROBOTS_MSG)
        self.latest_boxes = []
        self.latest_box_recv_t = None
        if args.use_detect_box and not HAS_ROBOTS_MSG:
            self.get_logger().warning("--use-detect-box 已请求，但 detect_result.msg.Robots 不可用，自动关闭")

        if self.use_detect_box:
            self.sub_det = self.create_subscription(Robots, args.det_topic, self._det_cb, qos_img)
        else:
            self.sub_det = None

        self.pub_dbg = self.create_publisher(Image, args.publish_topic, qos_img)

        self.latest_points = None
        self.latest_pc_stamp = None
        self.latest_pc_recv_t = None

        self.frame_idx = 0
        self.last_log_t = time.time()

        # overlay3d 模式的动态 alpha（可按键调节）
        self._overlay3d_alpha = float(np.clip(args.overlay3d_alpha, 0.0, 1.0))

        os.makedirs(args.save_dir, exist_ok=True)

        self.get_logger().info("=== lidar_cam_projection_debugger 已启动 ===")
        self.get_logger().info(f"config={args.config}")
        self.get_logger().info(f"pc_topic={args.pc_topic}, img_topic={args.img_topic}, det_topic={args.det_topic}")
        self.get_logger().info(f"calib image size=({self.calib_w}x{self.calib_h}), depth=[{args.min_depth}, {args.max_depth}]")
        self.get_logger().info(
            f"lidar crop x[{args.x_min},{args.x_max}] y[{args.y_min},{args.y_max}] z[{args.z_min},{args.z_max}]"
        )

    def _load_calib(self, cfg_path: str):
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)

        calib = cfg["calib"]
        ext = calib["extrinsic"]

        self.R = np.array(ext["R"]["data"], dtype=np.float64).reshape(3, 3)
        self.T = np.array(ext["T"]["data"], dtype=np.float64).reshape(1, 3)

        intr = calib["intrinsic"]
        self.fx = float(intr["fx"])
        self.fy = float(intr["fy"])
        self.cx = float(intr["cx"])
        self.cy = float(intr["cy"])
        self.K = np.array(
            [[self.fx, 0.0, self.cx],
             [0.0, self.fy, self.cy],
             [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        dist = calib.get("distortion", {}).get("data", [0, 0, 0, 0, 0])
        self.dist = np.array(dist, dtype=np.float64).reshape(-1, 1)

        p = cfg.get("params", {})
        self.calib_w = int(p.get("width", 3072))
        self.calib_h = int(p.get("height", 2048))

    def _pc_cb(self, msg: PointCloud2):
        pts = np.array(
            list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)),
            dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        if len(pts) == 0:
            return

        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=-1).astype(np.float64)

        # LiDAR 空间预裁剪（先减负担）
        m = (
            (xyz[:, 0] >= self.args.x_min) & (xyz[:, 0] <= self.args.x_max) &
            (xyz[:, 1] >= self.args.y_min) & (xyz[:, 1] <= self.args.y_max) &
            (xyz[:, 2] >= self.args.z_min) & (xyz[:, 2] <= self.args.z_max)
        )
        xyz = xyz[m]
        if len(xyz) == 0:
            return

        step = max(1, int(self.args.sample_step))
        if step > 1:
            xyz = xyz[::step]

        self.latest_points = xyz
        self.latest_pc_stamp = _safe_stamp_to_sec(msg.header.stamp)
        self.latest_pc_recv_t = time.time()

    def _det_cb(self, msg: Robots):
        boxes = []
        for d in msg.detect_results:
            x1, y1, x2, y2 = [int(v) for v in d.xyxy_box]
            boxes.append((x1, y1, x2, y2, d.label, int(d.track_id)))
        self.latest_boxes = boxes
        self.latest_box_recv_t = time.time()

    def _project_lidar_to_img(self, lidar_xyz: np.ndarray, out_w: int, out_h: int):
        # lidar -> camera
        cam_xyz = lidar_xyz @ self.R.T + self.T

        z = cam_xyz[:, 2]
        valid_z = (z > self.args.min_depth) & (z < self.args.max_depth)
        cam_valid = cam_xyz[valid_z]

        if len(cam_valid) == 0:
            return np.empty((0, 2), np.float32), np.empty((0,), np.float32), {
                "n_total": int(len(lidar_xyz)),
                "n_front": 0,
                "n_in_img": 0,
                "n_outside": 0,
            }

        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)
        img_pts, _ = cv2.projectPoints(cam_valid, rvec, tvec, self.K, self.dist)
        uv = img_pts.reshape(-1, 2)

        sx = float(out_w) / float(self.calib_w)
        sy = float(out_h) / float(self.calib_h)
        uv[:, 0] *= sx
        uv[:, 1] *= sy

        in_img = (
            (uv[:, 0] >= 0) & (uv[:, 0] < out_w) &
            (uv[:, 1] >= 0) & (uv[:, 1] < out_h)
        )
        uv_in = uv[in_img].astype(np.float32)
        depth_in = cam_valid[in_img, 2].astype(np.float32)

        stats = {
            "n_total": int(len(lidar_xyz)),
            "n_front": int(len(cam_valid)),
            "n_in_img": int(len(uv_in)),
            "n_outside": int(len(cam_valid) - len(uv_in)),
        }
        return uv_in, depth_in, stats

    def _scale_box_to_img(self, box, out_w, out_h):
        x1, y1, x2, y2, label, tid = box
        sx = float(out_w) / float(self.calib_w)
        sy = float(out_h) / float(self.calib_h)
        bx1 = int(round(x1 * sx))
        by1 = int(round(y1 * sy))
        bx2 = int(round(x2 * sx))
        by2 = int(round(y2 * sy))
        return bx1, by1, bx2, by2, label, tid

    def _filter_uv_by_boxes(self, uv, depth, out_w, out_h):
        if len(uv) == 0 or len(self.latest_boxes) == 0:
            return uv, depth

        margin = int(self.args.box_margin)
        uv_i = np.round(uv).astype(np.int32)
        keep = np.zeros((len(uv_i),), dtype=bool)

        for b in self.latest_boxes:
            x1, y1, x2, y2, _, _ = self._scale_box_to_img(b, out_w, out_h)
            xa, xb = min(x1, x2), max(x1, x2)
            ya, yb = min(y1, y2), max(y1, y2)
            xa = max(0, xa - margin)
            ya = max(0, ya - margin)
            xb = min(out_w - 1, xb + margin)
            yb = min(out_h - 1, yb + margin)
            inb = (
                (uv_i[:, 0] >= xa) & (uv_i[:, 0] <= xb) &
                (uv_i[:, 1] >= ya) & (uv_i[:, 1] <= yb)
            )
            keep |= inb

        return uv[keep], depth[keep]

    def _nearest_per_pixel(self, uv, depth, out_w, out_h):
        if len(uv) == 0:
            return uv, depth

        uv_i = np.round(uv).astype(np.int32)
        uv_i[:, 0] = np.clip(uv_i[:, 0], 0, out_w - 1)
        uv_i[:, 1] = np.clip(uv_i[:, 1], 0, out_h - 1)

        pix = uv_i[:, 1] * out_w + uv_i[:, 0]
        order = np.argsort(depth)  # 近到远
        pix_sorted = pix[order]
        _, first_idx = np.unique(pix_sorted, return_index=True)
        keep_order_idx = order[first_idx]

        return uv[keep_order_idx], depth[keep_order_idx]

    def _box_hit_stats_and_draw(self, img, uv, out_w, out_h):
        if not self.use_detect_box or len(self.latest_boxes) == 0:
            return {
                "n_boxes": 0,
                "n_boxes_hit": 0,
                "avg_pts_per_box": 0.0,
                "diag_lines": [],
            }

        uv_i = np.round(uv).astype(np.int32) if len(uv) > 0 else np.empty((0, 2), dtype=np.int32)

        total_hits = 0
        hit_boxes = 0
        diag_lines = []

        for b in self.latest_boxes:
            x1, y1, x2, y2, label, tid = self._scale_box_to_img(b, out_w, out_h)
            xa, xb = min(x1, x2), max(x1, x2)
            ya, yb = min(y1, y2), max(y1, y2)
            bw = max(1, xb - xa)
            bh = max(1, yb - ya)
            bcx = 0.5 * (xa + xb)
            bcy = 0.5 * (ya + yb)

            if len(uv_i) > 0:
                inside = (
                    (uv_i[:, 0] >= xa) & (uv_i[:, 0] <= xb) &
                    (uv_i[:, 1] >= ya) & (uv_i[:, 1] <= yb)
                )
                cnt = int(np.sum(inside))
                pts = uv_i[inside]
            else:
                cnt = 0
                pts = np.empty((0, 2), dtype=np.int32)

            total_hits += cnt
            if cnt > 20:
                hit_boxes += 1

            color = (0, 255, 0) if cnt > 20 else (0, 128, 255)
            cv2.rectangle(img, (xa, ya), (xb, yb), color, 1)

            if cnt > 0:
                pmin = pts.min(axis=0)
                pmax = pts.max(axis=0)
                pw = max(1, int(pmax[0] - pmin[0]))
                ph = max(1, int(pmax[1] - pmin[1]))
                pcx = 0.5 * (pmin[0] + pmax[0])
                pcy = 0.5 * (pmin[1] + pmax[1])

                w_ratio = pw / float(bw)
                h_ratio = ph / float(bh)
                dx_n = (pcx - bcx) / float(bw)
                dy_n = (pcy - bcy) / float(bh)

                # 下边缘倾斜估计：取底部30%点做线性拟合 v = k*u + b
                if cnt >= 15:
                    v_thr = np.percentile(pts[:, 1], 70)
                    bot = pts[pts[:, 1] >= v_thr]
                else:
                    bot = pts
                if len(bot) >= 8:
                    k, _ = np.polyfit(bot[:, 0].astype(np.float64), bot[:, 1].astype(np.float64), 1)
                    tilt_deg = float(np.degrees(np.arctan(k)))
                else:
                    tilt_deg = 0.0

                cv2.rectangle(img, (int(pmin[0]), int(pmin[1])), (int(pmax[0]), int(pmax[1])), (255, 255, 0), 1)
                cv2.putText(
                    img,
                    f"{label}:{tid} pts={cnt} wr={w_ratio:.2f} hr={h_ratio:.2f} dx={dx_n:+.2f} dy={dy_n:+.2f} tilt={tilt_deg:+.1f}d",
                    (xa, max(0, ya - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    color,
                    1,
                    cv2.LINE_AA,
                )
                diag_lines.append(
                    f"{label}:{tid} pts={cnt} wr={w_ratio:.2f} hr={h_ratio:.2f} dx={dx_n:+.2f} dy={dy_n:+.2f} tilt={tilt_deg:+.1f}d"
                )
            else:
                cv2.putText(img, f"{label}:{tid} pts=0", (xa, max(0, ya - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
                diag_lines.append(f"{label}:{tid} pts=0")

        return {
            "n_boxes": len(self.latest_boxes),
            "n_boxes_hit": hit_boxes,
            "avg_pts_per_box": float(total_hits / max(1, len(self.latest_boxes))),
            "diag_lines": diag_lines,
        }

    def _draw(self, img_bgr, uv, depth):
        out = img_bgr.copy()
        h, w = out.shape[:2]
        if len(uv) == 0:
            return out

        if len(uv) > self.args.max_draw_points:
            idx = np.random.choice(len(uv), self.args.max_draw_points, replace=False)
            uv = uv[idx]
            depth = depth[idx]

        d_min = max(self.args.min_depth, float(np.min(depth)))
        d_max = max(d_min + 1e-6, float(np.max(depth)))

        uv_i = np.round(uv).astype(np.int32)
        uv_i[:, 0] = np.clip(uv_i[:, 0], 0, w - 1)
        uv_i[:, 1] = np.clip(uv_i[:, 1], 0, h - 1)

        for (u, v), z in zip(uv_i, depth):
            t = (z - d_min) / (d_max - d_min + 1e-6)
            color = (int(255 * t), 0, int(255 * (1.0 - t)))  # 近红远蓝
            cv2.circle(out, (int(u), int(v)), self.args.point_radius, color, -1, lineType=cv2.LINE_AA)

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[uv_i[:, 1], uv_i[:, 0]] = 255

        k = max(1, int(self.args.silhouette_dilate))
        if k % 2 == 0:
            k += 1
        kernel = np.ones((k, k), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(out, contours, -1, (0, 255, 255), 1)

        return out

    def _draw_cloud_canvas(self, shape_hw, uv_all, uv_focus):
        """构建相机视角点云底图：全量点灰色，关注点(框内/过滤后)白色。"""
        h, w = shape_hw
        canvas = np.zeros((h, w, 3), dtype=np.uint8)

        if len(uv_all) > 0:
            uv_all_i = np.round(uv_all).astype(np.int32)
            uv_all_i[:, 0] = np.clip(uv_all_i[:, 0], 0, w - 1)
            uv_all_i[:, 1] = np.clip(uv_all_i[:, 1], 0, h - 1)
            canvas[uv_all_i[:, 1], uv_all_i[:, 0]] = (80, 80, 80)

        if len(uv_focus) > 0:
            uv_focus_i = np.round(uv_focus).astype(np.int32)
            uv_focus_i[:, 0] = np.clip(uv_focus_i[:, 0], 0, w - 1)
            uv_focus_i[:, 1] = np.clip(uv_focus_i[:, 1], 0, h - 1)
            canvas[uv_focus_i[:, 1], uv_focus_i[:, 0]] = (255, 255, 255)

        return canvas

    def _render_overlay3d(self, img_bgr):
        """overlay3d 模式：点云白点底图 + 半透明相机图像叠加，用于外参验证。"""
        h, w = img_bgr.shape[:2]

        # --- 投影全量点云 ---
        uv, depth, pstats = self._project_lidar_to_img(self.latest_points, w, h)

        # --- 构建点云底图（黑底白点）---
        cloud_layer = np.zeros((h, w, 3), dtype=np.uint8)
        if len(uv) > 0:
            uv_i = np.round(uv).astype(np.int32)
            uv_i[:, 0] = np.clip(uv_i[:, 0], 0, w - 1)
            uv_i[:, 1] = np.clip(uv_i[:, 1], 0, h - 1)
            # 白色小点（使用数组赋值，快速）
            cloud_layer[uv_i[:, 1], uv_i[:, 0]] = (255, 255, 255)

        # --- alpha 混合：cloud_layer * (1-alpha) + img_bgr * alpha ---
        alpha = self._overlay3d_alpha
        blended = cv2.addWeighted(cloud_layer, 1.0 - alpha, img_bgr, alpha, 0.0)

        # --- 在左上角标注信息 ---
        info = (
            f"overlay3d  alpha={alpha:.2f} (+/- to adjust, q to quit)  "
            f"pts_in_img={pstats['n_in_img']}/{pstats['n_front']}/{pstats['n_total']}"
        )
        cv2.putText(blended, info, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 255, 255), 1, cv2.LINE_AA)

        # --- 大图降采样显示 ---
        max_w = self.args.overlay3d_max_width
        if w > max_w:
            scale = max_w / float(w)
            disp = cv2.resize(blended, (max_w, int(h * scale)), interpolation=cv2.INTER_AREA)
        else:
            disp = blended

        return disp, blended, pstats

    def _make_fusion_view(self, img_bgr, uv_all, uv_focus):
        """相机视角融合图：点云画布 + 半透明相机图 + 检测框诊断。"""
        h, w = img_bgr.shape[:2]
        cloud_canvas = self._draw_cloud_canvas((h, w), uv_all, uv_focus)
        alpha = float(np.clip(self.args.fusion_image_alpha, 0.0, 1.0))
        fusion = cv2.addWeighted(cloud_canvas, 1.0, img_bgr, alpha, 0.0)

        # 在 fusion 图上继续画检测框和统计文字，便于对齐排查
        self._box_hit_stats_and_draw(fusion, uv_focus, w, h)
        cv2.putText(
            fusion,
            f"fusion(alpha={alpha:.2f}) gray=all projected, white=focus(box-filtered)",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return fusion

    def _img_cb(self, msg: Image):
        if self.latest_points is None:
            return

        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        # ---- overlay3d 专用分支 ----
        if self.args.render_mode == "overlay3d":
            self._img_cb_overlay3d(img)
            return

        h, w = img.shape[:2]

        img_stamp = _safe_stamp_to_sec(msg.header.stamp)
        now_t = time.time()
        img_t = img_stamp if img_stamp is not None else now_t

        if self.latest_pc_stamp is not None:
            pc_t = self.latest_pc_stamp
            dt_src = "stamp"
        elif self.latest_pc_recv_t is not None:
            pc_t = self.latest_pc_recv_t
            dt_src = "recv"
        else:
            pc_t = img_t
            dt_src = "none"

        dt = abs(img_t - pc_t)

        uv_all, depth_all, pstats = self._project_lidar_to_img(self.latest_points, w, h)

        mode = self.args.render_mode
        if mode == "auto":
            mode = "box-only" if (self.use_detect_box and len(self.latest_boxes) > 0) else "full"

        uv = uv_all
        depth = depth_all
        if mode == "box-only" and len(self.latest_boxes) > 0:
            uv, depth = self._filter_uv_by_boxes(uv, depth, w, h)

        if self.args.nearest_per_pixel:
            uv, depth = self._nearest_per_pixel(uv, depth, w, h)

        out = self._draw(img, uv, depth)
        bstats = self._box_hit_stats_and_draw(out, uv, w, h)
        fusion = self._make_fusion_view(img, uv_all, uv)

        warn = "" if dt <= self.args.sync_tol else f" [WARN sync dt={dt:.3f}s src={dt_src}]"
        line1 = (
            f"mode={mode} proj(in/front/total)={len(uv)}/{pstats['n_front']}/{pstats['n_total']} "
            f"outside={pstats['n_outside']}{warn}"
        )
        line2 = f"boxes hit/total={bstats['n_boxes_hit']}/{bstats['n_boxes']} avg_pts_per_box={bstats['avg_pts_per_box']:.1f}"
        cv2.putText(out, line1, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(out, line2, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)

        publish_img = fusion if self.args.view_mode == "fusion" else out
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(publish_img, encoding="bgr8"))
        except Exception as e:
            self.get_logger().error(f"调试图发布失败: {e}")

        if not self.args.no_window:
            if self.args.view_mode in ("overlay", "both"):
                cv2.imshow("lidar_cam_projection_debug", out)
            if self.args.view_mode in ("fusion", "both"):
                cv2.imshow("lidar_cam_projection_fusion", fusion)
            cv2.waitKey(1)

        self.frame_idx += 1
        if self.args.save_every > 0 and (self.frame_idx % self.args.save_every == 0):
            ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            save_path = os.path.join(self.args.save_dir, f"proj_{ts}_{self.frame_idx:06d}.jpg")
            cv2.imwrite(save_path, publish_img)

        now = time.time()
        if (now - self.last_log_t) >= self.args.log_interval:
            in_front = max(1, pstats["n_front"])
            in_img_ratio = pstats["n_in_img"] / in_front
            outside_ratio = pstats["n_outside"] / in_front
            log_msg = (
                f"模式={mode}, 原始投影率={in_img_ratio:.3f}, 原始溢出率={outside_ratio:.3f}, "
                f"框命中={bstats['n_boxes_hit']}/{bstats['n_boxes']}, 框均值点数={bstats['avg_pts_per_box']:.1f}, "
                f"dt={dt:.3f}s({dt_src})"
            )
            if bstats.get("diag_lines"):
                log_msg += " | " + " ; ".join(bstats["diag_lines"][:2])
            self.get_logger().info(log_msg)
            self.last_log_t = now

    def _img_cb_overlay3d(self, img_bgr):
        """overlay3d 模式的图像回调：点云白点 + 半透明相机图叠加，按键交互。"""
        disp, blended, pstats = self._render_overlay3d(img_bgr)

        # 发布完整分辨率的混合图
        try:
            self.pub_dbg.publish(self.bridge.cv2_to_imgmsg(blended, encoding="bgr8"))
        except Exception as e:
            self.get_logger().error(f"调试图发布失败: {e}")

        # 显示窗口
        win_title = f"overlay3d (alpha={self._overlay3d_alpha:.2f})"
        if not self.args.no_window:
            cv2.imshow(win_title, disp)
            key = cv2.waitKey(1) & 0xFF
            # 按键交互
            if key == ord("q"):
                self.get_logger().info("overlay3d: 用户按 q 退出")
                raise KeyboardInterrupt
            elif key in (ord("+"), ord("=")):
                self._overlay3d_alpha = min(1.0, self._overlay3d_alpha + 0.05)
                self.get_logger().info(f"overlay3d: alpha -> {self._overlay3d_alpha:.2f}")
                # 更新窗口标题（通过关闭旧窗口、下帧自动重建）
                cv2.destroyAllWindows()
            elif key in (ord("-"), ord("_")):
                self._overlay3d_alpha = max(0.0, self._overlay3d_alpha - 0.05)
                self.get_logger().info(f"overlay3d: alpha -> {self._overlay3d_alpha:.2f}")
                cv2.destroyAllWindows()

        # 保存
        self.frame_idx += 1
        if self.args.save_every > 0 and (self.frame_idx % self.args.save_every == 0):
            ts = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            save_path = os.path.join(self.args.save_dir, f"overlay3d_{ts}_{self.frame_idx:06d}.jpg")
            cv2.imwrite(save_path, blended)

        # 日志
        now = time.time()
        if (now - self.last_log_t) >= self.args.log_interval:
            self.get_logger().info(
                f"overlay3d: alpha={self._overlay3d_alpha:.2f}, "
                f"pts_in_img={pstats['n_in_img']}/{pstats['n_front']}/{pstats['n_total']}"
            )
            self.last_log_t = now


def main():
    parser = _build_parser()
    args = parser.parse_args()

    rclpy.init()
    node = LidarCamProjectionDebugger(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
        if not args.no_window:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()

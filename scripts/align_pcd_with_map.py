#!/usr/bin/env python3
"""
align_pcd_with_map.py — 交互式点云-地图对齐工具（纯键盘控制版）

原理：
  将PCD点云俯视图和地图叠加，通过键盘移动/旋转/缩放地图使其与点云对齐。
  对齐后计算变换矩阵，将PCD从雷达坐标系变换到世界坐标系。

  变换逻辑：
    1. 对齐确定雷达在世界坐标系中的位置 (tx, ty) 和朝向 yaw
    2. 对每个PCD点: new_point = R(yaw) @ old_point + [tx, ty, height]
    3. 变换后地图左下角 = 世界原点(0,0)

键盘操作：
  移动地图:  ←→↑↓ (粗调5px)  |  H/J/K/L (精调1px)
  旋转地图:  A/D (1°)  |  W/X (0.1°)
  缩放地图:  +/= (放大5%)  |  -/_ (缩小5%)  |  [/] (精调1%)
  显示:      G (网格)  |  T (变换信息)  |  O (透明度切换)
  操作:      S/Enter (确认)  |  R (重置)  |  Q/Esc (退出)

使用:
  python3 scripts/align_pcd_with_map.py [--height 1.05]
"""

import os
import sys
import json
import argparse
import shutil
import numpy as np
import cv2

try:
    import open3d as o3d
except ImportError:
    print("错误: 需要 open3d, pip install open3d")
    sys.exit(1)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def render_pcd_topview(pcd_path, resolution=100, margin=60):
    """将PCD渲染为俯视图"""
    print(f"加载点云: {pcd_path}")
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)
    print(f"  点数={len(points)}, X=[{points[:,0].min():.2f},{points[:,0].max():.2f}], "
          f"Y=[{points[:,1].min():.2f},{points[:,1].max():.2f}], Z=[{points[:,2].min():.2f},{points[:,2].max():.2f}]")

    x_min, x_max = points[:, 0].min(), points[:, 0].max()
    y_min, y_max = points[:, 1].min(), points[:, 1].max()

    w = int((x_max - x_min) * resolution) + 2 * margin
    h = int((y_max - y_min) * resolution) + 2 * margin

    img = np.zeros((h, w, 3), dtype=np.uint8)

    px = ((points[:, 0] - x_min) * resolution + margin).astype(np.int32)
    py = (h - ((points[:, 1] - y_min) * resolution + margin)).astype(np.int32)
    mask = (px >= 0) & (px < w) & (py >= 0) & (py < h)
    px, py = px[mask], py[mask]
    z_vals = points[mask, 2]

    z_min, z_max = z_vals.min(), z_vals.max()
    z_norm = np.clip((z_vals - z_min) / max(z_max - z_min, 0.01), 0, 1)
    colors = (80 + 175 * z_norm).astype(np.uint8)
    img[py, px, 0] = colors
    img[py, px, 1] = colors

    # 标记PCD原点
    opx = int((0 - x_min) * resolution) + margin
    opy = h - (int((0 - y_min) * resolution) + margin)
    if 0 <= opx < w and 0 <= opy < h:
        cv2.drawMarker(img, (opx, opy), (0, 0, 255), cv2.MARKER_CROSS, 25, 2)
        cv2.putText(img, "Radar(0,0)", (opx + 12, opy - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    return img, x_min, y_min, resolution, margin, w, h


def put_text_bg(img, text, pos, font_scale=0.65, color=(255, 255, 255),
                bg_color=(0, 0, 0), thickness=2):
    """带背景的文字绘制，提高可读性"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), baseline = cv2.getTextSize(text, font, font_scale, thickness)
    x, y = pos
    cv2.rectangle(img, (x - 2, y - th - 4), (x + tw + 2, y + baseline + 2), bg_color, -1)
    cv2.putText(img, text, (x, y), font, font_scale, color, thickness)


class MapAligner:
    def __init__(self, pcd_img, map_img, x_min, y_min, res, margin, pw, ph, radar_h):
        self.pcd_img = pcd_img
        self.map_orig = map_img
        self.x_min, self.y_min = x_min, y_min
        self.res, self.margin = res, margin
        self.pw, self.ph = pw, ph
        self.radar_h = radar_h

        # 地图状态
        self.cx = float(pw // 2)
        self.cy = float(ph // 2)
        self.angle = 0.0
        self.scale = 1.0

        # 地图世界尺寸
        self.map_world_w = 28.0
        self.map_world_h = 15.0
        mh, mw = map_img.shape[:2]
        self.base_scale = res / (mw / self.map_world_w)

        # 显示选项
        self.show_grid = True
        self.show_info = True
        self.alpha = 0.45

    def _map_point_to_canvas(self, lx_frac, ly_frac):
        """地图归一化坐标(0~1) → 画布像素"""
        mh, mw = self.map_orig.shape[:2]
        s = self.base_scale * self.scale
        nw, nh = mw * s, mh * s

        px_local = lx_frac * nw - nw / 2
        py_local = ly_frac * nh - nh / 2

        rad = np.radians(self.angle)
        c, sn = np.cos(rad), np.sin(rad)
        rx = px_local * c + py_local * sn
        ry = -px_local * sn + py_local * c
        return self.cx + rx, self.cy + ry

    def get_origin_pixel(self):
        """地图原点(0,0)=左下角 在画布中的像素位置"""
        return self._map_point_to_canvas(0.0, 1.0)  # 左下

    def get_corners(self):
        """四角像素位置 + 标签"""
        corners = [
            (0.0, 1.0, "(0,0)"),    # 左下
            (1.0, 1.0, "(28,0)"),   # 右下
            (1.0, 0.0, "(28,15)"),  # 右上
            (0.0, 0.0, "(0,15)"),   # 左上
        ]
        return [(self._map_point_to_canvas(fx, fy) + (lb,)) for fx, fy, lb in corners]

    def compute_transform(self):
        """计算雷达在世界坐标系中的位置"""
        ox, oy = self.get_origin_pixel()

        # PCD原点(0,0)在画布中的真实像素位置（和render_pcd_topview一致）
        pcd_ox = (0 - self.x_min) * self.res + self.margin
        pcd_oy = self.ph - ((0 - self.y_min) * self.res + self.margin)

        # 地图原点相对PCD原点的偏移（像素→米）
        dx_m = (ox - pcd_ox) / self.res
        dy_m = -(oy - pcd_oy) / self.res  # Y翻转（画布Y向下，世界Y向上）

        # 雷达在世界坐标中的位置 = -offset（因为要把PCD原点移到地图原点）
        return {
            "radar_x": -dx_m,
            "radar_y": -dy_m,
            "yaw_deg": self.angle,
        }

    def render_map_layer(self):
        """渲染变换后的地图层"""
        mh, mw = self.map_orig.shape[:2]
        s = self.base_scale * self.scale
        nw, nh = int(mw * s), int(mh * s)
        if nw < 5 or nh < 5:
            return np.zeros_like(self.pcd_img)

        scaled = cv2.resize(self.map_orig, (nw, nh))
        M = cv2.getRotationMatrix2D((nw / 2, nh / 2), self.angle, 1.0)
        ca, sa = abs(np.cos(np.radians(self.angle))), abs(np.sin(np.radians(self.angle)))
        rw, rh = int(nw * ca + nh * sa), int(nh * ca + nw * sa)
        M[0, 2] += (rw - nw) / 2
        M[1, 2] += (rh - nh) / 2
        rotated = cv2.warpAffine(scaled, M, (rw, rh))

        canvas = np.zeros_like(self.pcd_img)
        xo, yo = int(self.cx) - rw // 2, int(self.cy) - rh // 2
        sx1, sy1 = max(0, -xo), max(0, -yo)
        sx2, sy2 = min(rw, self.pw - xo), min(rh, self.ph - yo)
        dx1, dy1 = max(0, xo), max(0, yo)
        if sx2 > sx1 and sy2 > sy1:
            canvas[dy1:dy1+(sy2-sy1), dx1:dx1+(sx2-sx1)] = rotated[sy1:sy2, sx1:sx2]
        return canvas

    def draw_grid(self, img):
        """绘制世界坐标网格"""
        mh, mw = self.map_orig.shape[:2]
        s = self.base_scale * self.scale
        nw, nh = mw * s, mh * s
        rad = np.radians(self.angle)
        c, sn = np.cos(rad), np.sin(rad)

        def w2p(mx, my):
            lx = mx / self.map_world_w * nw - nw / 2
            ly = (1.0 - my / self.map_world_h) * nh - nh / 2
            rx = lx * c + ly * sn
            ry = -lx * sn + ly * c
            return int(self.cx + rx), int(self.cy + ry)

        # 边界
        pts = np.array([w2p(0, 0), w2p(28, 0), w2p(28, 15), w2p(0, 15)], np.int32)
        cv2.polylines(img, [pts], True, (0, 220, 0), 2)

        for x in range(0, 29, 5):
            cv2.line(img, w2p(x, 0), w2p(x, 15), (0, 100, 0), 1)
            put_text_bg(img, f"{x}m", w2p(x, 0), 0.5, (0, 220, 0))
        for y in range(0, 16, 5):
            cv2.line(img, w2p(0, y), w2p(28, y), (0, 100, 0), 1)
            put_text_bg(img, f"{y}m", w2p(0, y), 0.5, (0, 220, 0))

    def render(self):
        """渲染完整画面"""
        ml = self.render_map_layer()
        blended = cv2.addWeighted(self.pcd_img, 1.0, ml, self.alpha, 0)

        if self.show_grid:
            self.draw_grid(blended)

        # 地图中心
        cv2.drawMarker(blended, (int(self.cx), int(self.cy)), (0, 255, 0),
                       cv2.MARKER_CROSS, 30, 2)

        # 地图原点
        ox, oy = self.get_origin_pixel()
        cv2.drawMarker(blended, (int(ox), int(oy)), (0, 255, 255),
                       cv2.MARKER_DIAMOND, 18, 2)
        put_text_bg(blended, "Map(0,0)", (int(ox) + 12, int(oy) - 8), 0.65, (0, 255, 255))

        # 四角
        for px, py, lb in self.get_corners():
            cv2.circle(blended, (int(px), int(py)), 5, (0, 200, 200), -1)
            put_text_bg(blended, lb, (int(px) + 6, int(py) - 6), 0.5, (0, 200, 200))

        # 信息面板
        y = 25
        lh = 28
        hints = [
            "Arrows:move(5px) H/J/K/L:move(1px)",
            "A/D:rot(1deg) W/X:rot(0.1deg)",
            "+/-:zoom(5%) [/]:zoom(1%)",
            "G:grid T:info O:alpha R:reset",
            "S/Enter:confirm  Q/Esc:quit",
        ]
        for h in hints:
            put_text_bg(blended, h, (10, y), 0.6, (255, 255, 255))
            y += lh

        if self.show_info:
            y += 8
            t = self.compute_transform()
            if t:
                rx, ry, yaw = t["radar_x"], t["radar_y"], t["yaw_deg"]
                lines = [
                    f"Angle: {self.angle:.1f}deg  Scale: {self.scale:.3f}x  Alpha: {self.alpha:.2f}",
                    f"Radar world pos: ({rx:.2f}, {ry:.2f}, {self.radar_h:.2f})m",
                    f"Yaw: {yaw:.1f}deg",
                ]

                # 预期变换后范围
                yr = np.radians(yaw)
                R = np.array([[np.cos(yr), -np.sin(yr)], [np.sin(yr), np.cos(yr)]])
                tv = np.array([rx, ry])
                x_ext = (self.pw - 2 * self.margin) / self.res
                y_ext = (self.ph - 2 * self.margin) / self.res
                corners = np.array([
                    [self.x_min, self.y_min],
                    [self.x_min + x_ext, self.y_min],
                    [self.x_min + x_ext, self.y_min + y_ext],
                    [self.x_min, self.y_min + y_ext],
                ])
                tf = (R @ corners.T).T + tv
                lines.append(f"Expected X:[{tf[:,0].min():.1f},{tf[:,0].max():.1f}] "
                             f"Y:[{tf[:,1].min():.1f},{tf[:,1].max():.1f}]")
                lines.append("Target: X[0,28] Y[0,15]")

                for line in lines:
                    put_text_bg(blended, line, (10, y), 0.6, (0, 255, 128))
                    y += lh

        # 缩放到显示尺寸
        screen_w, screen_h = 1600, 900
        ds = min(screen_w / self.pw, screen_h / self.ph, 1.0)
        if ds < 1.0:
            disp = cv2.resize(blended, (int(self.pw * ds), int(self.ph * ds)))
        else:
            disp = blended
        return disp

    def run(self):
        wn = "Align PCD with Map (keyboard only)"
        cv2.namedWindow(wn, cv2.WINDOW_NORMAL | cv2.WINDOW_GUI_NORMAL)
        # 尽量大的窗口
        screen_w, screen_h = 1600, 900
        ds = min(screen_w / self.pw, screen_h / self.ph, 1.0)
        win_w, win_h = int(self.pw * ds), int(self.ph * ds)
        cv2.resizeWindow(wn, win_w, win_h)

        print("\n交互式对齐已启动（纯键盘控制）:")
        print("  ←→↑↓ = 移动地图(5px)  |  H/J/K/L = 精细移动(1px)")
        print("  A/D = 旋转(1°)  |  W/X = 精细旋转(0.1°)")
        print("  +/= = 放大(5%)  |  -/_ = 缩小(5%)  |  [/] = 精细缩放(1%)")
        print("  G = 网格  |  T = 信息  |  O = 透明度  |  R = 重置")
        print("  S/Enter = 确认  |  Q/Esc = 退出")

        while True:
            cv2.imshow(wn, self.render())
            key = cv2.waitKey(50) & 0xFF

            if key == ord('q') or key == 27:
                cv2.destroyAllWindows()
                return None
            elif key == ord('s') or key == 13:
                result = self.compute_transform()
                cv2.destroyAllWindows()
                return result
            elif key == ord('r'):
                self.cx, self.cy = float(self.pw // 2), float(self.ph // 2)
                self.angle, self.scale = 0.0, 1.0
            # 移动 - 粗调
            elif key == 81 or key == 2:   self.cx -= 5  # ←
            elif key == 83 or key == 3:   self.cx += 5  # →
            elif key == 82 or key == 0:   self.cy -= 5  # ↑
            elif key == 84 or key == 1:   self.cy += 5  # ↓
            # 移动 - 精调 (vim风格)
            elif key == ord('h'):  self.cx -= 1
            elif key == ord('l'):  self.cx += 1
            elif key == ord('k'):  self.cy -= 1
            elif key == ord('j'):  self.cy += 1
            # 旋转
            elif key == ord('a'):  self.angle += 1.0
            elif key == ord('d'):  self.angle -= 1.0
            elif key == ord('w'):  self.angle += 0.1
            elif key == ord('x'):  self.angle -= 0.1
            # 缩放 - 粗调
            elif key == ord('+') or key == ord('='):  self.scale *= 1.05
            elif key == ord('-') or key == ord('_'):  self.scale /= 1.05
            # 缩放 - 精调
            elif key == ord(']'):  self.scale *= 1.01
            elif key == ord('['):  self.scale /= 1.01
            # 显示
            elif key == ord('g'):  self.show_grid = not self.show_grid
            elif key == ord('t'):  self.show_info = not self.show_info
            elif key == ord('o'):  self.alpha = 0.7 if self.alpha < 0.5 else 0.3


def apply_transform(pcd_path, radar_x, radar_y, yaw_deg, radar_h, backup=True):
    """变换PCD: new = R(yaw) @ old + [radar_x, radar_y, radar_h]"""
    if backup:
        bak = pcd_path + ".bak"
        if not os.path.exists(bak):
            shutil.copy2(pcd_path, bak)
            print(f"  备份: {bak}")

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)

    yr = np.radians(yaw_deg)
    c, s = np.cos(yr), np.sin(yr)
    nx = pts[:, 0] * c - pts[:, 1] * s + radar_x
    ny = pts[:, 0] * s + pts[:, 1] * c + radar_y
    nz = pts[:, 2] + radar_h

    pts[:, 0], pts[:, 1], pts[:, 2] = nx, ny, nz
    pcd.points = o3d.utility.Vector3dVector(pts)
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"  变换完成: {pcd_path}")
    print(f"    X=[{nx.min():.2f},{nx.max():.2f}] Y=[{ny.min():.2f},{ny.max():.2f}] Z=[{nz.min():.2f},{nz.max():.2f}]")


def main():
    parser = argparse.ArgumentParser(description="交互式点云-地图对齐工具")
    parser.add_argument("--pcd", default=None)
    parser.add_argument("--map", default=None)
    parser.add_argument("--height", type=float, default=None)
    parser.add_argument("--resolution", type=int, default=80)
    args = parser.parse_args()

    config_path = os.path.join(PROJECT_ROOT, "configs", "main_config.yaml")
    pcd_path, map_path, down_path = args.pcd, args.map, None

    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        with open(config_path, encoding="utf-8") as f:
            cfg = yaml.load(f)
        scene = cfg.get("global", {}).get("scene", "competition")
        sc = cfg.get("scenes", {}).get(scene, {})
        if pcd_path is None:
            pcd_path = os.path.join(PROJECT_ROOT, sc.get("pcd_file", "data/pcds.pcd"))
        if map_path is None:
            map_path = os.path.join(PROJECT_ROOT, sc.get("std_map", "map/std_map.png"))
        dr = sc.get("downsampled_pcd")
        down_path = os.path.join(PROJECT_ROOT, dr) if dr else None
    except Exception as e:
        print(f"配置读取失败: {e}")
        if not pcd_path: pcd_path = os.path.join(PROJECT_ROOT, "data/lab_pcds.pcd")
        if not map_path: map_path = os.path.join(PROJECT_ROOT, "map/lab_map_28x15.png")

    for p, n in [(pcd_path, "PCD"), (map_path, "地图")]:
        if not os.path.exists(p):
            print(f"错误: {n}不存在: {p}"); sys.exit(1)

    radar_h = args.height
    if radar_h is None:
        try:
            v = input("雷达离地高度(m) [1.05]: ").strip()
            radar_h = float(v) if v else 1.05
        except ValueError:
            radar_h = 1.05
    print(f"雷达高度: {radar_h}m")

    pcd_img, xm, ym, res, margin, w, h = render_pcd_topview(pcd_path, args.resolution)
    map_img = cv2.imread(map_path)
    if map_img is None:
        print(f"错误: 无法加载地图: {map_path}"); sys.exit(1)

    aligner = MapAligner(pcd_img, map_img, xm, ym, res, margin, w, h, radar_h)
    result = aligner.run()
    if result is None:
        print("已取消。"); return

    rx, ry, yaw = result["radar_x"], result["radar_y"], result["yaw_deg"]
    print(f"\n对齐结果: 雷达位置=({rx:.3f}, {ry:.3f}, {radar_h:.3f})m, 旋转={yaw:.2f}°")

    if input("\n确认变换? [y/N]: ").strip().lower() != 'y':
        print("已取消。"); return

    print("\n变换PCD...")
    apply_transform(pcd_path, rx, ry, yaw, radar_h)
    if down_path and os.path.exists(down_path):
        apply_transform(down_path, rx, ry, yaw, radar_h)

    info = {
        "radar_world_position": [round(rx, 3), round(ry, 3), radar_h],
        "yaw_deg": round(yaw, 2),
        "method": "interactive_visual_alignment",
        "description": "PCD已变换到世界坐标系，地图左下角=(0,0,0)",
        "transformed_files": [pcd_path] + ([down_path] if down_path and os.path.exists(down_path) else []),
        "note": "恢复原始PCD用 .bak 文件",
    }
    info_path = os.path.join(PROJECT_ROOT, "configs", "lab_pcd_transform_info.json")
    with open(info_path, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=2, ensure_ascii=False)
    print(f"变换信息已保存: {info_path}")


if __name__ == "__main__":
    main()

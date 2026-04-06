#!/usr/bin/env python3
"""
calibrate_lab_map.py — 交互式实验室地图标定对齐工具

功能：
  通过在旧地图上点击已知物理位置，并输入对应的 RViz 世界坐标，
  计算仿射变换（平移 + 旋转 + 缩放），将旧地图挪到正确位置，
  生成新的 display_panel 底图。

使用方法：
  python3 scripts/calibrate_lab_map.py

操作步骤：
  1. 在弹出的旧地图窗口中，左键点击一个你能辨认的物理位置
  2. 在终端输入该位置在 RViz 中读到的世界坐标 (X Y)
  3. 重复以上步骤，至少标定 2 个点（推荐 3-5 个）
  4. 按 [p] 预览变换效果（带网格线）
  5. 满意后按 [w] 保存新地图

快捷键：
  左键点击  - 在旧地图上标记一个标定点
  [p]       - 预览变换效果
  [w]       - 保存新地图（自动备份旧地图）
  [u]       - 撤销最后一个标定点
  [r]       - 重置所有标定点
  [q/ESC]   - 退出
"""

import cv2
import numpy as np
import json
import os
import sys
import shutil
import time
from pathlib import Path

# ============ 路径配置 ============
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(PROJECT_ROOT, "configs", "main_config.yaml")

# 输出尺寸（与 display_panel 一致）
CANVAS_W = 2800   # 像素
CANVAS_H = 1500   # 像素
FIELD_W  = 28.0   # 米
FIELD_H  = 15.0   # 米
SCALE    = 100.0   # px/m

# 标定点持久化文件
CALIB_POINTS_PATH = os.path.join(PROJECT_ROOT, "configs", "calibration", "lab_map_calib_points.json")


def load_config():
    """加载 main_config.yaml 获取当前场景的地图路径"""
    try:
        import yaml
        with open(CONFIG_PATH, 'r') as f:
            cfg = yaml.safe_load(f)
        scene = cfg.get('global', {}).get('scene', 'lab')
        scenes = cfg.get('scenes', {})
        scene_cfg = scenes.get(scene, scenes.get('lab', {}))
        std_map_rel = scene_cfg.get('std_map', 'data/maps/lab/lab_map_28x15.png')
        return {
            'scene': scene,
            'std_map': os.path.join(PROJECT_ROOT, std_map_rel),
            'field_width': scene_cfg.get('field_width', FIELD_W),
            'field_height': scene_cfg.get('field_height', FIELD_H),
        }
    except Exception as e:
        print(f"[WARN] 无法加载配置: {e}")
        return {
            'scene': 'lab',
            'std_map': os.path.join(PROJECT_ROOT, 'data', 'maps', 'lab', 'lab_map_28x15.png'),
            'field_width': FIELD_W,
            'field_height': FIELD_H,
        }


def world_to_canvas(wx, wy):
    """世界坐标 → 画布像素（与 display_panel 一致）"""
    px = int(round(wx * SCALE))
    py = int(round(CANVAS_H - wy * SCALE))
    return px, py


def canvas_to_world(px, py):
    """画布像素 → 世界坐标"""
    wx = px / SCALE
    wy = (CANVAS_H - py) / SCALE
    return wx, wy


class MapCalibrator:
    """交互式地图标定器"""

    def __init__(self, map_path):
        self.map_path = map_path
        self.old_map = cv2.imread(map_path)
        if self.old_map is None:
            raise FileNotFoundError(f"无法读取地图: {map_path}")

        self.old_h, self.old_w = self.old_map.shape[:2]
        print(f"旧地图尺寸: {self.old_w}×{self.old_h} 像素")

        # 标定点列表: [(old_px, old_py, world_x, world_y), ...]
        self.calib_points = []

        # 当前点击的像素位置（等待输入世界坐标）
        self.pending_click = None

        # 显示缩放（旧地图可能很大，缩放显示）
        self.display_scale = min(1.0, 1200.0 / self.old_w, 800.0 / self.old_h)

        # 仿射变换矩阵
        self.transform_matrix = None

        # 加载持久化的标定点
        self._load_calib_points()

    def _load_calib_points(self):
        """从 JSON 加载已保存的标定点"""
        if os.path.exists(CALIB_POINTS_PATH):
            try:
                with open(CALIB_POINTS_PATH, 'r') as f:
                    data = json.load(f)
                self.calib_points = [tuple(p) for p in data.get('points', [])]
                if self.calib_points:
                    print(f"[INFO] 已加载 {len(self.calib_points)} 个历史标定点")
                    for i, (ox, oy, wx, wy) in enumerate(self.calib_points):
                        print(f"  #{i+1}: 旧图({ox:.0f}, {oy:.0f}) → 世界({wx:.2f}, {wy:.2f})")
            except Exception as e:
                print(f"[WARN] 加载标定点失败: {e}")

    def _save_calib_points(self):
        """将标定点保存到 JSON"""
        data = {
            'map_path': self.map_path,
            'points': [list(p) for p in self.calib_points],
            'saved_time': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        with open(CALIB_POINTS_PATH, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)

    def mouse_callback(self, event, x, y, flags, param):
        """OpenCV 鼠标回调"""
        if event == cv2.EVENT_LBUTTONDOWN:
            # 如果正在等待上一个点的世界坐标输入，忽略新的点击
            if self.pending_click is not None:
                print(f"[忽略] 请先输入上一个点的世界坐标再点击新位置")
                return
            # 将显示坐标转换回原图坐标
            real_x = int(x / self.display_scale)
            real_y = int(y / self.display_scale)
            self.pending_click = (real_x, real_y)
            print(f"\n[点击] 旧地图像素: ({real_x}, {real_y})")
            print("请在终端输入该点对应的 RViz 世界坐标 (格式: X Y):")

    def draw_old_map(self):
        """绘制带标定点标记的旧地图"""
        display = self.old_map.copy()

        # 画已标定的点
        for i, (ox, oy, wx, wy) in enumerate(self.calib_points):
            cv2.circle(display, (int(ox), int(oy)), 12, (0, 255, 0), 3)
            cv2.circle(display, (int(ox), int(oy)), 4, (0, 255, 0), -1)
            label = f"#{i+1} ({wx:.1f},{wy:.1f})"
            cv2.putText(display, label, (int(ox) + 15, int(oy) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 画等待确认的点击
        if self.pending_click:
            px, py = self.pending_click
            cv2.circle(display, (px, py), 15, (0, 0, 255), 3)
            cv2.drawMarker(display, (px, py), (0, 0, 255),
                           cv2.MARKER_CROSS, 25, 2)
            cv2.putText(display, "等待输入世界坐标...", (px + 20, py + 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 状态栏
        status = f"标定点: {len(self.calib_points)}个 | 点击地图添加 | [p]预览 [w]保存 [u]撤销 [r]重置 [q]退出"
        h = display.shape[0]
        cv2.rectangle(display, (0, h - 40), (display.shape[1], h), (0, 0, 0), -1)
        cv2.putText(display, status, (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        # 缩放显示
        if self.display_scale < 1.0:
            dw = int(display.shape[1] * self.display_scale)
            dh = int(display.shape[0] * self.display_scale)
            display = cv2.resize(display, (dw, dh))

        return display

    def compute_transform(self):
        """计算仿射变换矩阵"""
        if len(self.calib_points) < 2:
            print("[ERROR] 至少需要 2 个标定点！")
            return None

        # 源点（旧地图像素）
        src_pts = np.array([(ox, oy) for ox, oy, _, _ in self.calib_points],
                           dtype=np.float32)
        # 目标点（新画布像素，由世界坐标计算）
        dst_pts = np.array([world_to_canvas(wx, wy) for _, _, wx, wy in self.calib_points],
                           dtype=np.float32)

        if len(self.calib_points) == 2:
            # 2 个点：相似变换（平移 + 旋转 + 等比缩放，4 DOF）
            M, inliers = cv2.estimateAffinePartial2D(src_pts, dst_pts)
        else:
            # 3+ 个点：完整仿射变换（平移 + 旋转 + 缩放 + 剪切，6 DOF）
            M, inliers = cv2.estimateAffine2D(src_pts, dst_pts)

        if M is None:
            print("[ERROR] 无法计算变换矩阵，请检查标定点是否合理")
            return None

        self.transform_matrix = M

        # 分解变换参数
        sx = np.sqrt(M[0, 0] ** 2 + M[1, 0] ** 2)
        sy = np.sqrt(M[0, 1] ** 2 + M[1, 1] ** 2)
        theta = np.degrees(np.arctan2(M[1, 0], M[0, 0]))
        tx, ty = M[0, 2], M[1, 2]

        print(f"\n{'='*50}")
        print(f"变换参数:")
        print(f"  平移:   tx={tx:.1f}px ({tx/SCALE:.2f}m),  ty={ty:.1f}px ({ty/SCALE:.2f}m)")
        print(f"  旋转:   {theta:.2f}°")
        print(f"  缩放:   sx={sx:.4f},  sy={sy:.4f}")
        print(f"{'='*50}")

        # 计算每个标定点的残差
        if len(self.calib_points) >= 2:
            print("\n标定点残差:")
            for i, (ox, oy, wx, wy) in enumerate(self.calib_points):
                # 变换后的位置
                src = np.array([ox, oy, 1.0])
                transformed = M @ src
                # 期望位置
                exp_px, exp_py = world_to_canvas(wx, wy)
                err_px = np.sqrt((transformed[0] - exp_px) ** 2 + (transformed[1] - exp_py) ** 2)
                err_m = err_px / SCALE
                print(f"  #{i+1}: 残差 = {err_px:.1f}px ({err_m:.3f}m)")

        return M

    def generate_preview(self):
        """生成变换预览图"""
        if self.transform_matrix is None:
            M = self.compute_transform()
            if M is None:
                return None
        else:
            M = self.transform_matrix

        # 创建黑色画布
        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)

        # 将旧地图通过仿射变换贴到画布上
        warped = cv2.warpAffine(self.old_map, M, (CANVAS_W, CANVAS_H),
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=(0, 0, 0))

        # 合成：warped 非黑色区域覆盖 canvas
        mask = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY) > 35
        canvas[mask] = warped[mask]

        # 画网格线
        grid_color = (60, 60, 60)
        text_color = (0, 200, 0)

        for x_m in range(int(FIELD_W) + 1):
            px = int(x_m * SCALE)
            thickness = 2 if x_m % 5 == 0 else 1
            cv2.line(canvas, (px, 0), (px, CANVAS_H), grid_color, thickness)
            if x_m % 5 == 0:
                cv2.putText(canvas, str(x_m), (px + 5, CANVAS_H - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        for y_m in range(int(FIELD_H) + 1):
            py = int(CANVAS_H - y_m * SCALE)
            thickness = 2 if y_m % 5 == 0 else 1
            cv2.line(canvas, (0, py), (CANVAS_W, py), grid_color, thickness)
            if y_m % 5 == 0:
                cv2.putText(canvas, str(y_m), (5, py - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

        # 标注原点
        cv2.circle(canvas, (0, CANVAS_H - 1), 10, (0, 0, 255), -1)
        cv2.putText(canvas, "O(0,0)", (10, CANVAS_H - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 标注标定点在新画布上的位置
        for i, (_, _, wx, wy) in enumerate(self.calib_points):
            cpx, cpy = world_to_canvas(wx, wy)
            cv2.circle(canvas, (cpx, cpy), 10, (0, 255, 255), -1)
            cv2.putText(canvas, f"#{i+1}({wx:.1f},{wy:.1f})",
                        (cpx + 12, cpy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        return canvas

    def save_new_map(self, output_path=None):
        """保存新地图"""
        if self.transform_matrix is None:
            M = self.compute_transform()
            if M is None:
                return False
        else:
            M = self.transform_matrix

        if output_path is None:
            output_path = self.map_path

        # 备份旧地图
        if os.path.exists(output_path):
            backup_dir = os.path.join(os.path.dirname(output_path), "backup")
            os.makedirs(backup_dir, exist_ok=True)
            basename = os.path.basename(output_path)
            name, ext = os.path.splitext(basename)
            backup_name = f"{name}_{int(time.time())}{ext}"
            backup_path = os.path.join(backup_dir, backup_name)
            shutil.copy2(output_path, backup_path)
            print(f"[备份] {backup_path}")

        # 生成新地图（无网格）
        canvas = np.zeros((CANVAS_H, CANVAS_W, 3), dtype=np.uint8)
        warped = cv2.warpAffine(self.old_map, M, (CANVAS_W, CANVAS_H),
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=(0, 0, 0))
        mask = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY) > 35
        canvas[mask] = warped[mask]

        cv2.imwrite(output_path, canvas)
        print(f"[保存] 新地图已保存: {output_path}")

        # 同时生成带网格的版本
        grid_path = output_path.replace('.png', '_grid.png')
        grid_canvas = self.generate_preview()
        if grid_canvas is not None:
            cv2.imwrite(grid_path, grid_canvas)
            print(f"[保存] 网格版本: {grid_path}")

        # 保存标定信息
        self._save_calib_points()

        # 保存变换矩阵信息
        calib_info = {
            'description': '实验室地图标定对齐信息',
            'transform_matrix': M.tolist(),
            'calib_points': [
                {
                    'old_pixel': [float(ox), float(oy)],
                    'world_coord': [float(wx), float(wy)],
                    'canvas_pixel': list(world_to_canvas(wx, wy))
                }
                for ox, oy, wx, wy in self.calib_points
            ],
            'output_map': output_path,
            'created_time': time.strftime("%Y-%m-%d %H:%M:%S")
        }
        info_path = os.path.join(PROJECT_ROOT, "configs", "calibration", "lab_map_alignment_info.json")
        with open(info_path, 'w') as f:
            json.dump(calib_info, f, indent=2, ensure_ascii=False)
        print(f"[保存] 标定信息: {info_path}")

        return True

    def run(self):
        """主交互循环"""
        win_name = "Lab Map Calibrator (click to add points)"
        cv2.namedWindow(win_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(win_name, self.mouse_callback)

        preview_win = "Preview (press [w] to save)"
        preview_shown = False

        print("\n" + "=" * 60)
        print("       交互式实验室地图标定对齐工具")
        print("=" * 60)
        print("操作说明:")
        print("  1. 在地图窗口中左键点击一个你认识的位置")
        print("  2. 在终端输入该位置的 RViz 世界坐标 (X Y)")
        print("  3. 至少标定 2 个点后按 [p] 预览")
        print("  4. 满意后按 [w] 保存新地图")
        print("=" * 60)

        while True:
            # 显示旧地图
            display = self.draw_old_map()
            cv2.imshow(win_name, display)

            key = cv2.waitKey(50) & 0xFF

            # 处理终端输入（非阻塞检查 pending_click）
            if self.pending_click is not None:
                try:
                    # 使用 select 模拟非阻塞输入（或直接用 input）
                    coord_str = input("  >>> 世界坐标 (X Y): ").strip()
                    if coord_str:
                        # 清理输入：移除括号并替换逗号
                        clean_str = coord_str.replace('(', '').replace(')', '').replace('[', '').replace(']', '').replace(',', ' ')
                        parts = clean_str.split()
                        if len(parts) < 2:
                            print(f"  [ERROR] 输入不足 2 个数字，请重新点击并输入")
                            self.pending_click = None
                            continue
                        wx, wy = float(parts[0]), float(parts[1])
                        ox, oy = self.pending_click
                        self.calib_points.append((ox, oy, wx, wy))
                        self._save_calib_points()
                        print(f"  [OK] 标定点 #{len(self.calib_points)}: "
                              f"旧图({ox}, {oy}) → 世界({wx:.2f}, {wy:.2f})")
                        self.transform_matrix = None  # 需要重新计算
                except (ValueError, IndexError):
                    print("  [ERROR] 格式错误，请输入两个数字，如: 10.5 6.3")
                except EOFError:
                    break
                finally:
                    self.pending_click = None

            if key == ord('p') or key == ord('P'):
                # 预览
                if len(self.calib_points) < 2:
                    print("[ERROR] 至少需要 2 个标定点才能预览！")
                    continue
                self.transform_matrix = None  # 强制重新计算
                preview = self.generate_preview()
                if preview is not None:
                    # 缩放预览窗口
                    scale = min(1.0, 1400.0 / CANVAS_W, 900.0 / CANVAS_H)
                    if scale < 1.0:
                        dw = int(CANVAS_W * scale)
                        dh = int(CANVAS_H * scale)
                        preview_display = cv2.resize(preview, (dw, dh))
                    else:
                        preview_display = preview
                    cv2.imshow(preview_win, preview_display)
                    preview_shown = True
                    print("[预览] 预览窗口已更新。满意请按 [w] 保存。")

            elif key == ord('w') or key == ord('W'):
                # 保存
                if len(self.calib_points) < 2:
                    print("[ERROR] 至少需要 2 个标定点才能保存！")
                    continue
                self.transform_matrix = None
                if self.save_new_map():
                    print("\n[SUCCESS] 新地图已保存！重启 display_panel 即可看到效果。")

            elif key == ord('u') or key == ord('U'):
                # 撤销
                if self.calib_points:
                    removed = self.calib_points.pop()
                    self.transform_matrix = None
                    self._save_calib_points()
                    print(f"[撤销] 移除标定点: 旧图({removed[0]:.0f}, {removed[1]:.0f}) "
                          f"→ 世界({removed[2]:.2f}, {removed[3]:.2f})")
                else:
                    print("[INFO] 没有可撤销的标定点")

            elif key == ord('r') or key == ord('R'):
                # 重置
                self.calib_points = []
                self.transform_matrix = None
                self._save_calib_points()
                print("[重置] 所有标定点已清除")

            elif key == ord('q') or key == ord('Q') or key == 27:  # ESC
                break

        cv2.destroyAllWindows()


def main():
    cfg = load_config()
    map_path = cfg['std_map']

    print(f"当前场景: {cfg['scene']}")
    print(f"地图文件: {map_path}")

    if not os.path.exists(map_path):
        print(f"[ERROR] 地图文件不存在: {map_path}")
        sys.exit(1)

    calibrator = MapCalibrator(map_path)
    calibrator.run()


if __name__ == '__main__':
    main()

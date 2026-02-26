#!/usr/bin/env python3
"""
calibrate_bev_coords.py — BEV 地图坐标系交互式标定工具

使用方法：
  python scripts/calibrate_bev_coords.py

操作流程：
  1. 弹出 BEV 地图窗口
  2. 先点击原点位置 (0, 0)
  3. 依次点击其他已知坐标点，每次点击后在终端输入该点的真实坐标 (x, y)
     支持负坐标，如 -2 3 表示 x=-2m, y=3m
  4. 至少标 3 个不共线的点后，按 's' 保存标定
  5. 按 'u' 撤销上一个点，按 'q' 退出

标定结果保存到 configs/bev_calib.json，包含：
  - affine_matrix: 2x3 仿射变换矩阵 (像素 → 米)
  - origin_px: 原点像素坐标
  - points: 所有标定点对
  - resolution: 米/像素（近似值）
"""

import os
import sys
import json
import numpy as np
import cv2

# 路径
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
BEV_PATH = os.path.join(PROJECT_ROOT, "map", "lab_bev.png")
CALIB_OUTPUT = os.path.join(PROJECT_ROOT, "configs", "bev_calib.json")

# 标定数据
clicked_pixels = []      # [(px, py), ...]
world_coords = []         # [(wx, wy), ...]  真实米坐标
current_click = None      # 最近一次点击的像素坐标（等待输入真实坐标）

# 显示参数
WINDOW_NAME = "BEV Coordinate Calibration"
display_scale = 1.0       # 显示缩放比例
img_orig = None
img_display = None


def on_mouse(event, x, y, flags, param):
    global current_click
    if event == cv2.EVENT_LBUTTONDOWN:
        # 转换回原图坐标
        ox = int(x / display_scale)
        oy = int(y / display_scale)
        current_click = (ox, oy)
        print(f"\n  点击像素: ({ox}, {oy})")
        if len(clicked_pixels) == 0 and len(world_coords) == 0:
            print("  → 这是原点 (0, 0)")
        else:
            print("  → 请在终端输入该点的真实坐标 (x y)，支持负数，如: -2 3")
            print("    输入后按回车确认:")


def redraw():
    """重绘标注"""
    global img_display
    img = img_orig.copy()
    h, w = img.shape[:2]

    for i, (px, py) in enumerate(clicked_pixels):
        wx, wy = world_coords[i]
        # 颜色：原点红色，其他绿色
        color = (0, 0, 255) if i == 0 else (0, 255, 0)
        cv2.circle(img, (px, py), 8, color, -1)
        cv2.circle(img, (px, py), 10, (255, 255, 255), 2)
        label = f"({wx:.1f}, {wy:.1f})"
        cv2.putText(img, label, (px + 12, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(img, label, (px + 12, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 1)

    # 如果有 >= 3 个点，画坐标轴
    if len(clicked_pixels) >= 3:
        M = compute_affine_inverse()
        if M is not None:
            draw_coordinate_grid(img, M)

    # 缩放显示
    if display_scale != 1.0:
        dw = int(w * display_scale)
        dh = int(h * display_scale)
        img_display = cv2.resize(img, (dw, dh))
    else:
        img_display = img

    cv2.imshow(WINDOW_NAME, img_display)


def compute_affine():
    """计算像素→米的仿射变换 (2x3)"""
    if len(clicked_pixels) < 3:
        return None
    px = np.array(clicked_pixels, dtype=np.float64)
    wx = np.array(world_coords, dtype=np.float64)

    if len(clicked_pixels) == 3:
        M = cv2.getAffineTransform(
            px[:3].astype(np.float32),
            wx[:3].astype(np.float32))
        return M
    else:
        # 超定方程，最小二乘
        # wx = M * [px, 1]^T
        n = len(px)
        A = np.zeros((2 * n, 6))
        b = np.zeros(2 * n)
        for i in range(n):
            A[2*i, 0] = px[i, 0]
            A[2*i, 1] = px[i, 1]
            A[2*i, 2] = 1
            b[2*i] = wx[i, 0]
            A[2*i+1, 3] = px[i, 0]
            A[2*i+1, 4] = px[i, 1]
            A[2*i+1, 5] = 1
            b[2*i+1] = wx[i, 1]
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        M = result.reshape(2, 3)
        return M


def compute_affine_inverse():
    """计算米→像素的仿射变换 (2x3)"""
    if len(clicked_pixels) < 3:
        return None
    px = np.array(clicked_pixels, dtype=np.float64)
    wx = np.array(world_coords, dtype=np.float64)

    if len(clicked_pixels) == 3:
        M = cv2.getAffineTransform(
            wx[:3].astype(np.float32),
            px[:3].astype(np.float32))
        return M
    else:
        n = len(wx)
        A = np.zeros((2 * n, 6))
        b = np.zeros(2 * n)
        for i in range(n):
            A[2*i, 0] = wx[i, 0]
            A[2*i, 1] = wx[i, 1]
            A[2*i, 2] = 1
            b[2*i] = px[i, 0]
            A[2*i+1, 3] = wx[i, 0]
            A[2*i+1, 4] = wx[i, 1]
            A[2*i+1, 5] = 1
            b[2*i+1] = px[i, 1]
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        M_inv = result.reshape(2, 3)
        return M_inv


def world_to_pixel(M_inv, wx, wy):
    """米坐标 → 像素坐标"""
    p = M_inv @ np.array([wx, wy, 1.0])
    return int(round(p[0])), int(round(p[1]))


def draw_coordinate_grid(img, M_inv):
    """在图上画坐标轴和网格"""
    h, w = img.shape[:2]

    # 确定坐标范围：把图像四角转换为米坐标
    M_fwd = compute_affine()
    if M_fwd is None:
        return
    corners_px = np.array([[0, 0], [w, 0], [w, h], [0, h]], dtype=np.float64)
    corners_world = []
    for cp in corners_px:
        wc = M_fwd @ np.array([cp[0], cp[1], 1.0])
        corners_world.append(wc)
    corners_world = np.array(corners_world)

    x_min = int(np.floor(corners_world[:, 0].min())) - 1
    x_max = int(np.ceil(corners_world[:, 0].max())) + 1
    y_min = int(np.floor(corners_world[:, 1].min())) - 1
    y_max = int(np.ceil(corners_world[:, 1].max())) + 1

    # 画网格线（浅灰）
    for xi in range(x_min, x_max + 1):
        p1 = world_to_pixel(M_inv, xi, y_min)
        p2 = world_to_pixel(M_inv, xi, y_max)
        if xi == 0:
            cv2.line(img, p1, p2, (0, 0, 200), 2)  # X轴红色
        else:
            cv2.line(img, p1, p2, (80, 80, 80), 1)

    for yi in range(y_min, y_max + 1):
        p1 = world_to_pixel(M_inv, x_min, yi)
        p2 = world_to_pixel(M_inv, x_max, yi)
        if yi == 0:
            cv2.line(img, p1, p2, (200, 0, 0), 2)  # Y轴蓝色
        else:
            cv2.line(img, p1, p2, (80, 80, 80), 1)

    # 标注整数坐标
    for xi in range(x_min, x_max + 1):
        for yi in range(y_min, y_max + 1):
            px, py = world_to_pixel(M_inv, xi, yi)
            if 0 <= px < w and 0 <= py < h:
                if xi == 0 and yi == 0:
                    cv2.circle(img, (px, py), 5, (0, 0, 255), -1)
                    cv2.putText(img, "O", (px + 8, py - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                elif xi % 2 == 0 and yi % 2 == 0:
                    cv2.putText(img, f"{xi},{yi}", (px + 4, py - 4),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (150, 150, 150), 1)


def update_main_config(field_w, field_h):
    """自动更新 main_config.yaml 中 lab 场景的 field_width / field_height"""
    config_path = os.path.join(PROJECT_ROOT, "configs", "main_config.yaml")
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            lines = f.readlines()

        # 找到 lab: 段落下的 field_width 和 field_height 并替换
        in_lab = False
        new_lines = []
        for line in lines:
            stripped = line.strip()
            # 检测进入 lab: 段
            if stripped == 'lab:':
                in_lab = True
            elif in_lab and stripped and not stripped.startswith('#') and ':' in stripped:
                key = stripped.split(':')[0].strip()
                if key == 'field_width':
                    indent = line[:len(line) - len(line.lstrip())]
                    line = f"{indent}field_width: {round(field_w, 1)}" \
                           f"                       # 由标定工具自动更新\n"
                elif key == 'field_height':
                    indent = line[:len(line) - len(line.lstrip())]
                    line = f"{indent}field_height: {round(field_h, 1)}" \
                           f"                      # 由标定工具自动更新\n"
            # 检测离开 lab 段（遇到同级别的其他 key）
            if in_lab and stripped and not stripped.startswith('#') and \
               not stripped.startswith('-') and ':' in stripped and \
               not line.startswith(' ') and not line.startswith('\t') and \
               stripped != 'lab:':
                # 顶级 key，离开 lab 段
                if not stripped.startswith('field') and not stripped.startswith('std_') \
                   and not stripped.startswith('pfa_') and not stripped.startswith('pcd_') \
                   and not stripped.startswith('downsampled'):
                    in_lab = False

            new_lines.append(line)

        with open(config_path, 'w', encoding='utf-8') as f:
            f.writelines(new_lines)
        print(f"    ✔ main_config.yaml lab 场景尺寸已更新: {field_w:.1f}m × {field_h:.1f}m")
    except Exception as e:
        print(f"    ⚠ 更新 main_config.yaml 失败: {e}")
        print(f"      请手动修改 lab.field_width={field_w:.1f}, lab.field_height={field_h:.1f}")


def save_calibration():
    """保存标定结果"""
    M = compute_affine()
    if M is None:
        print("  ✘ 至少需要 3 个标定点！")
        return False

    # 计算重投影误差
    px = np.array(clicked_pixels, dtype=np.float64)
    wx = np.array(world_coords, dtype=np.float64)
    errors = []
    for i in range(len(px)):
        pred = M @ np.array([px[i, 0], px[i, 1], 1.0])
        err = np.linalg.norm(pred - wx[i])
        errors.append(err)

    # 计算近似分辨率
    sx = np.sqrt(M[0, 0]**2 + M[1, 0]**2)  # 米/像素 (x方向)
    sy = np.sqrt(M[0, 1]**2 + M[1, 1]**2)  # 米/像素 (y方向)

    calib = {
        "affine_pixel_to_meter": M.tolist(),
        "origin_px": list(clicked_pixels[0]),
        "points": [
            {"pixel": list(clicked_pixels[i]),
             "world": list(world_coords[i])}
            for i in range(len(clicked_pixels))
        ],
        "resolution_x": float(sx),
        "resolution_y": float(sy),
        "reprojection_errors_m": [round(e, 4) for e in errors],
        "mean_error_m": round(float(np.mean(errors)), 4),
        "bev_image": "map/lab_bev.png",
    }

    # 计算 BEV 图像覆盖的真实尺寸
    h_img, w_img = img_orig.shape[:2]
    corners = np.array([[0, 0], [w_img, 0], [w_img, h_img], [0, h_img]], dtype=np.float64)
    world_corners = []
    for c in corners:
        wc = M @ np.array([c[0], c[1], 1.0])
        world_corners.append(wc)
    world_corners = np.array(world_corners)
    field_w = float(world_corners[:, 0].max() - world_corners[:, 0].min())
    field_h = float(world_corners[:, 1].max() - world_corners[:, 1].min())
    calib["field_width"] = round(field_w, 2)
    calib["field_height"] = round(field_h, 2)

    os.makedirs(os.path.dirname(CALIB_OUTPUT), exist_ok=True)
    with open(CALIB_OUTPUT, 'w') as f:
        json.dump(calib, f, indent=2, ensure_ascii=False)

    print(f"\n  ✔ 标定已保存: {CALIB_OUTPUT}")
    print(f"    分辨率: {sx:.5f} m/px (x), {sy:.5f} m/px (y)")
    print(f"    BEV覆盖范围: {field_w:.1f}m × {field_h:.1f}m")
    print(f"    重投影误差: 均值={np.mean(errors):.4f}m, 最大={np.max(errors):.4f}m")
    print(f"    标定点数: {len(clicked_pixels)}")

    # 自动更新 main_config.yaml 中 lab 场景的尺寸
    update_main_config(field_w, field_h)

    # 也保存一份带坐标轴的预览图
    preview_path = os.path.join(PROJECT_ROOT, "map", "lab_bev_calibrated.png")
    img_preview = img_orig.copy()
    M_inv = compute_affine_inverse()
    if M_inv is not None:
        draw_coordinate_grid(img_preview, M_inv)
    for i, (px, py) in enumerate(clicked_pixels):
        wx, wy = world_coords[i]
        color = (0, 0, 255) if i == 0 else (0, 255, 0)
        cv2.circle(img_preview, (px, py), 8, color, -1)
        label = f"({wx:.0f},{wy:.0f})"
        cv2.putText(img_preview, label, (px + 12, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imwrite(preview_path, img_preview)
    print(f"    预览图: {preview_path}")

    return True


def main():
    global img_orig, display_scale, current_click

    bev_path = BEV_PATH
    if len(sys.argv) > 1:
        bev_path = sys.argv[1]

    print(f"加载 BEV: {bev_path}")
    img_orig = cv2.imread(bev_path)
    if img_orig is None:
        print(f"✘ 无法读取: {bev_path}")
        sys.exit(1)

    h, w = img_orig.shape[:2]
    print(f"  尺寸: {w}x{h}")

    # 自动缩放以适应屏幕
    screen_w, screen_h = 1600, 900
    scale_w = screen_w / w
    scale_h = screen_h / h
    display_scale = min(scale_w, scale_h, 1.0)
    if display_scale < 1.0:
        print(f"  显示缩放: {display_scale:.2f}")

    print()
    print("=" * 50)
    print("  BEV 坐标系标定工具")
    print("=" * 50)
    print()
    print("操作步骤:")
    print("  1. 点击地图上的原点位置 (将自动设为 0,0)")
    print("  2. 点击其他已知坐标点，在终端输入真实坐标")
    print("     格式: x y （支持负数，如 -2 3）")
    print("  3. 至少标 3 个不共线的点")
    print()
    print("快捷键:")
    print("  s = 保存标定    u = 撤销上一点    q = 退出")
    print()
    print("请点击原点位置...")

    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    redraw()

    while True:
        key = cv2.waitKey(50) & 0xFF

        # 处理点击
        if current_click is not None:
            px, py = current_click
            current_click = None

            if len(clicked_pixels) == 0 and len(world_coords) == 0:
                # 第一个点 = 原点
                clicked_pixels.append((px, py))
                world_coords.append((0.0, 0.0))
                print(f"  ✔ 原点已设置: 像素({px}, {py}) → (0, 0)m")
                print(f"\n  现在点击下一个已知坐标点...")
                redraw()
            else:
                # 等待用户输入真实坐标
                try:
                    coord_str = input("    坐标 (x y): ").strip()
                    if coord_str:
                        parts = coord_str.replace(',', ' ').split()
                        wx = float(parts[0])
                        wy = float(parts[1])
                        clicked_pixels.append((px, py))
                        world_coords.append((wx, wy))
                        n = len(clicked_pixels)
                        print(f"  ✔ 点 {n}: 像素({px}, {py}) → ({wx}, {wy})m")
                        if n < 3:
                            print(f"    还需 {3 - n} 个点才能计算变换")
                        else:
                            print(f"    已有 {n} 个点，可按 's' 保存")
                        redraw()
                    else:
                        print("  ✘ 输入为空，忽略此点")
                except (ValueError, IndexError):
                    print("  ✘ 格式错误，请输入两个数字，如: -2 3")
                except EOFError:
                    break

        # 快捷键
        if key == ord('s'):
            save_calibration()
            redraw()
        elif key == ord('u'):
            if clicked_pixels:
                removed_px = clicked_pixels.pop()
                removed_wx = world_coords.pop()
                print(f"  撤销: ({removed_wx[0]}, {removed_wx[1]})m")
                redraw()
        elif key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()
    print("\n标定工具已退出。")


if __name__ == '__main__':
    main()

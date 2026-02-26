#!/usr/bin/env python3
"""
prepare_lab_map.py — 实验室地图裁剪与坐标系建立

已知条件：
  原图 lab_bev.png: 2551×2185 像素，对应现实 28.3m × 24.1m
  （像素x方向=现实x方向=28.3m，像素y方向=现实y方向=24.1m，y轴反向）

步骤：
  1. 根据已知的像素/米比率，计算28m×15m对应的像素数
  2. 从原图左下角裁剪该区域
  3. resize到2800×1500像素（100px/m）
  4. 建立坐标系：图片左下角=世界(0,0)，x向右，y向上
  5. 计算原始标定点在新坐标系中的位置
  6. 生成带网格的地图
"""

import cv2
import numpy as np
import json
import os

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ============ 输入 ============
ORIG_BEV_PATH = os.path.join(PROJECT_ROOT, "map", "lab_bev.png")
ORIG_CALIB_PATH = os.path.join(PROJECT_ROOT, "configs", "bev_calib.json")

# ============ 输出 ============
NEW_MAP_PATH = os.path.join(PROJECT_ROOT, "map", "lab_map_28x15.png")
NEW_GRID_MAP_PATH = os.path.join(PROJECT_ROOT, "map", "lab_map_28x15_grid.png")
NEW_CALIB_PATH = os.path.join(PROJECT_ROOT, "configs", "bev_calib_lab.json")

# ============ 已知参数 ============
# 原图对应的现实尺寸（来自标定）
ORIG_FIELD_W = 28.3   # 原图宽度对应的现实米数（x方向）
ORIG_FIELD_H = 24.1   # 原图高度对应的现实米数（y方向）

# 目标裁剪的现实尺寸
TARGET_FIELD_W = 28.0  # 米，x轴
TARGET_FIELD_H = 15.0  # 米，y轴

# 目标输出像素
TARGET_PX_W = 2800     # 像素
TARGET_PX_H = 1500     # 像素
SCALE = 100.0           # 目标 px/m


def main():
    # ========== 1. 读取原始图片和标定数据 ==========
    img = cv2.imread(ORIG_BEV_PATH)
    if img is None:
        print(f"错误：无法读取 {ORIG_BEV_PATH}")
        return
    orig_h, orig_w = img.shape[:2]
    print(f"原始图片尺寸: {orig_w}×{orig_h} 像素")
    print(f"原始图片对应现实: {ORIG_FIELD_W}m × {ORIG_FIELD_H}m")

    with open(ORIG_CALIB_PATH, 'r') as f:
        calib = json.load(f)

    # ========== 2. 计算像素/米比率 ==========
    orig_ppm_x = orig_w / ORIG_FIELD_W  # 像素/米，x方向
    orig_ppm_y = orig_h / ORIG_FIELD_H  # 像素/米，y方向
    print(f"原图像素密度: x={orig_ppm_x:.2f} px/m, y={orig_ppm_y:.2f} px/m")

    # ========== 3. 计算裁剪区域 ==========
    # 28m × 15m 对应的像素数
    crop_w = int(round(TARGET_FIELD_W * orig_ppm_x))  # 28 * 90.1 = 2523
    crop_h = int(round(TARGET_FIELD_H * orig_ppm_y))  # 15 * 90.7 = 1360

    # 确保不超出原图
    crop_w = min(crop_w, orig_w)
    crop_h = min(crop_h, orig_h)

    # 从左下角裁剪
    x_start = 0
    y_start = orig_h - crop_h  # 从底部往上 crop_h 像素

    print(f"\n裁剪 {TARGET_FIELD_W}m × {TARGET_FIELD_H}m:")
    print(f"  裁剪像素: {crop_w}×{crop_h}")
    print(f"  裁剪区域: x=[{x_start}, {x_start + crop_w}], y=[{y_start}, {y_start + crop_h}]")
    print(f"  保留左下角区域")

    cropped = img[y_start:y_start + crop_h, x_start:x_start + crop_w]

    # ========== 4. resize 到 2800×1500 ==========
    new_map = cv2.resize(cropped, (TARGET_PX_W, TARGET_PX_H), interpolation=cv2.INTER_LINEAR)
    cv2.imwrite(NEW_MAP_PATH, new_map)
    print(f"\n新地图已保存: {NEW_MAP_PATH} ({TARGET_PX_W}×{TARGET_PX_H})")

    # resize 缩放因子
    resize_sx = TARGET_PX_W / crop_w   # 2800 / 2523
    resize_sy = TARGET_PX_H / crop_h   # 1500 / 1360
    print(f"resize 缩放因子: x={resize_sx:.4f}, y={resize_sy:.4f}")

    # ========== 5. 坐标系定义 ==========
    # 新地图坐标系（与赛场完全一致）：
    #   世界(0,0) = 图片左下角 = 像素(0, 1499)
    #   世界x向右，世界y向上
    #   px = wx * 100,  py = 1500 - wy * 100
    #
    # 原图像素 → 新地图像素：
    #   new_px = (orig_px - x_start) * resize_sx
    #   new_py = (orig_py - y_start) * resize_sy
    #
    # 新地图像素 → 世界坐标：
    #   wx = new_px / 100
    #   wy = (1500 - new_py) / 100

    def orig_pixel_to_new_pixel(orig_px, orig_py):
        """原图像素 → 新地图像素"""
        new_px = (orig_px - x_start) * resize_sx
        new_py = (orig_py - y_start) * resize_sy
        return new_px, new_py

    def new_pixel_to_world(new_px, new_py):
        """新地图像素 → 世界坐标"""
        wx = new_px / SCALE
        wy = (TARGET_PX_H - new_py) / SCALE
        return wx, wy

    def orig_pixel_to_world(orig_px, orig_py):
        """原图像素 → 新世界坐标"""
        npx, npy = orig_pixel_to_new_pixel(orig_px, orig_py)
        return new_pixel_to_world(npx, npy)

    def world_to_new_pixel(wx, wy):
        """世界坐标 → 新地图像素"""
        px = int(round(wx * SCALE))
        py = int(round(TARGET_PX_H - wy * SCALE))
        return px, py

    # ========== 6. 计算原始标定点在新坐标系中的位置 ==========
    orig_points = calib["points"]

    print("\n" + "=" * 75)
    print("原始标定点在新坐标系中的位置")
    print("=" * 75)
    print(f"{'原始标定(x,y)':>16s} | {'原图像素':>14s} | {'新地图像素':>14s} | {'新世界坐标(x,y)':>18s} | 状态")
    print("-" * 75)

    new_points = []
    for p in orig_points:
        orig_world = p["world"]
        orig_px, orig_py = p["pixel"]

        npx, npy = orig_pixel_to_new_pixel(orig_px, orig_py)
        new_wx, new_wy = new_pixel_to_world(npx, npy)
        map_px, map_py = world_to_new_pixel(new_wx, new_wy)

        new_points.append({
            "original_label": f"({orig_world[0]},{orig_world[1]})",
            "original_pixel": [orig_px, orig_py],
            "new_pixel": [round(npx, 1), round(npy, 1)],
            "new_world": [round(new_wx, 2), round(new_wy, 2)]
        })

        in_range = (0 <= new_wx <= TARGET_FIELD_W and 0 <= new_wy <= TARGET_FIELD_H)
        marker = "✓" if in_range else "✗ 超出范围"

        print(f"  ({orig_world[0]:>4},{orig_world[1]:>4}) | ({orig_px:>5},{orig_py:>5}) | "
              f"({npx:>7.1f},{npy:>5.1f}) | ({new_wx:>7.2f},{new_wy:>7.2f}) | {marker}")

    # ========== 7. 生成带网格的地图 ==========
    grid_map = new_map.copy()
    grid_color = (100, 100, 100)
    text_color = (0, 200, 0)

    for x_m in range(int(TARGET_FIELD_W) + 1):
        px = int(x_m * SCALE)
        thickness = 2 if x_m % 5 == 0 else 1
        cv2.line(grid_map, (px, 0), (px, TARGET_PX_H), grid_color, thickness)
        if x_m % 5 == 0:
            cv2.putText(grid_map, str(x_m), (px + 5, TARGET_PX_H - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

    for y_m in range(int(TARGET_FIELD_H) + 1):
        py = int(TARGET_PX_H - y_m * SCALE)
        thickness = 2 if y_m % 5 == 0 else 1
        cv2.line(grid_map, (0, py), (TARGET_PX_W, py), grid_color, thickness)
        if y_m % 5 == 0:
            cv2.putText(grid_map, str(y_m), (5, py - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, text_color, 2)

    # 标注原点
    cv2.circle(grid_map, (0, TARGET_PX_H - 1), 15, (0, 0, 255), -1)
    cv2.putText(grid_map, "O(0,0)", (10, TARGET_PX_H - 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # 标注标定点
    for pt in new_points:
        npx_r, npy_r = int(round(pt["new_pixel"][0])), int(round(pt["new_pixel"][1]))
        nwx, nwy = pt["new_world"]
        label = pt["original_label"]
        if 0 <= npx_r < TARGET_PX_W and 0 <= npy_r < TARGET_PX_H:
            cv2.circle(grid_map, (npx_r, npy_r), 10, (0, 255, 255), -1)
            cv2.putText(grid_map, f"{label}->({nwx:.1f},{nwy:.1f})",
                        (npx_r + 12, npy_r - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.imwrite(NEW_GRID_MAP_PATH, grid_map)
    print(f"\n带网格的地图已保存: {NEW_GRID_MAP_PATH}")

    # ========== 8. 保存标定信息 ==========
    new_calib = {
        "description": "实验室地图（从左下角裁剪28x15m，resize到2800x1500px）",
        "coordinate_convention": {
            "world_origin": "图片左下角 = 像素(0, 1499) = 世界(0, 0)",
            "world_x": "向右，0~28m（对应像素x方向）",
            "world_y": "向上，0~15m（对应像素y方向，反向）",
            "pixel_from_world": "px = wx * 100, py = 1500 - wy * 100",
            "world_from_pixel": "wx = px / 100, wy = (1500 - py) / 100"
        },
        "field_width": TARGET_FIELD_W,
        "field_height": TARGET_FIELD_H,
        "px_width": TARGET_PX_W,
        "px_height": TARGET_PX_H,
        "scale_px_per_m": SCALE,
        "original_image": {
            "path": "map/lab_bev.png",
            "size_px": [orig_w, orig_h],
            "size_m": [ORIG_FIELD_W, ORIG_FIELD_H],
            "ppm_x": round(orig_ppm_x, 2),
            "ppm_y": round(orig_ppm_y, 2)
        },
        "crop_info": {
            "x_start": x_start,
            "y_start": y_start,
            "crop_w": crop_w,
            "crop_h": crop_h
        },
        "calibration_points_mapping": new_points,
        "bev_image": "map/lab_map_28x15.png",
        "grid_image": "map/lab_map_28x15_grid.png"
    }

    with open(NEW_CALIB_PATH, 'w', encoding='utf-8') as f:
        json.dump(new_calib, f, indent=2, ensure_ascii=False)
    print(f"标定信息已保存: {NEW_CALIB_PATH}")

    # ========== 9. 总结 ==========
    print("\n" + "=" * 75)
    print("总结")
    print("=" * 75)
    print(f"新地图: {NEW_MAP_PATH} ({TARGET_PX_W}×{TARGET_PX_H}px = {TARGET_FIELD_W}×{TARGET_FIELD_H}m)")
    print(f"网格图: {NEW_GRID_MAP_PATH}")
    print(f"坐标转换: px = wx*100, py = 1500 - wy*100 （与赛场完全一致）")
    print(f"世界原点(0,0) = 图片左下角")


if __name__ == "__main__":
    main()

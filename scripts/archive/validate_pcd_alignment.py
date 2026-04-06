#!/usr/bin/env python3
"""
validate_pcd_alignment.py — PCD 地图对齐精度定量验证工具

原理：
  align_pcd_with_map.py 通过人工视觉对齐将 PCD 从雷达坐标系变换到世界坐标系。
  本工具通过以下方法定量评估对齐质量：

  1. 控制点验证法：
     在已知世界坐标的特征点（墙角、柱子等）处，从对齐后的 PCD 中提取最近邻点，
     计算坐标偏差，统计 RMSE / 最大误差 / 方向性偏差。

  2. 边界一致性检验：
     检查对齐后 PCD 的 XY 边界是否落在期望的地图范围 [0, field_w] × [0, field_h] 内。

  3. 旋转角灵敏度分析：
     在当前对齐参数附近扫描 ±Δθ，评估旋转角误差对远点坐标的影响幅度。

  4. 俯视图叠加可视化：
     将对齐后 PCD 俯视图与地图图片叠加，输出高分辨率对比图。

误差传播估算：
  设对齐参数误差为 (δrx, δry, δθ)，PCD 中某点距雷达原点距离为 d，
  则该点在世界坐标系中的位置误差上界为：
    δp ≤ √(δrx² + δry²) + d × |δθ|  (δθ 单位:弧度)

  例如 δrx=0.05m, δry=0.05m, δθ=0.5°=0.0087rad, d=10m:
    δp ≤ √(0.05²+0.05²) + 10×0.0087 ≈ 0.07 + 0.087 = 0.16m

使用:
  # 基本验证（使用默认配置）
  python3 scripts/validate_pcd_alignment.py

  # 指定控制点文件
  python3 scripts/validate_pcd_alignment.py --control-points configs/lab_control_points.json

  # 完整分析 + 保存报告
  python3 scripts/validate_pcd_alignment.py --full-analysis --save-report
"""

import os
import sys
import json
import argparse
import numpy as np

try:
    import open3d as o3d
except ImportError:
    print("错误: 需要 open3d, pip install open3d")
    sys.exit(1)

try:
    import cv2
except ImportError:
    print("警告: 未安装 cv2, 可视化功能不可用")
    cv2 = None

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def load_config():
    """加载项目配置"""
    try:
        from ruamel.yaml import YAML
        yaml = YAML()
        config_path = os.path.join(PROJECT_ROOT, "configs", "main_config.yaml")
        with open(config_path, encoding="utf-8") as f:
            cfg = yaml.load(f)
        scene = cfg.get("global", {}).get("scene", "lab")
        sc = cfg.get("scenes", {}).get(scene, {})
        pcd_path = os.path.join(PROJECT_ROOT, sc.get("pcd_file", "data/pointclouds/registration/lab_pcds.pcd"))
        map_path = os.path.join(PROJECT_ROOT, sc.get("std_map", "data/maps/lab/lab_map_28x15.png"))
        field_w = sc.get("field_width", 28.0)
        field_h = sc.get("field_height", 15.0)
        return pcd_path, map_path, field_w, field_h
    except Exception as e:
        print(f"配置加载失败: {e}")
        return (
            os.path.join(PROJECT_ROOT, "data/pointclouds/registration/lab_pcds.pcd"),
            os.path.join(PROJECT_ROOT, "data/maps/lab/lab_map_28x15.png"),
            28.0,
            15.0,
        )


def load_transform_info():
    """加载对齐变换信息"""
    info_path = os.path.join(PROJECT_ROOT, "configs", "lab_pcd_transform_info.json")
    if os.path.exists(info_path):
        with open(info_path, "r", encoding="utf-8") as f:
            return json.load(f)
    return None


def analyze_pcd_bounds(pcd_path, field_w, field_h):
    """分析对齐后 PCD 的边界范围"""
    print("\n" + "=" * 60)
    print("1. PCD 边界一致性检验")
    print("=" * 60)

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)
    n = len(pts)

    x_min, x_max = pts[:, 0].min(), pts[:, 0].max()
    y_min, y_max = pts[:, 1].min(), pts[:, 1].max()
    z_min, z_max = pts[:, 2].min(), pts[:, 2].max()

    print(f"\n  点云文件: {pcd_path}")
    print(f"  总点数: {n:,}")
    print(f"  X 范围: [{x_min:.3f}, {x_max:.3f}] m  (期望 [0, {field_w}])")
    print(f"  Y 范围: [{y_min:.3f}, {y_max:.3f}] m  (期望 [0, {field_h}])")
    print(f"  Z 范围: [{z_min:.3f}, {z_max:.3f}] m")

    # 检查边界是否合理
    issues = []
    margin = 2.0  # 允许 2m 余量

    if x_min < -margin:
        issues.append(f"  ⚠ X_min={x_min:.3f} 远低于 0, PCD 可能向左偏移过大")
    if x_max > field_w + margin:
        issues.append(f"  ⚠ X_max={x_max:.3f} 远超 {field_w}, PCD 可能向右偏移过大")
    if y_min < -margin:
        issues.append(f"  ⚠ Y_min={y_min:.3f} 远低于 0, PCD 可能向下偏移过大")
    if y_max > field_h + margin:
        issues.append(f"  ⚠ Y_max={y_max:.3f} 远超 {field_h}, PCD 可能向上偏移过大")

    # 检查主体覆盖率
    in_field = np.sum(
        (pts[:, 0] >= 0) & (pts[:, 0] <= field_w) &
        (pts[:, 1] >= 0) & (pts[:, 1] <= field_h)
    )
    coverage = in_field / n * 100
    print(f"\n  场地内点比例: {coverage:.1f}% ({in_field:,}/{n:,})")
    if coverage < 50:
        issues.append(f"  ⚠ 场地内点比例仅 {coverage:.1f}%, 对齐可能严重偏移")

    if issues:
        print("\n  发现问题:")
        for iss in issues:
            print(iss)
    else:
        print("\n  ✓ 边界检查通过")

    return pts


def validate_control_points(pcd_path, control_points, search_radius=0.3):
    """用控制点验证对齐精度"""
    print("\n" + "=" * 60)
    print("2. 控制点精度验证")
    print("=" * 60)

    if not control_points:
        print("\n  未提供控制点, 跳过此步骤")
        print("  提示: 创建 configs/lab_control_points.json 文件:")
        print('  [')
        print('    {"name": "墙角A", "world_xy": [2.0, 3.0], "description": "实验室西南角"}')
        print('    {"name": "柱子B", "world_xy": [5.5, 7.2], "description": "中间柱子"}')
        print('  ]')
        return None

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)

    # 构建 KD-Tree 用于最近邻搜索
    tree = o3d.geometry.KDTreeFlann(pcd)

    errors = []
    print(f"\n  {'控制点':<12} {'期望坐标':<16} {'PCD最近点':<16} {'误差(m)':<10} {'方向'}")
    print("  " + "-" * 70)

    for cp in control_points:
        name = cp["name"]
        wx, wy = cp["world_xy"]
        wz = cp.get("world_z", None)

        # 在 PCD 中搜索最近邻
        query = [wx, wy, wz if wz is not None else 0.0]
        [k, idx, dist2] = tree.search_radius_vector_3d(query, search_radius)

        if k == 0:
            # 扩大搜索范围
            [k, idx, dist2] = tree.search_knn_vector_3d(query, 1)

        if k > 0:
            nearest = pts[idx[0]]
            dx = nearest[0] - wx
            dy = nearest[1] - wy
            dist_2d = np.sqrt(dx ** 2 + dy ** 2)

            # 方向判断
            angle = np.degrees(np.arctan2(dy, dx))
            if abs(dx) > abs(dy):
                direction = "→偏右" if dx > 0 else "←偏左"
            else:
                direction = "↑偏上" if dy > 0 else "↓偏下"

            errors.append({
                "name": name,
                "expected": [wx, wy],
                "actual": [nearest[0], nearest[1]],
                "dx": dx,
                "dy": dy,
                "dist_2d": dist_2d,
                "angle_deg": angle,
            })

            print(f"  {name:<12} ({wx:.2f},{wy:.2f})    ({nearest[0]:.2f},{nearest[1]:.2f})    "
                  f"{dist_2d:.3f}     {direction} (dx={dx:+.3f}, dy={dy:+.3f})")
        else:
            print(f"  {name:<12} ({wx:.2f},{wy:.2f})    未找到最近点")

    if errors:
        dists = [e["dist_2d"] for e in errors]
        dxs = [e["dx"] for e in errors]
        dys = [e["dy"] for e in errors]

        rmse = np.sqrt(np.mean(np.array(dists) ** 2))
        max_err = np.max(dists)
        mean_dx = np.mean(dxs)
        mean_dy = np.mean(dys)

        print(f"\n  RMSE: {rmse:.3f} m")
        print(f"  最大误差: {max_err:.3f} m")
        print(f"  系统性偏移: dx_mean={mean_dx:+.3f} m, dy_mean={mean_dy:+.3f} m")

        if abs(mean_dx) > 0.05 or abs(mean_dy) > 0.05:
            print(f"\n  ⚠ 检测到系统性偏移！")
            print(f"    建议在 align_pcd_with_map.py 中调整:")
            print(f"    - radar_x 调整约 {-mean_dx:+.3f} m")
            print(f"    - radar_y 调整约 {-mean_dy:+.3f} m")

        if rmse > 0.3:
            print(f"\n  ⚠ RMSE={rmse:.3f}m 较大, 建议重新对齐")
        elif rmse > 0.15:
            print(f"\n  △ RMSE={rmse:.3f}m 可接受, 但仍有改进空间")
        else:
            print(f"\n  ✓ RMSE={rmse:.3f}m, 精度良好")

    return errors


def rotation_sensitivity_analysis(transform_info, field_w, field_h):
    """旋转角灵敏度分析"""
    print("\n" + "=" * 60)
    print("3. 旋转角灵敏度分析")
    print("=" * 60)

    if transform_info is None:
        print("\n  未找到对齐信息, 跳过")
        return

    rx, ry, rh = transform_info["radar_world_position"]
    yaw = transform_info["yaw_deg"]

    print(f"\n  当前对齐参数:")
    print(f"    雷达位置: ({rx}, {ry}, {rh}) m")
    print(f"    旋转角: {yaw}°")

    # 分析在不同距离处, 旋转角误差的影响
    print(f"\n  旋转角误差 → 位置误差（距雷达不同距离处）:")
    print(f"  {'距离(m)':<10} {'δθ=0.1°':<12} {'δθ=0.5°':<12} {'δθ=1.0°':<12} {'δθ=2.0°':<12}")
    print("  " + "-" * 58)

    for d in [3, 5, 8, 10, 15, 20]:
        errs = []
        for dtheta in [0.1, 0.5, 1.0, 2.0]:
            err = d * np.radians(dtheta)
            errs.append(err)
        print(f"  {d:<10} {errs[0]:<12.3f} {errs[1]:<12.3f} {errs[2]:<12.3f} {errs[3]:<12.3f}")

    # 分析平移误差的影响
    print(f"\n  平移误差 → 位置误差（直接叠加，与距离无关）:")
    print(f"    δrx=0.05m, δry=0.05m → δp = {np.sqrt(0.05**2 + 0.05**2):.3f} m")
    print(f"    δrx=0.10m, δry=0.10m → δp = {np.sqrt(0.10**2 + 0.10**2):.3f} m")
    print(f"    δrx=0.20m, δry=0.20m → δp = {np.sqrt(0.20**2 + 0.20**2):.3f} m")

    # 组合误差估算
    print(f"\n  综合误差估算 (δrx=δry=0.05m, δθ=0.5°):")
    trans_err = np.sqrt(0.05 ** 2 + 0.05 ** 2)
    for d in [5, 10, 15]:
        rot_err = d * np.radians(0.5)
        total = trans_err + rot_err
        print(f"    d={d}m: 平移误差={trans_err:.3f}m + 旋转误差={rot_err:.3f}m = 总计 ≤ {total:.3f}m")

    # 对齐工具精度评估
    print(f"\n  align_pcd_with_map.py 工具精度评估:")
    print(f"    渲染分辨率: 80 px/m")
    print(f"    精调步长: 1 px = {1.0 / 80:.4f} m = {1.0 / 80 * 100:.2f} cm")
    print(f"    旋转精调步长: 0.1°")
    print(f"    理论最小平移误差: ±{1.0 / 80:.4f} m (±{1.0 / 80 * 100:.2f} cm)")
    print(f"    理论最小旋转误差: ±0.05° (半步)")
    print(f"    但人眼对齐精度通常为 ±3~5 px = ±{3.0 / 80:.3f}~{5.0 / 80:.3f} m")
    print(f"    和 ±0.3~0.5°")


def generate_overlay_visualization(pcd_path, map_path, field_w, field_h, output_path=None):
    """生成对齐后 PCD 与地图的叠加可视化"""
    print("\n" + "=" * 60)
    print("4. 俯视图叠加可视化")
    print("=" * 60)

    if cv2 is None:
        print("\n  cv2 不可用, 跳过可视化")
        return

    pcd = o3d.io.read_point_cloud(pcd_path)
    pts = np.asarray(pcd.points)

    map_img = cv2.imread(map_path)
    if map_img is None:
        print(f"\n  地图加载失败: {map_path}")
        return

    # 高分辨率渲染: 100 px/m
    res = 100
    canvas_w = int(field_w * res)
    canvas_h = int(field_h * res)

    # 缩放地图到画布尺寸
    map_resized = cv2.resize(map_img, (canvas_w, canvas_h))

    # 渲染 PCD 俯视图
    pcd_canvas = np.zeros((canvas_h, canvas_w, 3), dtype=np.uint8)

    # 将世界坐标转为画布像素
    px = (pts[:, 0] * res).astype(np.int32)
    py = (canvas_h - pts[:, 1] * res).astype(np.int32)  # Y 翻转

    mask = (px >= 0) & (px < canvas_w) & (py >= 0) & (py < canvas_h)
    px_valid, py_valid = px[mask], py[mask]
    z_valid = pts[mask, 2]

    if len(z_valid) > 0:
        z_min, z_max = z_valid.min(), z_valid.max()
        z_norm = np.clip((z_valid - z_min) / max(z_max - z_min, 0.01), 0, 1)
        colors = (80 + 175 * z_norm).astype(np.uint8)
        pcd_canvas[py_valid, px_valid, 0] = colors  # B
        pcd_canvas[py_valid, px_valid, 1] = colors  # G

    # 叠加
    alpha = 0.45
    overlay = cv2.addWeighted(map_resized, 1.0, pcd_canvas, alpha, 0)

    # 绘制坐标网格
    for x in range(0, int(field_w) + 1, 5):
        xx = int(x * res)
        cv2.line(overlay, (xx, 0), (xx, canvas_h), (0, 100, 0), 1)
        cv2.putText(overlay, f"{x}m", (xx + 3, canvas_h - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)
    for y in range(0, int(field_h) + 1, 5):
        yy = canvas_h - int(y * res)
        cv2.line(overlay, (0, yy), (canvas_w, yy), (0, 100, 0), 1)
        cv2.putText(overlay, f"{y}m", (3, yy - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 0), 1)

    # 标记原点
    cv2.drawMarker(overlay, (0, canvas_h), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
    cv2.putText(overlay, "(0,0)", (5, canvas_h - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if output_path is None:
        output_path = os.path.join(PROJECT_ROOT, "test_output", "pcd_alignment_overlay.png")
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    cv2.imwrite(output_path, overlay)
    print(f"\n  叠加图已保存: {output_path}")
    print(f"  分辨率: {canvas_w}x{canvas_h} ({res} px/m)")
    print(f"  检查要点:")
    print(f"    - PCD 点云轮廓（青色）是否与地图墙壁/结构重合")
    print(f"    - 特别注意远端（距雷达 >10m）区域的偏差")
    print(f"    - 旋转偏差表现为一端对齐而另一端扭曲")


def compute_alignment_quality_score(pts, field_w, field_h, transform_info, control_errors=None):
    """计算综合对齐质量评分 (0~100)"""
    print("\n" + "=" * 60)
    print("5. 综合对齐质量评分")
    print("=" * 60)

    scores = {}

    # 评分项1: 边界覆盖率 (30分)
    in_field = np.sum(
        (pts[:, 0] >= -0.5) & (pts[:, 0] <= field_w + 0.5) &
        (pts[:, 1] >= -0.5) & (pts[:, 1] <= field_h + 0.5)
    )
    coverage = in_field / len(pts)
    scores["边界覆盖率"] = min(30, int(coverage * 30))

    # 评分项2: 控制点精度 (40分)
    if control_errors:
        rmse = np.sqrt(np.mean([e["dist_2d"] ** 2 for e in control_errors]))
        if rmse < 0.1:
            scores["控制点精度"] = 40
        elif rmse < 0.2:
            scores["控制点精度"] = 30
        elif rmse < 0.3:
            scores["控制点精度"] = 20
        elif rmse < 0.5:
            scores["控制点精度"] = 10
        else:
            scores["控制点精度"] = 0
    else:
        scores["控制点精度"] = None  # 未测试

    # 评分项3: Z 轴合理性 (15分)
    z_median = np.median(pts[:, 2])
    if 0.0 < z_median < 3.0:
        scores["Z轴合理性"] = 15
    elif -1.0 < z_median < 5.0:
        scores["Z轴合理性"] = 10
    else:
        scores["Z轴合理性"] = 0

    # 评分项4: 系统性偏移 (15分)
    if control_errors:
        mean_dx = abs(np.mean([e["dx"] for e in control_errors]))
        mean_dy = abs(np.mean([e["dy"] for e in control_errors]))
        sys_err = np.sqrt(mean_dx ** 2 + mean_dy ** 2)
        if sys_err < 0.05:
            scores["系统性偏移"] = 15
        elif sys_err < 0.1:
            scores["系统性偏移"] = 10
        elif sys_err < 0.2:
            scores["系统性偏移"] = 5
        else:
            scores["系统性偏移"] = 0
    else:
        scores["系统性偏移"] = None

    # 汇总
    tested_scores = {k: v for k, v in scores.items() if v is not None}
    total = sum(tested_scores.values())
    max_possible = sum([30, 40, 15, 15] if all(v is not None for v in scores.values())
                       else [v_max for k, v_max, v in
                             zip(scores.keys(), [30, 40, 15, 15], scores.values()) if v is not None])

    print(f"\n  {'评分项':<16} {'得分':>6} {'满分':>6}")
    print("  " + "-" * 32)
    max_list = {"边界覆盖率": 30, "控制点精度": 40, "Z轴合理性": 15, "系统性偏移": 15}
    for name, score in scores.items():
        if score is not None:
            print(f"  {name:<16} {score:>6} /{max_list[name]:>5}")
        else:
            print(f"  {name:<16} {'未测试':>6} /{max_list[name]:>5}")

    print(f"  {'─' * 32}")
    if max_possible > 0:
        normalized = total / max_possible * 100
        print(f"  {'总分':<16} {total:>6} /{max_possible:>5}  ({normalized:.0f}/100)")
        if normalized >= 85:
            print(f"\n  ✓ 对齐质量: 优秀")
        elif normalized >= 70:
            print(f"\n  △ 对齐质量: 良好, 但有改进空间")
        elif normalized >= 50:
            print(f"\n  ⚠ 对齐质量: 一般, 建议重新对齐")
        else:
            print(f"\n  ✗ 对齐质量: 差, 需要重新对齐")


def print_error_propagation_analysis(transform_info):
    """打印误差传播链路分析"""
    print("\n" + "=" * 60)
    print("6. 完整坐标变换误差传播分析")
    print("=" * 60)

    print("""
  完整变换链路:
    p_world = T_map←livox × T_livox←camera × p_camera

  各环节误差源:

  PCD对齐误差:     0.05~0.5m  (系统性平移+旋转放大, 远距离放大)
  ICP配准误差:     0.1~3.0m   (随时间漂移+偶发跳变, 最大不确定性源)
  外参标定误差:    0.01~0.2m  (系统性偏移, 距离越远影响越大)
  内参+畸变误差:   0~30px     (选点偏差, 图像边缘区域更大)
  DBSCAN聚类误差:  0.1~0.5m   (随帧波动, 地面点未过滤会拉偏)
  检测框抖动误差:  0~0.5m     (随帧波动, 无装甲板时更明显)

  PCD对齐(align_pcd_with_map.py)的误差特征:
    - 平移误差(rx,ry): 均匀作用于所有点, 与距离无关
    - 旋转误差(theta): 距雷达越远的点误差越大, delta_p = d * delta_theta
    - 判断标志: 如果所有测试点都朝同一方向偏移, 可能是平移误差
                如果近处对齐好但远处偏大, 可能是旋转角误差""")

    if transform_info:
        rx, ry, rh = transform_info.get("radar_world_position", [0, 0, 0])
        yaw = transform_info.get("yaw_deg", 0)
        print(f"\n  当前 align_pcd 参数: rx={rx}, ry={ry}, rh={rh}, yaw={yaw} deg")
        print(f"  10m处旋转0.5度误差: {10 * np.radians(0.5):.3f} m")
        print(f"  15m处旋转0.5度误差: {15 * np.radians(0.5):.3f} m")


def main():
    parser = argparse.ArgumentParser(description="PCD 地图对齐精度定量验证工具")
    parser.add_argument("--pcd", default=None, help="PCD 文件路径 (默认从 main_config.yaml 读取)")
    parser.add_argument("--map", default=None, help="地图图片路径")
    parser.add_argument("--control-points", default=None,
                        help="控制点 JSON 文件路径 (格式: [{name, world_xy, ...}])")
    parser.add_argument("--full-analysis", action="store_true", help="运行全部分析")
    parser.add_argument("--save-report", action="store_true", help="保存报告到 test_output/")
    parser.add_argument("--output-overlay", default=None, help="叠加图输出路径")
    args = parser.parse_args()

    # 加载配置
    pcd_path, map_path, field_w, field_h = load_config()
    if args.pcd:
        pcd_path = args.pcd
    if args.map:
        map_path = args.map

    transform_info = load_transform_info()

    print("=" * 60)
    print("PCD 地图对齐精度验证报告")
    print("=" * 60)

    # 检查 PCD 文件
    if not os.path.exists(pcd_path):
        print(f"\n错误: PCD 文件不存在: {pcd_path}")
        sys.exit(1)

    # 1. 边界检验
    pts = analyze_pcd_bounds(pcd_path, field_w, field_h)

    # 2. 控制点验证
    control_points = None
    if args.control_points:
        cp_path = args.control_points
        if os.path.exists(cp_path):
            with open(cp_path, "r", encoding="utf-8") as f:
                control_points = json.load(f)
    control_errors = validate_control_points(pcd_path, control_points)

    # 3. 旋转灵敏度
    rotation_sensitivity_analysis(transform_info, field_w, field_h)

    # 4. 可视化
    if os.path.exists(map_path):
        generate_overlay_visualization(pcd_path, map_path, field_w, field_h, args.output_overlay)
    else:
        print(f"\n地图文件不存在: {map_path}, 跳过可视化")

    # 5. 质量评分
    compute_alignment_quality_score(pts, field_w, field_h, transform_info, control_errors)

    # 6. 误差传播分析
    if args.full_analysis:
        print_error_propagation_analysis(transform_info)

    print("\n" + "=" * 60)
    print("验证完成")
    print("=" * 60)


if __name__ == "__main__":
    main()

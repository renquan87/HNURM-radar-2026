#!/usr/bin/env python3
"""
YOLO 三阶段推理效果测试脚本（独立运行，不依赖 ROS2）

用法:
    python3 scripts/test_yolo_3stage.py                          # 使用默认测试图片
    python3 scripts/test_yolo_3stage.py --image path/to/img.jpg  # 指定图片
    python3 scripts/test_yolo_3stage.py --save-dir output/       # 指定输出目录

三阶段流程:
    Stage 1 (目标检测+追踪): stage_one.pt  → 检测车辆 bounding box
    Stage 2 (装甲板分类):    stage_two.pt  → 对每个 ROI 识别装甲板颜色+编号 (B1~R7)
    Stage 3 (灰色装甲板):    stage_three.pt → 对每个 ROI 识别灰色装甲板
"""

import sys
import os
import argparse
import time
import cv2
import numpy as np

# 把 ultralytics 所在路径加入 sys.path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src', 'hnurm_radar'))
from ultralytics import YOLO


# ── 默认路径 ──────────────────────────────────────────────
BASE = os.path.join(os.path.dirname(__file__), '..')
DEFAULT_IMAGE   = os.path.join(BASE, 'test_resources', 'red1.png')
STAGE_ONE_PATH  = os.path.join(BASE, 'weights', 'stage_one.pt')
STAGE_TWO_PATH  = os.path.join(BASE, 'weights', 'stage_two.pt')
STAGE_THREE_PATH = os.path.join(BASE, 'weights', 'stage_three.pt')

# 类别标签（与 detector_config.yaml 一致）
LABELS = ["B1", "B2", "B3", "B4", "B5", "B7",
          "R1", "R2", "R3", "R4", "R5", "R7"]

# 灰色装甲板映射
GRAY_LABELS = {0: "G1(工程)", 1: "G2(步兵)", 2: "G3(步兵)",
               3: "G4(步兵)", 4: "G5(步兵)", 5: "G7(哨兵)"}

# 颜色
COLOR_STAGE1 = (0, 255, 0)    # 绿色 - 一阶段检测框
COLOR_BLUE   = (255, 128, 0)  # 蓝色系
COLOR_RED    = (0, 0, 255)    # 红色系
COLOR_GRAY   = (180, 180, 180)  # 灰色


def load_models():
    """加载三个 YOLO 模型"""
    print("=" * 60)
    print("加载模型中...")

    t0 = time.time()
    model1 = YOLO(STAGE_ONE_PATH, task="detect")
    model1.overrides['imgsz'] = 1280
    t1 = time.time()
    print(f"  Stage 1 (目标检测): {STAGE_ONE_PATH}")
    print(f"    加载耗时: {t1 - t0:.2f}s")

    model2 = YOLO(STAGE_TWO_PATH)
    model2.overrides['imgsz'] = 256
    t2 = time.time()
    print(f"  Stage 2 (装甲板分类): {STAGE_TWO_PATH}")
    print(f"    加载耗时: {t2 - t1:.2f}s")

    model3 = YOLO(STAGE_THREE_PATH)
    model3.overrides['imgsz'] = 256
    t3 = time.time()
    print(f"  Stage 3 (灰色装甲板): {STAGE_THREE_PATH}")
    print(f"    加载耗时: {t3 - t2:.2f}s")

    print(f"  总加载耗时: {t3 - t0:.2f}s")
    print("=" * 60)
    return model1, model2, model3


def run_stage1(model, frame):
    """
    一阶段: 目标检测
    返回: results 原始结果, 以及解析后的 (boxes_xywh, confs) 列表
    """
    t0 = time.time()
    results = model.predict(frame, conf=0.1, verbose=False)
    dt = time.time() - t0

    detections = []
    if results and results[0].boxes is not None and len(results[0].boxes) > 0:
        boxes = results[0].boxes.xywh.cpu().numpy()
        confs = results[0].boxes.conf.cpu().numpy()
        for box, conf in zip(boxes, confs):
            detections.append({'xywh': box, 'conf': float(conf)})

    print(f"\n[Stage 1 - 目标检测]")
    print(f"  推理耗时: {dt*1000:.1f}ms")
    print(f"  检测到 {len(detections)} 个目标")
    for i, d in enumerate(detections):
        x, y, w, h = d['xywh']
        print(f"    #{i}: center=({x:.0f},{y:.0f}) size={w:.0f}x{h:.0f} conf={d['conf']:.3f}")

    return results, detections, dt


def run_stage2(model, frame, detections):
    """
    二阶段: 对每个检测框裁剪 ROI，送入分类模型
    返回: 每个检测的分类标签和置信度
    """
    if not detections:
        print(f"\n[Stage 2 - 装甲板分类] 无目标，跳过")
        return [], 0

    roi_list = []
    for d in detections:
        x, y, w, h = d['xywh']
        x1 = max(0, int(x - w / 2))
        y1 = max(0, int(y - h / 2))
        x2 = min(frame.shape[1], int(x + w / 2))
        y2 = min(frame.shape[0], int(y + h / 2))
        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            roi_list.append(roi)
        else:
            roi_list.append(np.zeros((64, 64, 3), dtype=np.uint8))

    t0 = time.time()
    results_s2 = []
    for roi in roi_list:
        res = model.predict(roi, conf=0.6, device=0, verbose=False)
        results_s2.append(res[0] if res else None)
    dt = time.time() - t0

    classifications = []
    print(f"\n[Stage 2 - 装甲板分类]")
    print(f"  推理耗时: {dt*1000:.1f}ms ({len(roi_list)} 个 ROI)")

    for i, (res, d) in enumerate(zip(results_s2, detections)):
        label = "NULL"
        conf = 0.0
        if res is not None and res.boxes is not None and len(res.boxes.data) > 0:
            data = res.boxes.data.cpu().numpy()
            # 取置信度最高的
            best_idx = np.argmax(data[:, 4])
            conf = float(data[best_idx, 4])
            cls_id = int(data[best_idx, 5])
            if 0 <= cls_id < len(LABELS):
                label = LABELS[cls_id]
            else:
                label = f"cls_{cls_id}"
        classifications.append({'label': label, 'conf': conf})
        print(f"    #{i}: {label} (conf={conf:.3f})")

    return classifications, dt


def run_stage3(model, frame, detections):
    """
    三阶段: 灰色装甲板检测
    返回: 每个检测的灰色分类结果
    """
    if not detections:
        print(f"\n[Stage 3 - 灰色装甲板] 无目标，跳过")
        return [], 0

    roi_list = []
    for d in detections:
        x, y, w, h = d['xywh']
        x1 = max(0, int(x - w / 2))
        y1 = max(0, int(y - h / 2))
        x2 = min(frame.shape[1], int(x + w / 2))
        y2 = min(frame.shape[0], int(y + h / 2))
        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            roi_list.append(roi)
        else:
            roi_list.append(np.zeros((64, 64, 3), dtype=np.uint8))

    t0 = time.time()
    results_s3 = []
    for roi in roi_list:
        res = model.predict(roi, verbose=False)
        results_s3.append(res[0] if res else None)
    dt = time.time() - t0

    gray_classifications = []
    print(f"\n[Stage 3 - 灰色装甲板]")
    print(f"  推理耗时: {dt*1000:.1f}ms ({len(roi_list)} 个 ROI)")

    for i, (res, d) in enumerate(zip(results_s3, detections)):
        label = "无灰色"
        conf = 0.0
        if res is not None and res.boxes is not None and len(res.boxes.data) > 0:
            data = res.boxes.data.cpu().numpy()
            best_idx = np.argmax(data[:, 4])
            conf = float(data[best_idx, 4])
            cls_id = int(data[best_idx, 5])
            label = GRAY_LABELS.get(cls_id, f"gray_cls_{cls_id}")
        gray_classifications.append({'label': label, 'conf': conf})
        print(f"    #{i}: {label} (conf={conf:.3f})")

    return gray_classifications, dt


def draw_results(frame, detections, stage2_results, stage3_results):
    """在图像上绘制三阶段结果"""
    vis = frame.copy()

    for i, d in enumerate(detections):
        x, y, w, h = d['xywh']
        x1, y1 = int(x - w / 2), int(y - h / 2)
        x2, y2 = int(x + w / 2), int(y + h / 2)

        # 确定颜色和标签
        s2 = stage2_results[i] if i < len(stage2_results) else {'label': 'NULL', 'conf': 0}
        s3 = stage3_results[i] if i < len(stage3_results) else {'label': '无灰色', 'conf': 0}

        label = s2['label']
        if label.startswith('B'):
            color = COLOR_BLUE
        elif label.startswith('R'):
            color = COLOR_RED
        else:
            color = COLOR_GRAY

        # 画框
        cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)

        # 标签文字
        text_s2 = f"S2: {s2['label']} ({s2['conf']:.2f})"
        text_s3 = f"S3: {s3['label']} ({s3['conf']:.2f})"

        # 背景框让文字更清晰
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        thickness = 1

        for j, text in enumerate([text_s2, text_s3]):
            ty = y1 - 10 - j * 20
            if ty < 15:
                ty = y2 + 20 + j * 20
            (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)
            cv2.rectangle(vis, (x1, ty - th - 2), (x1 + tw, ty + 2), (0, 0, 0), -1)
            cv2.putText(vis, text, (x1, ty), font, font_scale, color, thickness)

        # 一阶段置信度
        text_s1 = f"det: {d['conf']:.2f}"
        cv2.putText(vis, text_s1, (x2 + 3, y1 + 15), font, 0.4, COLOR_STAGE1, 1)

    return vis


def print_summary(detections, stage2_results, stage3_results, t1, t2, t3):
    """打印汇总表格"""
    print("\n" + "=" * 70)
    print("三阶段推理结果汇总")
    print("=" * 70)
    print(f"{'#':<4} {'S1检测conf':<12} {'S2分类':<10} {'S2conf':<10} {'S3灰色':<12} {'S3conf':<10}")
    print("-" * 70)

    for i, d in enumerate(detections):
        s2 = stage2_results[i] if i < len(stage2_results) else {'label': '-', 'conf': 0}
        s3 = stage3_results[i] if i < len(stage3_results) else {'label': '-', 'conf': 0}
        print(f"{i:<4} {d['conf']:<12.3f} {s2['label']:<10} {s2['conf']:<10.3f} {s3['label']:<12} {s3['conf']:<10.3f}")

    print("-" * 70)
    total = t1 + t2 + t3
    print(f"耗时统计: S1={t1*1000:.1f}ms  S2={t2*1000:.1f}ms  S3={t3*1000:.1f}ms  总计={total*1000:.1f}ms")
    if total > 0:
        print(f"等效帧率: {1/total:.1f} FPS")
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description='YOLO 三阶段推理效果测试')
    parser.add_argument('--image', default=DEFAULT_IMAGE, help='输入图片路径')
    parser.add_argument('--save-dir', default=os.path.join(BASE, 'test_output'),
                        help='结果保存目录')
    parser.add_argument('--infer-size', type=int, default=1920,
                        help='推理图像宽度 (高度按比例缩放)')
    parser.add_argument('--show', action='store_true', help='用 cv2.imshow 显示结果')
    args = parser.parse_args()

    # 读取图片
    print(f"读取图片: {args.image}")
    img = cv2.imread(args.image)
    if img is None:
        print(f"错误: 无法读取图片 {args.image}")
        sys.exit(1)
    print(f"  原始尺寸: {img.shape[1]}x{img.shape[0]}")

    # 缩放到推理分辨率
    h, w = img.shape[:2]
    infer_w = args.infer_size
    infer_h = int(h * infer_w / w)
    frame = cv2.resize(img, (infer_w, infer_h))
    print(f"  推理尺寸: {infer_w}x{infer_h}")

    # 加载模型
    model1, model2, model3 = load_models()

    # 三阶段推理
    _, detections, t1 = run_stage1(model1, frame)
    stage2_results, t2 = run_stage2(model2, frame, detections)
    stage3_results, t3 = run_stage3(model3, frame, detections)

    # 汇总
    print_summary(detections, stage2_results, stage3_results, t1, t2, t3)

    # 绘制可视化
    vis = draw_results(frame, detections, stage2_results, stage3_results)

    # 保存结果
    os.makedirs(args.save_dir, exist_ok=True)

    # 保存一阶段单独结果
    vis_s1 = frame.copy()
    for i, d in enumerate(detections):
        x, y, w, h = d['xywh']
        x1, y1 = int(x - w / 2), int(y - h / 2)
        x2, y2 = int(x + w / 2), int(y + h / 2)
        cv2.rectangle(vis_s1, (x1, y1), (x2, y2), COLOR_STAGE1, 2)
        cv2.putText(vis_s1, f"{d['conf']:.2f}", (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_STAGE1, 1)
    path_s1 = os.path.join(args.save_dir, 'stage1_detection.jpg')
    cv2.imwrite(path_s1, vis_s1)
    print(f"\n已保存 Stage 1 结果: {path_s1}")

    # 保存 ROI 裁剪
    roi_dir = os.path.join(args.save_dir, 'rois')
    os.makedirs(roi_dir, exist_ok=True)
    for i, d in enumerate(detections):
        x, y, w, h = d['xywh']
        x1 = max(0, int(x - w / 2))
        y1 = max(0, int(y - h / 2))
        x2 = min(frame.shape[1], int(x + w / 2))
        y2 = min(frame.shape[0], int(y + h / 2))
        roi = frame[y1:y2, x1:x2]
        s2_label = stage2_results[i]['label'] if i < len(stage2_results) else 'NULL'
        roi_path = os.path.join(roi_dir, f'roi_{i}_{s2_label}.jpg')
        cv2.imwrite(roi_path, roi)
    print(f"已保存 {len(detections)} 个 ROI 到: {roi_dir}/")

    # 保存综合结果
    path_all = os.path.join(args.save_dir, 'all_stages_result.jpg')
    cv2.imwrite(path_all, vis)
    print(f"已保存综合结果: {path_all}")

    # 显示
    if args.show:
        cv2.namedWindow("3-Stage Result", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("3-Stage Result", 1280, 720)
        cv2.imshow("3-Stage Result", vis)
        print("\n按任意键关闭窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    print("\n完成！")


if __name__ == '__main__':
    main()

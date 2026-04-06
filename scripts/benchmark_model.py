#!/usr/bin/env python3
"""
benchmark_model.py — YOLO 模型性能基准测试脚本

测量三阶段推理管线的延迟、吞吐量和 GPU 显存占用，
输出结构化报告，方便模型迭代时对比性能变化。

用法:
    # 完整三阶段基准测试（默认 100 轮预热 + 200 轮计时）
    python3 scripts/benchmark_model.py

    # 指定轮次和图片
    python3 scripts/benchmark_model.py --warmup 50 --rounds 100 --image path/to/img.jpg

    # 仅测试 Stage 1
    python3 scripts/benchmark_model.py --stage 1

    # 测试不同推理分辨率
    python3 scripts/benchmark_model.py --infer-size 1280
    python3 scripts/benchmark_model.py --infer-size 1920

    # 保存报告到 JSON
    python3 scripts/benchmark_model.py --output benchmark_result.json

依赖:
    - PyTorch（已安装）
    - 项目内嵌 ultralytics
    - 模型权重文件存在于 data/weights/
"""

import argparse
import json
import os
import statistics
import sys
import time
from datetime import datetime

import cv2
import numpy as np

# ── 路径设置 ──────────────────────────────────────────────
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))

# 将 ultralytics 所在路径加入 sys.path
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src", "hnurm_radar"))
from ultralytics import YOLO

# ── 默认路径 ──────────────────────────────────────────────
DEFAULT_IMAGE = os.path.join(PROJECT_ROOT, "test_resources", "red1.png")
STAGE_ONE_PATH = os.path.join(PROJECT_ROOT, "data", "weights", "stage_one", "stage_one.pt")
STAGE_TWO_PATH = os.path.join(PROJECT_ROOT, "data", "weights", "stage_two", "stage_two.pt")
STAGE_THREE_PATH = os.path.join(PROJECT_ROOT, "data", "weights", "stage_three", "stage_three.pt")


# ================================================================
# GPU 信息采集
# ================================================================

def get_gpu_info():
    """获取 GPU 信息（如果可用）"""
    info = {"available": False}
    try:
        import torch
        if torch.cuda.is_available():
            info["available"] = True
            info["device_name"] = torch.cuda.get_device_name(0)
            info["cuda_version"] = torch.version.cuda or "N/A"
            mem = torch.cuda.get_device_properties(0).total_mem
            info["total_memory_mb"] = round(mem / 1024 / 1024)
        info["torch_version"] = torch.__version__
    except ImportError:
        info["torch_version"] = "N/A"
    return info


def get_gpu_memory_mb():
    """获取当前 GPU 显存占用（MB）"""
    try:
        import torch
        if torch.cuda.is_available():
            return round(torch.cuda.memory_allocated(0) / 1024 / 1024, 1)
    except ImportError:
        pass
    return 0.0


# ================================================================
# 单阶段基准测试
# ================================================================

def benchmark_single_stage(model, frame, rounds, warmup, stage_name,
                           is_track=False, imgsz=1280, verbose=True):
    """
    对单个模型进行基准测试。

    Returns:
        dict: 包含延迟统计的字典
    """
    if verbose:
        print(f"\n{'─' * 50}")
        print(f"  {stage_name}")
        print(f"  推理分辨率: {imgsz} | 预热: {warmup} 轮 | 计时: {rounds} 轮")
        print(f"{'─' * 50}")

    # 预热
    if verbose:
        print("  预热中...", end="", flush=True)
    for i in range(warmup):
        if is_track:
            model.track(frame, persist=True, verbose=False, imgsz=imgsz)
        else:
            model(frame, verbose=False, imgsz=imgsz)
    if verbose:
        print(" 完成")

    # 记录显存（预热后）
    mem_after_warmup = get_gpu_memory_mb()

    # 计时轮次
    latencies = []
    if verbose:
        print("  计时中...", end="", flush=True)

    for i in range(rounds):
        t0 = time.perf_counter()
        if is_track:
            model.track(frame, persist=True, verbose=False, imgsz=imgsz)
        else:
            model(frame, verbose=False, imgsz=imgsz)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000)  # ms

    if verbose:
        print(" 完成")

    # 统计
    lat_mean = statistics.mean(latencies)
    lat_median = statistics.median(latencies)
    lat_std = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
    lat_min = min(latencies)
    lat_max = max(latencies)
    lat_p95 = sorted(latencies)[int(0.95 * len(latencies))]
    lat_p99 = sorted(latencies)[int(0.99 * len(latencies))]
    fps = 1000.0 / lat_mean if lat_mean > 0 else 0

    result = {
        "stage": stage_name,
        "rounds": rounds,
        "warmup": warmup,
        "imgsz": imgsz,
        "latency_ms": {
            "mean": round(lat_mean, 2),
            "median": round(lat_median, 2),
            "std": round(lat_std, 2),
            "min": round(lat_min, 2),
            "max": round(lat_max, 2),
            "p95": round(lat_p95, 2),
            "p99": round(lat_p99, 2),
        },
        "fps": round(fps, 1),
        "gpu_memory_mb": mem_after_warmup,
    }

    if verbose:
        print("\n  结果:")
        print(f"    平均延迟:  {lat_mean:7.2f} ms")
        print(f"    中位延迟:  {lat_median:7.2f} ms")
        print(f"    标准差:    {lat_std:7.2f} ms")
        print(f"    最小/最大: {lat_min:.2f} / {lat_max:.2f} ms")
        print(f"    P95/P99:   {lat_p95:.2f} / {lat_p99:.2f} ms")
        print(f"    吞吐量:    {fps:.1f} FPS")
        print(f"    GPU 显存:  {mem_after_warmup:.1f} MB")

    return result


def benchmark_stage2_batch(model, frame, detections, rounds, warmup,
                           imgsz=256, verbose=True):
    """
    Stage 2/3 基准测试 — 模拟真实场景的 ROI 批量分类。

    使用 Stage 1 的检测结果裁剪 ROI 后逐个推理（当前项目的实际逻辑）。
    """
    stage_name = "Stage 2 (装甲板分类 - 批量 ROI)"

    if not detections:
        if verbose:
            print(f"\n  {stage_name}: 无检测结果，跳过")
        return {"stage": stage_name, "skipped": True, "reason": "no detections"}

    # 预先裁剪所有 ROI
    rois = []
    h, w = frame.shape[:2]
    for d in detections:
        x, y, bw, bh = d["xywh"]
        x1 = max(0, int(x - bw / 2))
        y1 = max(0, int(y - bh / 2))
        x2 = min(w, int(x + bw / 2))
        y2 = min(h, int(y + bh / 2))
        roi = frame[y1:y2, x1:x2]
        if roi.size > 0:
            rois.append(roi)

    n_rois = len(rois)
    if verbose:
        print(f"\n{'─' * 50}")
        print(f"  {stage_name}")
        print(f"  ROI 数量: {n_rois} | 推理分辨率: {imgsz}")
        print(f"  预热: {warmup} 轮 | 计时: {rounds} 轮")
        print(f"{'─' * 50}")

    # 预热
    if verbose:
        print("  预热中...", end="", flush=True)
    for _ in range(warmup):
        for roi in rois:
            model(roi, verbose=False, imgsz=imgsz)
    if verbose:
        print(" 完成")

    mem_after_warmup = get_gpu_memory_mb()

    # 计时：每轮处理全部 ROI
    latencies = []
    if verbose:
        print("  计时中...", end="", flush=True)
    for _ in range(rounds):
        t0 = time.perf_counter()
        for roi in rois:
            model(roi, verbose=False, imgsz=imgsz)
        t1 = time.perf_counter()
        latencies.append((t1 - t0) * 1000)
    if verbose:
        print(" 完成")

    lat_mean = statistics.mean(latencies)
    lat_median = statistics.median(latencies)
    lat_std = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
    per_roi = lat_mean / n_rois if n_rois > 0 else 0
    fps = 1000.0 / lat_mean if lat_mean > 0 else 0

    result = {
        "stage": stage_name,
        "rounds": rounds,
        "warmup": warmup,
        "imgsz": imgsz,
        "n_rois": n_rois,
        "latency_ms": {
            "mean_total": round(lat_mean, 2),
            "median_total": round(lat_median, 2),
            "std": round(lat_std, 2),
            "per_roi": round(per_roi, 2),
        },
        "fps": round(fps, 1),
        "gpu_memory_mb": mem_after_warmup,
    }

    if verbose:
        print(f"\n  结果 ({n_rois} 个 ROI):")
        print(f"    总延迟:    {lat_mean:7.2f} ms")
        print(f"    每 ROI:    {per_roi:7.2f} ms")
        print(f"    帧吞吐量:  {fps:.1f} FPS")
        print(f"    GPU 显存:  {mem_after_warmup:.1f} MB")

    return result


# ================================================================
# 主流程
# ================================================================

def run_stage1_detect(model, frame, imgsz=1280):
    """运行 Stage 1 并返回检测结果列表"""
    results = model.track(frame, persist=True, verbose=False, imgsz=imgsz)
    detections = []
    if results and results[0].boxes is not None:
        boxes = results[0].boxes
        for i in range(len(boxes)):
            xywh = boxes.xywh[i].cpu().numpy().tolist()
            conf = float(boxes.conf[i])
            detections.append({"xywh": xywh, "conf": conf})
    return detections


def main():
    parser = argparse.ArgumentParser(
        description="YOLO 三阶段推理管线性能基准测试",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  python3 scripts/benchmark_model.py                      # 完整基准测试
  python3 scripts/benchmark_model.py --stage 1            # 仅 Stage 1
  python3 scripts/benchmark_model.py --rounds 500         # 500 轮计时
  python3 scripts/benchmark_model.py --infer-size 1280    # 1280 分辨率
  python3 scripts/benchmark_model.py --output result.json # 保存 JSON 报告
""",
    )
    parser.add_argument("--image", default=DEFAULT_IMAGE,
                        help="测试图片路径 (默认: test_resources/red1.png)")
    parser.add_argument("--stage", type=int, choices=[1, 2, 3], default=None,
                        help="仅测试指定阶段 (1/2/3)，默认测试全部")
    parser.add_argument("--warmup", type=int, default=100,
                        help="预热轮次 (默认: 100)")
    parser.add_argument("--rounds", type=int, default=200,
                        help="计时轮次 (默认: 200)")
    parser.add_argument("--infer-size", type=int, default=1920,
                        help="Stage 1 推理分辨率 (默认: 1920)")
    parser.add_argument("--roi-size", type=int, default=256,
                        help="Stage 2/3 ROI 推理分辨率 (默认: 256)")
    parser.add_argument("--output", type=str, default=None,
                        help="保存 JSON 报告到指定路径")
    parser.add_argument("--no-track", action="store_true",
                        help="Stage 1 使用 detect 而非 track 模式")
    args = parser.parse_args()

    # ── 环境信息 ──
    gpu_info = get_gpu_info()
    print("=" * 60)
    print("  YOLO 三阶段推理管线 · 性能基准测试")
    print("=" * 60)
    print(f"  时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print(f"  PyTorch: {gpu_info.get('torch_version', 'N/A')}")
    if gpu_info["available"]:
        print(f"  GPU: {gpu_info['device_name']}")
        print(f"  CUDA: {gpu_info['cuda_version']}")
        print(f"  显存: {gpu_info['total_memory_mb']} MB")
    else:
        print("  GPU: 不可用（使用 CPU 推理）")

    # ── 读取图片 ──
    print(f"\n  输入图片: {args.image}")
    img = cv2.imread(args.image)
    if img is None:
        print(f"  错误: 无法读取图片 {args.image}")
        sys.exit(1)

    h, w = img.shape[:2]
    infer_w = args.infer_size
    infer_h = int(h * infer_w / w)
    frame = cv2.resize(img, (infer_w, infer_h))
    print(f"  原始尺寸: {w}x{h} → 推理尺寸: {infer_w}x{infer_h}")

    # ── 模型检查 ──
    stages_to_test = [args.stage] if args.stage else [1, 2, 3]
    model_paths = {
        1: STAGE_ONE_PATH,
        2: STAGE_TWO_PATH,
        3: STAGE_THREE_PATH,
    }
    for s in stages_to_test:
        p = model_paths[s]
        if not os.path.isfile(p):
            print(f"\n  ⚠ Stage {s} 模型文件不存在: {p}")
            print("    请确保模型权重已放置在正确位置。")
            sys.exit(1)

    # ── 收集结果 ──
    results = {
        "timestamp": datetime.now().isoformat(),
        "gpu_info": gpu_info,
        "image": args.image,
        "image_size": f"{infer_w}x{infer_h}",
        "warmup": args.warmup,
        "rounds": args.rounds,
        "stages": [],
    }

    mem_before = get_gpu_memory_mb()
    results["gpu_memory_before_mb"] = mem_before

    # ── Stage 1 ──
    detections = []
    if 1 in stages_to_test:
        print(f"\n  加载 Stage 1 模型: {STAGE_ONE_PATH}")
        model1 = YOLO(STAGE_ONE_PATH, task="detect")
        model1.overrides["imgsz"] = args.infer_size

        r1 = benchmark_single_stage(
            model1, frame,
            rounds=args.rounds,
            warmup=args.warmup,
            stage_name="Stage 1 (目标检测+追踪)",
            is_track=not args.no_track,
            imgsz=args.infer_size,
        )
        results["stages"].append(r1)

        # 获取检测结果供 Stage 2/3 使用
        detections = run_stage1_detect(model1, frame, imgsz=args.infer_size)
        r1["detections_count"] = len(detections)
        print(f"\n  检测到 {len(detections)} 个目标（供 Stage 2/3 使用）")

    # ── Stage 2 ──
    if 2 in stages_to_test:
        print(f"\n  加载 Stage 2 模型: {STAGE_TWO_PATH}")
        model2 = YOLO(STAGE_TWO_PATH)
        model2.overrides["imgsz"] = args.roi_size

        if detections:
            r2 = benchmark_stage2_batch(
                model2, frame, detections,
                rounds=args.rounds,
                warmup=args.warmup,
                imgsz=args.roi_size,
            )
        else:
            # 没有检测结果时，使用合成 ROI 测试单帧延迟
            print("  （无 Stage 1 检测结果，使用合成 256x256 ROI）")
            fake_roi = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
            r2 = benchmark_single_stage(
                model2, fake_roi,
                rounds=args.rounds,
                warmup=args.warmup,
                stage_name="Stage 2 (装甲板分类 - 合成 ROI)",
                is_track=False,
                imgsz=args.roi_size,
            )
        results["stages"].append(r2)

    # ── Stage 3 ──
    if 3 in stages_to_test:
        print(f"\n  加载 Stage 3 模型: {STAGE_THREE_PATH}")
        model3 = YOLO(STAGE_THREE_PATH)
        model3.overrides["imgsz"] = args.roi_size

        if detections:
            r3 = benchmark_stage2_batch(
                model3, frame, detections,
                rounds=args.rounds,
                warmup=args.warmup,
                imgsz=args.roi_size,
            )
            r3["stage"] = "Stage 3 (灰色装甲板 - 批量 ROI)"
        else:
            fake_roi = np.random.randint(0, 255, (256, 256, 3), dtype=np.uint8)
            r3 = benchmark_single_stage(
                model3, fake_roi,
                rounds=args.rounds,
                warmup=args.warmup,
                stage_name="Stage 3 (灰色装甲板 - 合成 ROI)",
                is_track=False,
                imgsz=args.roi_size,
            )
        results["stages"].append(r3)

    # ── 总结 ──
    mem_after = get_gpu_memory_mb()
    results["gpu_memory_after_mb"] = mem_after

    print("\n" + "=" * 60)
    print("  基准测试总结")
    print("=" * 60)
    total_latency = 0
    for s in results["stages"]:
        if s.get("skipped"):
            print(f"  {s['stage']}: 跳过 ({s.get('reason', '')})")
            continue
        lat = s["latency_ms"]
        mean_val = lat.get("mean", lat.get("mean_total", 0))
        total_latency += mean_val
        fps_val = s.get("fps", 0)
        print(f"  {s['stage']}:")
        print(f"    延迟 {mean_val:.2f} ms | {fps_val:.1f} FPS")

    if total_latency > 0:
        total_fps = 1000.0 / total_latency
        print(f"\n  三阶段总延迟: {total_latency:.2f} ms ({total_fps:.1f} FPS)")
        results["total_latency_ms"] = round(total_latency, 2)
        results["total_fps"] = round(total_fps, 1)

    print(f"  GPU 显存: {mem_before:.1f} → {mem_after:.1f} MB")
    print("=" * 60)

    # ── 保存 JSON 报告 ──
    if args.output:
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as f:
            json.dump(results, f, indent=2, ensure_ascii=False)
        print(f"\n  报告已保存: {args.output}")

    return results


if __name__ == "__main__":
    main()

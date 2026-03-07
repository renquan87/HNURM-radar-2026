#!/usr/bin/env python3
# pyright: reportMissingImports=false
"""空中目标参数自动调优：遍历 eps/min_samples/高度范围。"""

import argparse
import glob
import os
import sys
import numpy as np
import open3d as o3d
from itertools import product
from ruamel.yaml import YAML

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src", "hnurm_radar")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)

from hnurm_radar.air_scheme.air_config import load_air_target_config
from hnurm_radar.air_scheme.point_cloud_processor import AirPointCloudProcessor
from hnurm_radar.air_scheme.cluster_detector import AirClusterDetector


def parse_list(s, cast=float):
    return [cast(x.strip()) for x in s.split(",") if x.strip()]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--pcd_dir", required=True)
    p.add_argument("--glob", default="*.pcd")
    p.add_argument("--config", default=os.path.join(PROJECT_ROOT, "configs", "main_config.yaml"))
    p.add_argument("--eps", default="0.25,0.30,0.35")
    p.add_argument("--min_samples", default="4,5,6")
    p.add_argument("--z_min", default="-1.7,-1.5,-1.3")
    p.add_argument("--z_max", default="-0.7,-0.5,-0.3")
    p.add_argument("--topk", type=int, default=10)
    return p.parse_args()


def evaluate(files, air_cfg):
    processor = AirPointCloudProcessor(air_cfg.preprocessing)
    detector = AirClusterDetector(air_cfg.clustering, air_cfg.target_filter)

    counts = []
    for fp in files:
        pcd = o3d.io.read_point_cloud(fp)
        pcd = processor.crop_roi(pcd)
        pcd = processor.downsample(pcd)
        pcd = processor.filter_height(pcd)
        targets = detector.detect(pcd)
        counts.append(len(targets))

    avg_count = float(np.mean(counts)) if counts else 0.0
    var_count = float(np.var(counts)) if counts else 1e9
    score = avg_count - 0.3 * np.sqrt(var_count)
    return score, avg_count, var_count


def main():
    args = parse_args()
    yaml = YAML(typ="safe")
    with open(args.config, "r", encoding="utf-8") as f:
        cfg = yaml.load(f)

    air_cfg = load_air_target_config(cfg)
    files = sorted(glob.glob(os.path.join(args.pcd_dir, args.glob)))
    if not files:
        raise RuntimeError("未找到输入点云序列")

    eps_list = parse_list(args.eps, float)
    min_samples_list = parse_list(args.min_samples, int)
    z_min_list = parse_list(args.z_min, float)
    z_max_list = parse_list(args.z_max, float)

    results = []
    for eps, ms, zmin, zmax in product(eps_list, min_samples_list, z_min_list, z_max_list):
        if zmin >= zmax:
            continue
        air_cfg.clustering.eps = eps
        air_cfg.clustering.min_samples = ms
        air_cfg.preprocessing.height_filter.z_min = zmin
        air_cfg.preprocessing.height_filter.z_max = zmax

        score, avg_count, var_count = evaluate(files, air_cfg)
        results.append((score, eps, ms, zmin, zmax, avg_count, var_count))
        print(f"score={score:.3f} eps={eps} ms={ms} z=[{zmin},{zmax}] avg={avg_count:.2f} var={var_count:.2f}")

    results.sort(key=lambda x: x[0], reverse=True)
    print("\n==== Top Results ====")
    for row in results[: args.topk]:
        score, eps, ms, zmin, zmax, avg_count, var_count = row
        print(f"score={score:.3f} eps={eps} ms={ms} z=[{zmin},{zmax}] avg={avg_count:.2f} var={var_count:.2f}")


if __name__ == "__main__":
    main()

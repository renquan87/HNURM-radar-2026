# HNURM Radar 2026

RoboMaster 2026 雷达站项目，包含三种方案：地面方案一（纯相机）、地面方案二（相机+激光雷达）、空中方案（纯激光雷达）。

---

## 三种方案总览

| 维度 | 方案一（纯相机） | 方案二（相机+雷达） | 空中方案（纯雷达） |
|------|:---------------:|:------------------:|:-----------------:|
| **目标类型** | 地面机器人 | 地面机器人 | 空中无人机 |
| **硬件需求** | 仅相机（海康/USB） | 相机 + Livox HAP | 仅 Livox HAP |
| **定位方式** | 透视变换 2D→2D | 点云投影+聚类 2D→3D | 背景减除+DBSCAN 3D |
| **核心节点** | `camera_detector` | `lidar_node` + `detector_node` + `radar_node` | `lidar_node` + `air_target_node` |
| **Launch 文件** | 无专用（手动启动） | `hnurm_radar_launch.py` | `hnurm_air_launch.py` |

三种方案共享下游流程：`→ ekf_node（卡尔曼滤波）→ judge_messager（裁判系统通信）→ display_panel（可视化）`

---

## 快速开始

👉 **新手请先阅读 [快速开始指南](docs/guides/quickstart.md)**，15 分钟内完成首次运行。

### 最简启动命令

```bash
# ===================== 通用前置 =====================
source /data/venv/radar-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# =================== 方案二（主力方案，一键启动）====================
bash bringup.sh
# ⚠ 启动后必须在 RViz 中给 2D Pose Estimate！
```

其他方案的启动方式详见 [快速开始指南](docs/guides/quickstart.md)。

---

## 项目结构

```
hnurm_radar/
├── configs/              # 配置文件（→ configs/README.md）
├── data/                 # 数据资产（地图、权重、点云）
│   └── weights/          # 模型权重（→ data/weights/README.md）
├── docs/                 # 项目文档（→ docs/README.md）
├── scripts/              # 辅助工具脚本
├── src/
│   ├── hnurm_radar/      # 主 ROS2 包
│   │   └── hnurm_radar/
│   │       ├── camera_scheme/   # 方案一：纯相机
│   │       ├── lidar_scheme/    # 方案二：相机+激光雷达
│   │       ├── air_scheme/      # 空中方案：纯激光雷达
│   │       ├── communication/   # 裁判系统通信
│   │       ├── detection/       # 通用检测模块
│   │       ├── core/            # 核心抽象层
│   │       └── shared/          # 共享工具
│   ├── ekf/              # 扩展卡尔曼滤波
│   ├── hnurm_bringup/    # Launch 文件
│   └── registration/     # ICP 点云配准
├── tests/                # 测试
└── requirements.txt      # Python 依赖
```

---

## 文档导航

| 文档 | 说明 |
|------|------|
| 📖 [文档总览](docs/README.md) | 所有文档索引 |
| 🚀 [快速开始](docs/guides/quickstart.md) | 新手 15 分钟上手 |
| 🏗️ [系统架构](docs/architecture/system_overview.md) | 架构图与数据流 |
| 📡 [ROS2 话题清单](docs/api/ros_topics.md) | 节点、话题、消息类型 |
| ⚙️ [配置文件说明](configs/README.md) | 配置项参考 |
| 🧠 [开发计划](docs/OPTIMIZATION_PLAN.md) | 四阶段优化路线图 |
| 📚 [学习路线](docs/guides/LEARNING_ROADMAP.md) | 技术栈学习指南 |

---

## 关键配置

所有配置集中在 [`configs/main_config.yaml`](configs/main_config.yaml)，主要段落：

| 配置段 | 控制内容 |
|--------|---------:|
| `global` | 颜色、调试模式、场景选择 |
| `scenes` | 赛场/实验室场景切换（地图、PCD 路径） |
| `camera` | 相机输入模式（海康/USB/测试图片） |
| `lidar` | 激光雷达过滤参数、话题名 |
| `communication` | 串口设备路径、波特率 |
| `air_target` | 空中方案全部参数 |

---

## 依赖安装

```bash
# Livox SDK 依赖
sudo apt install libpkgconf libpcap-dev

# Python 依赖
pip install -r requirements.txt
```

---

## 辅助工具脚本

| 脚本 | 用途 |
|------|------|
| `scripts/test_yolo_3stage.py` | YOLO 三阶段推理效果测试 |
| `scripts/benchmark_model.py` | 模型推理性能基准测试 |
| `scripts/export_models.py` | 模型导出（PyTorch→TensorRT） |
| `scripts/align_pcd_with_map.py` | 交互式点云-地图对齐 |
| `scripts/calibrate_lab_map.py` | 实验室地图标定 |
| `scripts/lidar_collect_background.py` | 背景点云采集 |
| `scripts/air_param_grid_search.py` | 空中方案 DBSCAN 参数调优 |

完整脚本说明见 [快速开始 → 辅助工具](docs/guides/quickstart.md#辅助工具脚本)。

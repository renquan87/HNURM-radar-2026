# HNURM Radar 2026 — 项目结构规范文档

> 本文档对项目的整体架构、目录组织、ROS2 包划分、节点通信关系、配置文件约定等进行全面说明。
> 文档仅做分析与规范描述，不改变代码功能。

---

## 1. 项目概述

本项目是 RoboMaster 2026 赛季雷达站系统，基于 ROS2 (Humble) 构建，运行于 Ubuntu 22.04 / Linux 环境。系统核心功能：

- 通过海康工业相机采集赛场图像
- 使用 YOLO 三阶段推理（检测 → 颜色分类 → 灰色装甲板分类）识别敌方机器人
- 支持两种定位方案：
  - **相机 + 激光雷达方案**（`detector_node` + `lidar_node`）：通过 Livox 激光雷达点云与相机外参联合解算三维坐标
  - **纯相机透视变换方案**（`camera_detector`）：通过 Homography 矩阵将像素坐标映射到赛场坐标
- EKF / 卡尔曼滤波平滑坐标
- 通过串口与裁判系统通信，发送敌方机器人位置和双倍易伤指令
- 小地图实时可视化

---

## 2. 顶层目录结构

```
hnurm_radar/                    # 项目根目录（ROS2 工作空间）
├── bringup.sh                  # 一键编译 + 启动脚本
├── requirements.txt            # Python 依赖列表
├── README.md                   # 项目原始说明
├── LICENSE                     # 开源许可证
├── .gitignore                  # Git 忽略规则
├── .gitattributes              # Git 属性配置
│
├── configs/                    # 全局配置文件目录（含 RViz 配置）
├── docs/                       # 项目文档
├── map/                        # 赛场地图图片
├── weights/                    # YOLO 模型权重文件
├── scripts/                    # 独立工具脚本
├── test_resources/             # 测试用图片资源
│
├── build/                      # colcon 编译产物（.gitignore 排除）
├── install/                    # colcon 安装产物（.gitignore 排除）
├── log/                        # colcon 编译日志（.gitignore 排除）
├── record/                     # 运行时录制数据（.gitignore 排除）
├── MvFGSdkLog/                 # 海康相机 SDK 日志（.gitignore 排除）
│
└── src/                        # ROS2 功能包源码目录
    ├── hnurm_radar/            # 【核心包】雷达站主功能
    ├── hnurm_bringup/          # 【启动包】launch 文件
    ├── detect_result/          # 【消息包】自定义 ROS2 消息
    ├── ekf/                    # 【滤波包】扩展卡尔曼滤波节点
    ├── registration/           # 【配准包】点云配准节点（ICP/Quatro）
    ├── livox_ros_driver2/      # 【第三方】Livox 激光雷达 ROS2 驱动
    ├── Livox-SDK2/             # 【第三方】Livox SDK
    └── Quatro/                 # 【第三方】Quatro 鲁棒配准算法库
```

---

## 3. ROS2 功能包详解

### 3.1 `hnurm_radar`（核心包，Python）

项目的核心功能包，包含所有检测、定位、通信、可视化逻辑。

```
src/hnurm_radar/
├── package.xml                 # ROS2 包描述
├── setup.py                    # Python 包安装配置（定义所有节点入口）
├── setup.cfg                   # setuptools 配置
├── resource/hnurm_radar        # ament 资源索引标记文件
├── test/                       # 单元测试
│
├── hnurm_radar/                # Python 包主目录
│   ├── __init__.py
│   │
│   ├── camera_scheme/          # ★ 方案一：纯相机透视变换方案
│   │   ├── __init__.py
│   │   └── camera_detector.py  # 【节点】纯相机透视变换检测节点
│   │
│   ├── lidar_scheme/           # ★ 方案二：相机 + 激光雷达方案
│   │   ├── __init__.py
│   │   ├── detector_node.py    # 【节点】YOLO 三阶段检测节点
│   │   ├── lidar_node.py       # 【节点】激光雷达数据接收与处理节点
│   │   └── radar.py            # 【节点】相机+雷达联合定位主节点
│   │
│   ├── shared/                 # ★ 两种方案共享的节点
│   │   ├── __init__.py
│   │   ├── judge_messager.py   # 【节点】裁判系统串口通信节点
│   │   ├── display_panel.py    # 【节点】操作手显示面板节点
│   │   └── publish_video.py    # 【节点】视频流发布节点
│   │
│   ├── Camera/                 # 相机驱动封装（共享模块）
│   │   └── HKCam.py            # 海康工业相机 SDK 封装
│   │
│   ├── Car/                    # 车辆数据模型（共享模块）
│   │   └── Car.py              # Car / CarList 类（车辆信息管理）
│   │
│   ├── Lidar/                  # 激光雷达处理模块（lidar_scheme 使用）
│   │   └── Converter.py        # 坐标系转换器（雷达↔相机↔赛场）
│   │
│   ├── camera_locator/         # 透视变换标定工具（camera_scheme 使用）
│   │   ├── perspective_calibrator.py  # GUI 标定工具（PyQt5）
│   │   ├── anchor.py           # 标定点数据结构
│   │   └── point_picker.py     # 图像点选取工具
│   │
│   ├── filters/                # 滤波器模块（camera_scheme 使用）
│   │   └── kalman_filter.py    # 增强型卡尔曼滤波器（坐标平滑）
│   │
│   └── Tools/                  # 通用工具函数
│
└── ultralytics/                # 内嵌的 Ultralytics YOLO 库（本地修改版）
```

**节点入口点**（定义在 `setup.py`）：

| 入口名称 | 对应模块 | 说明 |
|---|---|---|
| `camera_detector` | `camera_scheme.camera_detector:main` | 方案一：纯相机透视变换检测节点 |
| `detector_node` | `lidar_scheme.detector_node:main` | 方案二：YOLO 三阶段检测节点 |
| `radar_node` | `lidar_scheme.radar:main` | 方案二：相机+雷达联合定位主节点 |
| `lidar_node` | `lidar_scheme.lidar_node:main` | 方案二：激光雷达数据处理节点 |
| `display_panel` | `shared.display_panel:main` | 共享：操作手显示面板 |
| `judge_messager` | `shared.judge_messager:main` | 共享：裁判系统通信节点 |
| `publish_video` | `shared.publish_video:main` | 共享：视频流发布节点 |
| `perspective_calibrator` | `camera_locator.perspective_calibrator:main` | 工具：透视变换标定 |
| `make_mask` | `camera_locator.make_mask:main` | 工具：分区掩码制作 |

### 3.2 `hnurm_bringup`（启动包，CMake）

仅包含 launch 文件，用于一键启动多个节点。

| Launch 文件 | 说明 | 启动的节点 |
|---|---|---|
| `hnurm_radar_launch.py` | 相机+雷达方案（主 launch） | lidar_node, detector_node, radar_node, display_panel, judge_messager, ekf_node |
| `hnurm_radar_video_launch.py` | 相机+雷达方案（视频回放调试） | lidar_node, publish_video, detector_node, radar_node |

> 注意：`livox_ros_driver2` 和 `registration` 不在 launch 文件中，而是由 `bringup.sh` 在独立终端窗口中分别启动。纯相机方案（`camera_detector`）目前没有专用 launch 文件，需手动启动各节点。

### 3.3 `detect_result`（消息包，CMake）

| 消息文件 | 用途 |
|---|---|
| `DetectResult.msg` | 单个检测结果（框、ID、标签） |
| `Robots.msg` | 检测结果数组 |
| `Location.msg` | 单个机器人赛场坐标 |
| `Locations.msg` | 坐标数组 |

### 3.4 `ekf`（滤波包，Python）

扩展卡尔曼滤波节点。订阅 `location` → 发布 `ekf_location_filtered`。

### 3.5 `registration`（配准包，C++）

点云配准节点（Quatro 算法），用于 Lidar-to-Map 对齐。

### 3.6 第三方包

| 包名 | 说明 |
|---|---|
| `livox_ros_driver2` | Livox 激光雷达 ROS2 驱动 |
| `Livox-SDK2` | Livox 硬件通信 SDK |
| `Quatro` | 鲁棒点云配准算法库 |

---

## 4. 配置文件规范

所有配置文件统一存放在 `configs/` 目录下。

| 文件 | 格式 | 说明 |
|---|---|---|
| `main_config.yaml` | YAML | 全局主配置（颜色、调试开关、相机模式等） |
| `detector_config.yaml` | YAML | YOLO 检测参数（模型路径、置信度阈值等） |
| `converter_config.yaml` | YAML | 坐标转换参数（内外参、滤波、聚类参数） |
| `bytetrack.yaml` | YAML | ByteTrack 追踪器配置 |
| `HAP_config.json` | JSON | Livox HAP 激光雷达连接配置 |
| `perspective_calib.json` | JSON | 透视变换标定结果（由标定工具生成） |
| `bin_cam_config.yaml` | YAML | 双目相机配置（预留） |
| `icp.rviz` | RViz | ICP 点云配准可视化配置 |

**路径约定**：代码中所有路径通过 `src/hnurm_radar/hnurm_radar/shared/paths.py` 基于项目根目录动态计算，配置文件使用相对路径，项目可部署到任意目录。

---

## 5. ROS2 话题通信关系

```
┌─────────────────┐     location      ┌──────────┐  ekf_location_filtered  ┌────────────────┐
│  detector_node   │ ───────────────→ │ ekf_node │ ─────────────────────→ │ judge_messager │
│  (或 camera_     │                  └──────────┘                        │  (串口发送)     │
│   detector)      │                                                      └────────────────┘
│                  │     detect_view
│                  │ ───────────────→  display_panel (可视化)
└─────────────────┘

┌──────────────────┐   /livox/lidar   ┌────────────────────┐
│ livox_ros_driver2│ ──────────────→  │ registration_node  │ ──→ TF ──→ lidar_node
└──────────────────┘                  └────────────────────┘
```

---

## 6. 资源文件

### 模型权重（`weights/`）

| 文件 | 用途 |
|---|---|
| `stage_one.pt` | 第一阶段：车辆检测（YOLO，1280px） |
| `stage_two.pt` | 第二阶段：装甲板颜色+编号分类（256px） |
| `stage_three.pt` | 第三阶段：灰色装甲板分类（256px） |

### 地图资源（`map/`）

| 文件 | 说明 |
|---|---|
| `pfa_map_2025.jpg` | 赛场地图（2800×1500，100px/m） |
| `pfa_map_mask_2025.jpg` | 分区掩码（黑色=地面，非黑色=高地） |
| `pfa_map_blue_2025.jpg` / `pfa_map_red_2025.jpg` | 红蓝方视角地图 |

### 工具脚本（`scripts/`）

| 文件 | 说明 |
|---|---|
| `check_points.py` | 检查点工具脚本 |

---

## 7. 编码规范建议

以下为改进建议（仅建议，不改变现有代码）：

- **模块目录命名**：当前混用 `PascalCase`（`Camera/`, `Car/`, `Lidar/`）和 `snake_case`（`camera_locator/`, `filters/`），建议统一为 `snake_case`
- **配置路径**：当前使用硬编码绝对路径，建议后续改为 ROS2 参数或环境变量
- **日志**：非 ROS2 模块使用 `print()`，建议统一为 Python `logging` 模块

---

## 8. .gitignore 规则

以下内容被排除在版本控制之外：

| 模式 | 说明 |
|---|---|
| `build/`, `install/`, `log/` | colcon 编译产物 |
| `src/*/build/`, `src/*/install/`, `src/*/log/` | 包内嵌套编译产物 |
| `__pycache__/`, `*.pyc` | Python 缓存 |
| `*.engine`, `*.onnx` | 模型转换文件 |
| `*.pcd`, `target.xyz` | 点云数据文件 |
| `MvFGSdkLog/`, `record/` | 运行时生成 |
| `.vscode/`, `.idea/` | IDE 配置 |

---

*文档版本: 2026-02-14*

# HNURM Radar 2026 — 源码详细说明文档

> 本文档对 `src/` 目录下所有 ROS2 功能包的代码结构、类设计、核心算法、数据流进行详细说明。

---

## 目录

1. [hnurm_radar — 核心功能包](#1-hnurm_radar--核心功能包)
   - 1.1 [camera_scheme — 纯相机方案](#11-camera_scheme--纯相机方案)
   - 1.2 [lidar_scheme — 相机+激光雷达方案](#12-lidar_scheme--相机激光雷达方案)
   - 1.3 [shared — 共享节点](#13-shared--共享节点)
   - 1.4 [Camera — 相机驱动封装](#14-camera--相机驱动封装)
   - 1.5 [Car — 车辆数据模型](#15-car--车辆数据模型)
   - 1.6 [Lidar — 坐标转换模块](#16-lidar--坐标转换模块)
   - 1.7 [camera_locator — 透视变换标定工具](#17-camera_locator--透视变换标定工具)
   - 1.8 [filters — 滤波器模块](#18-filters--滤波器模块)
   - 1.9 [Tools — 通用工具](#19-tools--通用工具)
   - 1.10 [ultralytics — 内嵌 YOLO 库](#110-ultralytics--内嵌-yolo-库)
2. [hnurm_bringup — 启动包](#2-hnurm_bringup--启动包)
3. [detect_result — 自定义消息包](#3-detect_result--自定义消息包)
4. [ekf — 扩展卡尔曼滤波包](#4-ekf--扩展卡尔曼滤波包)
5. [registration — 点云配准包](#5-registration--点云配准包)
6. [第三方包](#6-第三方包)
7. [数据流总览](#7-数据流总览)

---

## 1. hnurm_radar — 核心功能包

包类型：Python（ament_python），位于 `src/hnurm_radar/`。

### 1.1 camera_scheme — 纯相机方案

路径：`hnurm_radar/camera_scheme/camera_detector.py`

#### CameraDetector（ROS2 节点）

当前主力方案。不依赖激光雷达，仅通过相机图像 + 透视变换（Homography）完成从像素坐标到赛场坐标的映射。

**核心流程**：

```
相机取帧 → YOLO 三阶段推理 → ByteTrack 追踪 → 投票确定车辆 ID
    → 透视变换（像素→赛场坐标）→ 卡尔曼滤波平滑 → 发布 location 话题
    → 同时发布 detect_view 可视化图像
```

**关键类与方法**：

| 类/方法 | 说明 |
|---|---|
| `CameraDetector(Node)` | ROS2 节点，继承自 `rclpy.node.Node` |
| `__init__()` | 加载配置、初始化相机、加载 YOLO 模型（三阶段）、加载透视变换标定、初始化 ByteTrack 追踪器、创建 ROS2 发布者 |
| `detect_callback()` | 定时器回调（主循环），执行完整的检测-定位-发布流程 |
| `three_stage_detect(frame)` | 三阶段 YOLO 推理：stage1 检测车辆 → stage2 颜色+编号分类 → stage3 灰色装甲板分类 |
| `perspective_transform(pixel_xy)` | 使用 Homography 矩阵将像素坐标映射到赛场坐标，支持地面层和高地层两套矩阵 |
| `vote_system()` | 追踪器投票机制：每个 track_id 对多帧检测结果投票，取票数最高的标签作为最终 ID |

**三阶段推理详解**：

| 阶段 | 模型 | 输入 | 输出 | 分辨率 |
|---|---|---|---|---|
| Stage 1 | `stage_one.pt` | 全图 | 车辆检测框 (bbox) | 1280px |
| Stage 2 | `stage_two.pt` | 裁剪的车辆区域 | 颜色+编号标签（如 R1, B3） | 256px |
| Stage 3 | `stage_three.pt` | 裁剪的车辆区域 | 灰色装甲板编号（无法判断颜色时使用） | 256px |

**透视变换**：
- 使用 `cv2.findHomography()` 计算 Homography 矩阵
- 标定数据来自 `configs/perspective_calib.json`（由标定工具生成）
- 支持两层标定：地面层（ground）和高地层（highland）
- 通过 `pfa_map_mask_2025.jpg` 掩码图判断目标所在区域，选择对应的 Homography 矩阵

**卡尔曼滤波**：
- 使用 `filters/kalman_filter.py` 中的增强型卡尔曼滤波器
- 对每个追踪到的车辆维护独立的滤波器实例
- 支持坐标跳变检测（超过阈值时重置滤波器）
- 超时未更新的滤波器自动清理

---

### 1.2 lidar_scheme — 相机+激光雷达方案

#### detector_node.py — DetectorNode（ROS2 节点）

路径：`hnurm_radar/lidar_scheme/detector_node.py`

与 camera_scheme 类似的 YOLO 三阶段检测，但定位方式不同：将检测框投影到激光雷达点云上，通过点云聚类获取三维坐标。

**核心流程**：

```
相机取帧 → YOLO 三阶段推理 → ByteTrack 追踪 → 投票确定车辆 ID
    → 将检测框映射到点云空间 → DBSCAN 聚类 → 取最大簇中心作为三维坐标
    → 通过 Converter 转换到赛场坐标系 → 发布 location 话题
```

**与 camera_scheme 的区别**：

| 特性 | camera_scheme | lidar_scheme |
|---|---|---|
| 定位方式 | 透视变换（2D→2D） | 点云投影+聚类（2D→3D） |
| 硬件依赖 | 仅相机 | 相机 + Livox HAP 激光雷达 |
| 精度 | 依赖标定质量 | 直接三维测量，理论精度更高 |
| 额外节点 | 无 | 需要 lidar_node、registration |

**关键方法**：

| 方法 | 说明 |
|---|---|
| `detect_callback()` | 主循环：取帧 → 检测 → 匹配点云 → 发布 |
| `get_3d_position(bbox, point_cloud)` | 将 2D 检测框投影到点云，提取框内点云并聚类 |
| `update_car_list(results)` | 更新 CarList 中的车辆信息 |

#### lidar_node.py — LidarNode（ROS2 节点）

路径：`hnurm_radar/lidar_scheme/lidar_node.py`

激光雷达数据接收与预处理节点。

**功能**：
- 订阅 `/livox/lidar` 话题接收 Livox 点云数据
- 对点云进行体素降采样和离群点滤波
- 将处理后的点云通过 TF 变换到相机坐标系
- 提供点云数据供 `detector_node` 查询

#### radar.py — RadarNode（ROS2 节点）

路径：`hnurm_radar/lidar_scheme/radar.py`

相机+雷达联合定位的主节点（备用方案），整合了相机检测和雷达数据处理的完整流程。

**功能**：
- 初始化相机和 Converter
- 通过 GUI 进行相机-赛场坐标系标定
- 执行 YOLO 检测 + 点云匹配
- 管理 CarList 并发布位置信息

---

### 1.3 shared — 共享节点

两种方案共用的节点。

#### judge_messager.py — JudgeMessager（ROS2 节点）

路径：`hnurm_radar/shared/judge_messager.py`

裁判系统串口通信节点，负责与 RoboMaster 裁判系统的双向数据交换。

**架构设计**：

```
┌─────────────────────────────────────────────┐
│              JudgeMessager (主进程)           │
│  ┌─────────────┐    ┌────────────────────┐  │
│  │ judge_loop  │    │ location_callback  │  │
│  │ (发送线程)   │    │ (ROS2 订阅回调)    │  │
│  └──────┬──────┘    └────────┬───────────┘  │
│         │                    │               │
│         ▼                    ▼               │
│    串口发送              更新 locations       │
│  (0x0305 坐标)                               │
│  (0x0301 双倍易伤)                            │
├─────────────────────────────────────────────┤
│              Receiver (子进程)                │
│  ┌──────────────────────────────────┐       │
│  │ parse_cmd_id_batch()             │       │
│  │ 批量解析串口接收帧               │       │
│  │ → 0x0001 比赛状态（剩余时间）     │       │
│  │ → 0x0003 机器人血量              │       │
│  │ → 0x020C 标记进度                │       │
│  │ → 0x020E 双倍易伤状态            │       │
│  │ → 0x0105 飞镖目标                │       │
│  └──────────────────────────────────┘       │
└─────────────────────────────────────────────┘
```

**关键类**：

| 类 | 说明 |
|---|---|
| `JudgeMessager(Node)` | ROS2 节点，管理发送线程和接收进程 |
| `Receiver` | 独立进程，持续解析串口接收数据，通过 `multiprocessing.Value/Array` 共享内存与主进程通信 |
| `Tools` | 帧率控制工具（静态方法） |

**通信协议**：
- 遵循 RoboMaster 裁判系统 v1.4 协议
- 帧格式：`[SOF(1B) | data_length(2B) | seq(1B) | CRC8(1B)] [cmd_id(2B)] [data(nB)] [CRC16(2B)]`
- SOF 固定为 `0xA5`
- 使用 CRC8 校验帧头，CRC16 校验整帧

**发送功能**：

| 方法 | 命令 ID | 说明 |
|---|---|---|
| `send_map_robot_location()` | `0x0305` | 发送 6 个敌方机器人的赛场坐标（单位 cm） |
| `send_double_effect()` | `0x0301→0x0121` | 触发双倍易伤 |

**接收解析**：

| 命令 ID | 方法 | 数据内容 |
|---|---|---|
| `0x0001` | `process_game_status()` | 比赛阶段、剩余时间 |
| `0x0003` | `parse_robot_status()` | 红蓝双方 1-5 号 + 7 号 + 前哨站 + 基地血量 |
| `0x020C` | `parse_mark_process()` | 6 个敌方车辆的标记进度（0-120） |
| `0x020E` | `parse_double_effect()` | 双倍易伤机会次数 + 是否正在激活 |
| `0x0105` | `parse_dart_target()` | 飞镖选定目标（0=前哨站, 1=基地固定, 2=基地随机） |

#### display_panel.py — DisplayPanel（ROS2 节点）

路径：`hnurm_radar/shared/display_panel.py`

操作手显示面板节点，订阅检测结果并在小地图上实时显示机器人位置。

**功能**：
- 订阅 `detect_view` 话题获取标注后的相机画面
- 订阅 `location` / `ekf_location_filtered` 获取机器人坐标
- 在赛场地图上绘制机器人位置标记
- 显示 CameraDetector 窗口和 MiniMap 窗口

#### publish_video.py — PublishVideo（ROS2 节点）

路径：`hnurm_radar/shared/publish_video.py`

视频流发布节点，将相机画面通过 ROS2 话题发布，供其他节点或远程监控使用。

---

### 1.4 Camera — 相机驱动封装

路径：`hnurm_radar/Camera/`

#### HKCam.py — HKCam 类

海康工业相机（MV-CS 系列）的 Python SDK 封装。

**初始化流程**：
1. 枚举设备（GigE + USB）
2. 根据 `cameraId` 索引创建句柄并打开设备
3. 设置参数：分辨率 3072×2048、像素格式 BayerRG12、曝光 8000μs、增益 20
4. 开始取流

**关键方法**：

| 方法 | 说明 |
|---|---|
| `__init__(cameraId)` | 初始化相机，设置参数并开始取流 |
| `getFrame()` | 获取一帧图像，BayerRG12 → BGR8 转换，返回 numpy 数组 (H, W, 3) |
| `__del__()` | 析构：停止取流 → 关闭设备 → 销毁句柄 |

**依赖**：`MvImport/` 目录下的海康 MVS SDK Python 绑定（`MvCameraControl_class.py` 等）。

---

### 1.5 Car — 车辆数据模型

路径：`hnurm_radar/Car/Car.py`

#### Car 类

单个车辆的信息容器，存储从检测到通信所需的全部数据。

**属性**：

| 属性 | 类型 | 说明 |
|---|---|---|
| `car_id` | int | 车辆 ID（1-5, 7 为红方；101-105, 107 为蓝方） |
| `color` | str | 颜色，由 car_id 自动推断 |
| `track_id` | int | 追踪器分配的追踪编号 |
| `conf` | float | 当前追踪置信度 |
| `image_xywh` | list | 图像中的中心+宽高（归一化） |
| `image_xyxy` | list | 图像中的左上右下坐标（归一化） |
| `center_xy` | list | 图像中心坐标（归一化） |
| `camera_xyz` | list | 相机坐标系三维坐标（m） |
| `field_xyz` | list | 赛场坐标系三维坐标（m） |
| `life_span` | int | 当前可信生命周期（帧数） |
| `trust` | bool | 信息是否可信 |

**生命周期机制**：
- 每次检测到车辆时，`life_span` 重置为 `life_span_max`
- 每帧未检测到时 `life_span -= 1`
- 降至 0 时标记为不可信（`trust = False`），但不清除坐标数据（仍发送给裁判系统）

#### CarList 类

车辆列表管理器，所有检测结果写入此处，串口通信和决策从此处读取。

**线程安全**：使用 `threading.Lock()` 保护读写操作。

**关键方法**：

| 方法 | 说明 |
|---|---|
| `update_car_info(results)` | 批量更新车辆信息，参数格式：`[track_id, car_id, xywh, conf, camera_xyz, field_xyz]` |
| `get_map_info()` | 获取所有有坐标的车辆信息，供串口发送 |
| `get_all_info()` | 获取全部车辆详细信息，供可视化使用 |
| `get_center_info()` | 获取哨兵和敌方车辆的图像中心坐标，供辅助决策 |
| `get_car_id(label)` | 标签转 ID，如 `"R1"` → `1`，`"B3"` → `103` |

---

### 1.6 Lidar — 坐标转换模块

路径：`hnurm_radar/Lidar/Converter.py`

#### Converter 类

多坐标系转换器，处理激光雷达、相机、图像、赛场四个坐标系之间的变换。使用 CuPy（GPU 加速）进行矩阵运算。

**坐标系关系**：

```
激光雷达坐标系 ──(外参 R,T)──→ 相机坐标系 ──(内参 K)──→ 图像坐标系
                                    │
                                    └──(PnP 求解)──→ 赛场坐标系
```

**初始化参数**（从 `converter_config.yaml` 加载）：

| 参数组 | 内容 |
|---|---|
| `calib.extrinsic` | 外参矩阵 R(3×3) 和 T(3×1)，激光雷达→相机 |
| `calib.intrinsic` | 内参 fx, fy, cx, cy |
| `calib.distortion` | 畸变系数（5 参数） |
| `params` | 最大深度、图像宽高 |
| `cluster` | DBSCAN 参数：eps, min_points |
| `filter` | 体素降采样 voxel_size、离群点滤波 nb_neighbors, std_ratio |
| `field` | 赛场四角坐标（红/蓝方各一组） |

**关键方法**：

| 方法 | 输入 | 输出 | 说明 |
|---|---|---|---|
| `lidar_to_camera(pcd)` | Open3D 点云 | 修改 pcd.points | 激光雷达→相机坐标系变换 |
| `camera_to_lidar(pcd)` | Open3D 点云 | 修改 pcd.points | 相机→激光雷达坐标系变换 |
| `camera_to_image(pc)` | numpy (N,3) | numpy (N,3) uvz | 相机→图像坐标系投影 |
| `camera_to_field(point)` | numpy (3,) | numpy (3,) | 相机→赛场坐标系变换 |
| `generate_depth_map(pc)` | numpy (N,3) | BGR 深度图 | 生成伪彩色深度图 |
| `get_points_in_box(pc, box)` | 点云 + 检测框 | numpy (M,3) | 提取检测框内的点云 |
| `cluster(pcd)` | Open3D 点云 | 最大簇点云 + 中心 | DBSCAN 聚类，返回最大簇 |
| `filter(pcd)` | Open3D 点云 | 滤波后点云 | 体素降采样 + 离群点去除 |

**相机-赛场标定**：
- `camera_to_field_init()` / `camera_to_field_init_by_image()`：通过手动选取赛场四角对应的像素点，使用 `cv2.solvePnP()` 求解相机到赛场的变换矩阵

#### ROISelector 类

ROI 区域选择器，用于手动框选英雄梯高区和哨兵巡逻区。

| 方法 | 说明 |
|---|---|
| `select_hero_highland_points()` | GUI 选取英雄梯高区四角 |
| `select_sentinel_patrol_points()` | GUI 选取哨兵巡逻区四角 |
| `is_point_in_hero_highland(point)` | 判断点是否在英雄梯高区内 |
| `is_point_in_sentinel_patrol(point)` | 判断点是否在哨兵巡逻区内 |

---

### 1.7 camera_locator — 透视变换标定工具

路径：`hnurm_radar/camera_locator/`

#### perspective_calibrator.py

PyQt5 GUI 标定工具，用于建立相机像素坐标与赛场坐标的映射关系。

**操作流程**：
1. 左侧显示相机实时画面，右侧显示赛场地图
2. 点击「开始标定」冻结画面
3. 在左侧点击地面特征点（≥4 个），在右侧点击对应赛场位置
4. 可切换「高地层」标定高地区域
5. 点击「保存计算」，使用 `cv2.findHomography()` 计算 Homography 矩阵
6. 结果保存到 `configs/perspective_calib.json`

**输出格式**（`perspective_calib.json`）：
```json
{
  "ground": {
    "H": [[...], [...], [...]],
    "src_points": [...],
    "dst_points": [...]
  },
  "highland": {
    "H": [[...], [...], [...]],
    "src_points": [...],
    "dst_points": [...]
  }
}
```

#### anchor.py — Anchor 类

标定点数据结构，存储用户选取的像素点坐标列表（`vertexes`）。

#### point_picker.py — PointsPicker 类

图像点选取工具，提供 OpenCV 窗口让用户点击选取坐标点。

---

### 1.8 filters — 滤波器模块

路径：`hnurm_radar/filters/kalman_filter.py`

#### EnhancedKalmanFilter 类

增强型卡尔曼滤波器，用于平滑 camera_scheme 中的赛场坐标。

**特性**：
- 状态向量：`[x, y]`（赛场坐标）
- 支持坐标跳变检测：当新观测值与预测值偏差超过 `jump_threshold` 时，重置滤波器
- 超时清理：超过 `max_inactive_time` 未更新的滤波器自动销毁
- 可配置过程噪声（`process_noise`）和观测噪声（`measurement_noise`）

---

### 1.9 Tools — 通用工具

路径：`hnurm_radar/Tools/Tools.py`

#### Tools 类（静态方法集合）

| 方法 | 说明 |
|---|---|
| `get_time_stamp()` | 返回格式化时间戳字符串 `YYYY-MM-DD-HH-MM-SS` |
| `frame_control_sleep(fps, last_time)` | 帧率控制（sleep 方式），返回当前时间戳 |
| `frame_control_skip(fps, last_time)` | 帧率控制（跳帧方式），返回 `(skip, last_time)` |

---

### 1.10 ultralytics — 内嵌 YOLO 库

路径：`src/hnurm_radar/ultralytics/`

本地修改版的 Ultralytics YOLO 库，包含完整的模型定义、推理引擎、数据处理、追踪器等。

**主要子目录**：

| 目录 | 说明 |
|---|---|
| `cfg/` | 模型配置文件（YOLO 各版本的 yaml） |
| `data/` | 数据加载与增强 |
| `engine/` | 推理引擎（predictor, trainer, exporter） |
| `models/` | 模型定义（YOLO, SAM, RTDETR 等） |
| `nn/` | 神经网络模块（卷积、注意力、检测头等） |
| `trackers/` | 目标追踪器（ByteTrack, BotSort） |
| `utils/` | 工具函数（NMS, 指标计算, 可视化等） |

> 此库为本地内嵌版本，可能包含针对本项目的定制修改。不建议直接用 pip 版本替换。

---

## 2. hnurm_bringup — 启动包

路径：`src/hnurm_bringup/`，包类型：CMake（ament_cmake）。

仅包含 launch 文件，不含业务代码。

### hnurm_radar_launch.py

相机+激光雷达方案的完整启动文件。

**启动的节点**：

| 节点 | 包 | 说明 |
|---|---|---|
| `livox_ros_driver2_node` | `livox_ros_driver2` | 激光雷达驱动 |
| `registration_node` | `registration` | 点云配准 |
| `detector_node` | `hnurm_radar` | YOLO 检测 |
| `lidar_node` | `hnurm_radar` | 雷达数据处理 |
| `ekf_node` | `ekf` | 坐标滤波 |
| `judge_messager` | `hnurm_radar` | 裁判系统通信 |
| `display_panel` | `hnurm_radar` | 操作手面板 |

### hnurm_radar_video_launch.py

纯相机方案的启动文件。

**启动的节点**：

| 节点 | 包 | 说明 |
|---|---|---|
| `camera_detector` | `hnurm_radar` | 相机检测+透视变换 |
| `ekf_node` | `ekf` | 坐标滤波 |
| `judge_messager` | `hnurm_radar` | 裁判系统通信 |

---

## 3. detect_result — 自定义消息包

路径：`src/detect_result/`，包类型：CMake（ament_cmake）。

定义了节点间通信使用的 ROS2 自定义消息类型。

### 消息定义

#### DetectResult.msg
```
int32 id            # 车辆 ID
string label        # 标签（如 "R1", "B3"）
float32[] bbox      # 检测框 [x, y, w, h]（归一化）
float32 conf        # 置信度
```

#### Robots.msg
```
DetectResult[] robots   # 检测结果数组
```

#### Location.msg
```
int32 id            # 车辆 ID
string label        # 颜色标签（"Red" 或 "Blue"）
float32 x           # 赛场 X 坐标（m）
float32 y           # 赛场 Y 坐标（m）
float32 z           # 赛场 Z 坐标（m）
```

#### Locations.msg
```
Location[] locs     # 位置信息数组
```

**话题使用关系**：

| 话题名 | 消息类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `location` | `Locations` | `camera_detector` / `detector_node` | `ekf_node` |
| `ekf_location_filtered` | `Locations` | `ekf_node` | `judge_messager`, `display_panel` |
| `detect_view` | `sensor_msgs/Image` | `camera_detector` / `detector_node` | `display_panel` |

---

## 4. ekf — 扩展卡尔曼滤波包

路径：`src/ekf/`，包类型：Python（ament_python）。

### EKFNode（ROS2 节点）

文件：`ekf/ekf_node.py`

对检测节点输出的原始坐标进行扩展卡尔曼滤波，平滑轨迹并预测运动趋势。

**状态向量**：`[x, vx, y, vy]`（位置 + 速度，4 维）

**工作流程**：
1. 订阅 `location` 话题接收原始坐标
2. 50ms 定时器触发滤波计算
3. 对每个机器人维护独立的 EKF 滤波器实例（共 6 个）
4. 计算速度和加速度作为控制输入
5. 发布滤波后的坐标到 `ekf_location_filtered` 话题

**关键类**：

| 类 | 说明 |
|---|---|
| `RobotInfo` | 机器人状态信息（位置、速度、加速度、时间戳） |
| `EKFNode(Node)` | ROS2 节点，管理 6 个 EKF 滤波器 |
| `RobotEKF` | EKF 滤波器实现（位于 `ekf/RobotEKF/` 子模块） |

**EKF 参数**：

| 参数 | 值 | 说明 |
|---|---|---|
| `pval` | 0.001 | 初始协方差 |
| `qval` | 1e-4 | 过程噪声（较小，信任模型） |
| `rval` | 0.0005 | 观测噪声（较小，信任观测） |
| `interval` | 0.05 | 滤波周期（50ms） |

**ID 映射**：车辆 ID 映射到数组下标（1→0, 2→1, ..., 7→5），红蓝方共用同一套映射。

---

## 5. registration — 点云配准包

路径：`src/registration/`，包类型：C++（ament_cmake）。

点云配准节点，使用 Quatro 鲁棒配准算法将 Livox 激光雷达的实时点云与预建地图对齐，输出 TF 变换。

**功能**：
- 订阅 `/livox/lidar` 点云话题
- 加载预建点云地图（`target.xyz`）
- 使用 Quatro + ICP 进行粗配准 + 精配准
- 发布 TF 变换（激光雷达→地图坐标系）

**配置文件**：`params/default.yaml`，包含 ICP 参数、Quatro 参数、地图路径等。

**依赖**：`Quatro` 库（`src/Quatro/`）。

---

## 6. 第三方包

### livox_ros_driver2

路径：`src/livox_ros_driver2/`

Livox 官方 ROS2 驱动，支持 HAP、MID-360 等型号。

**配置**：`config/HAP_config.json`（项目中复制到 `configs/HAP_config.json`）

**发布话题**：`/livox/lidar`（PointCloud2）

### Livox-SDK2

路径：`src/Livox-SDK2/`

Livox 硬件通信 SDK，`livox_ros_driver2` 的底层依赖。纯 C++ 库，编译后供驱动调用。

### Quatro

路径：`src/Quatro/`

鲁棒点云配准算法库，基于 TEASER++ 实现。提供 FPFH 特征提取和鲁棒匹配功能，供 `registration` 包调用。

---

## 7. 数据流总览

### 方案一：纯相机方案

```
┌──────────┐    frame     ┌──────────────────┐   location   ┌──────────┐  ekf_location  ┌────────────────┐
│  相机     │ ──────────→ │ camera_detector  │ ──────────→ │ ekf_node │ ────────────→ │ judge_messager │
│ (HKCam/  │             │ (YOLO+透视变换)   │             └──────────┘               │ (串口→裁判系统) │
│  USB/    │             │                  │                                         └────────────────┘
│  静态图)  │             │  detect_view     │
│          │             │ ──────────────→  display_panel (小地图可视化)
└──────────┘             └──────────────────┘
```

### 方案二：相机+激光雷达方案

```
┌──────────┐   /livox/lidar  ┌───────────────┐    TF     ┌────────────┐
│ Livox HAP│ ─────────────→ │ registration  │ ────────→ │ lidar_node │
└──────────┘                 │ (Quatro+ICP)  │           └─────┬──────┘
                             └───────────────┘                 │ 点云数据
                                                               ▼
┌──────────┐    frame     ┌───────────────┐   location   ┌──────────┐  ekf_location  ┌────────────────┐
│  相机     │ ──────────→ │ detector_node │ ──────────→ │ ekf_node │ ────────────→ │ judge_messager │
│ (HKCam)  │             │ (YOLO+点云匹配)│             └──────────┘               │ (串口→裁判系统) │
└──────────┘             │               │                                         └────────────────┘
                         │  detect_view  │
                         │ ───────────→  display_panel (小地图可视化)
                         └───────────────┘
```

### 关键话题一览

| 话题 | 消息类型 | 发布者 | 订阅者 | 频率 |
|---|---|---|---|---|
| `/livox/lidar` | `PointCloud2` | `livox_ros_driver2` | `registration`, `lidar_node` | ~10Hz |
| `location` | `Locations` | `camera_detector` / `detector_node` | `ekf_node` | ~20Hz |
| `ekf_location_filtered` | `Locations` | `ekf_node` | `judge_messager`, `display_panel` | 20Hz |
| `detect_view` | `Image` | `camera_detector` / `detector_node` | `display_panel` | ~20Hz |

---

*文档版本: 2026-02-14*

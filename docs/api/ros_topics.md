# 📡 ROS2 话题与节点清单

本文档列出雷达站系统中所有 ROS2 节点、话题、消息类型及其订阅/发布关系。

---

## 目录

- [节点总览](#节点总览)
- [话题清单](#话题清单)
- [自定义消息类型](#自定义消息类型)
- [节点详细说明](#节点详细说明)
- [数据流图](#数据流图)

---

## 节点总览

| 节点名 | 包 | 入口点 | 方案 | 说明 |
|--------|-----|--------|------|------|
| `lidar_listener` | `hnurm_radar` | `lidar_node` | 方案二/空中 | 激光雷达数据接收与预处理 |
| `detector` | `hnurm_radar` | `detector_node` | 方案二 | YOLO 三阶段目标检测 |
| `radar` | `hnurm_radar` | `radar_node` | 方案二 | 点云-图像融合定位 |
| `camera_detector` | `hnurm_radar` | `camera_detector` | 方案一 | 相机检测+透视变换定位 |
| `air_target_node` | `hnurm_radar` | `air_target_node` | 空中 | 空中目标检测（背景减除+DBSCAN） |
| `display_panel` | `hnurm_radar` | `display_panel` | 通用 | 小地图可视化面板 |
| `judge_messager` | `hnurm_radar` | `judge_messager` | 通用 | 裁判系统串口通信 |
| `ekf_node` | `ekf` | `ekf_node` | 通用 | 扩展卡尔曼滤波（7 路独立滤波器） |
| `registration_node` | `registration` | — (C++) | 方案二/空中 | ICP 点云配准 |
| `livox_driver` | `livox_ros_driver2` | — (C++) | 方案二/空中 | Livox HAP 激光雷达驱动 |

---

## 话题清单

### 传感器输入话题

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|---------|--------|--------|------|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | `livox_driver` | `lidar_listener` | Livox HAP 原始点云 |
| `/image` | `sensor_msgs/Image` | `hnurm_camera_node` (外部) | `detector`, `radar` | 相机图像（方案二） |
| `/camera_info` | `sensor_msgs/CameraInfo` | `camera_info_publisher` (外部) | `radar` | 相机内参 |

### 中间处理话题

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|---------|--------|--------|------|
| `/lidar_pcds` | `sensor_msgs/PointCloud2` | `lidar_listener` | `registration_node` | 累积点云（用于 ICP 配准） |
| `/target_pointcloud` | `sensor_msgs/PointCloud2` | `lidar_listener` | `radar`, `air_target_node` | 背景减除后的前景点云 |
| `/detect_result` | `detect_result/Robots` | `detector`, `camera_detector` | `radar` | 目标检测结果 |

### 输出话题

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|---------|--------|--------|------|
| `/location` | `detect_result/Locations` | `radar`, `camera_detector`, `air_target_node` | `ekf_node` | 目标赛场坐标（未滤波） |
| `/ekf_location_filtered` | `detect_result/Locations` | `ekf_node` | `judge_messager`, `display_panel` | 卡尔曼滤波后的坐标 |
| `/ekf_diagnostics` | `detect_result/EkfDiagnosticsArray` | `ekf_node` | `display_panel`, Foxglove | EKF 诊断指标（新息、抖动、检测率、协方差等） |
| `/detect_view` | `sensor_msgs/Image` | `detector`, `camera_detector` | Foxglove / RViz | YOLO 检测结果叠加画面 |
| `/map_view` | `sensor_msgs/Image` | `display_panel` | Foxglove / RViz | 渲染后的小地图画面（BGR8, RELIABLE QoS） |

### TF 变换

| 变换 | 发布者 | 说明 |
|------|--------|------|
| `map` → `livox` | `registration_node` | ICP 配准结果（激光雷达→地图坐标系） |

### 调试话题（空中方案）

| 话题名 | 消息类型 | 发布者 | 说明 |
|--------|---------|--------|------|
| `air_debug/cluster_points` | `sensor_msgs/PointCloud2` | `air_target_node` | 聚类后的目标点云 |
| `air_debug/cluster_markers` | `visualization_msgs/MarkerArray` | `air_target_node` | 聚类可视化标记 |
| `air_debug/stats` | `std_msgs/String` | `air_target_node` | 检测统计信息（JSON） |

### 配准相关话题

| 话题名 | 消息类型 | 发布者 | 订阅者 | 说明 |
|--------|---------|--------|--------|------|
| `/global_pcd_map` | `sensor_msgs/PointCloud2` | `registration_node` | RViz | 全局点云地图 |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | RViz (用户) | `registration_node` | 用户给的初始位姿 |

---

## 自定义消息类型

所有自定义消息定义在 `src/detect_result/msg/` 中：

### `DetectResult.msg`

单个检测目标：

```
int32 id           # 机器人 ID
float64 conf       # 置信度
int32[] xyxy        # 边界框 [x1, y1, x2, y2]
```

### `Robots.msg`

一帧内所有检测目标：

```
DetectResult[] robots    # 检测结果列表
```

### `Location.msg`

单个目标的赛场坐标：

```
int32 id           # 机器人 ID
float64 x          # 赛场 X 坐标 (m)
float64 y          # 赛场 Y 坐标 (m)
float64 z          # 赛场 Z 坐标 (m)（空中方案使用）
```

### `Locations.msg`

所有目标的赛场坐标：

```
Location[] locations    # 坐标列表
```

### `EkfDiagnostics.msg`

单个 EKF slot 的诊断指标：

```
int32 slot_id                    # EKF 滤波器编号 0-6
int32 robot_id                   # 实际机器人 ID，未激活时为 0
float32 detection_rate_hz        # 滑动窗口内的检测频率 (Hz)
float32 time_since_last_det_ms   # 距上次检测的毫秒数
float32 jitter                   # 帧间位移抖动 (m)
float32 innovation_x             # EKF 新息 X: 观测 − 预测 (m)
float32 innovation_y             # EKF 新息 Y: 观测 − 预测 (m)
float32 innovation_norm          # 新息范数 (m)
float32 raw_vs_filtered_dist     # |原始检测 − EKF 滤波| (m)
float32 covariance_trace         # 协方差矩阵迹 tr(P)
float32 cov_xx                   # P[0,0] X 方向位置方差 (m²)
float32 cov_yy                   # P[2,2] Y 方向位置方差 (m²)
```

### `EkfDiagnosticsArray.msg`

一帧中所有 EKF slot 的诊断指标集合：

```
builtin_interfaces/Time stamp    # 发布时间戳
EkfDiagnostics[] slots           # 各 slot 诊断数据（最多 7 个）
```

---

## 节点详细说明

### `lidar_listener` — 激光雷达预处理

- **源码**：`src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py`
- **配置**：`configs/main_config.yaml` → `lidar` 段
- **功能**：
  1. 订阅 `/livox/lidar` 原始点云
  2. 距离滤波（近距离/远距离剔除）
  3. 多帧累积（PcdQueue，10 帧）→ 发布到 `/lidar_pcds`
  4. 背景减除 → 发布前景到 `/target_pointcloud`

### `detector` — YOLO 目标检测

- **源码**：`src/hnurm_radar/hnurm_radar/lidar_scheme/detector_node.py`
- **配置**：`configs/detector_config.yaml`
- **功能**：三阶段推理（目标检测→装甲板分类→灰色装甲板）

### `radar` — 点云-图像融合

- **源码**：`src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py`
- **功能**：将 2D 检测框投影到 3D 点云，通过聚类获取精确坐标

### `camera_detector` — 纯相机方案

- **源码**：`src/hnurm_radar/hnurm_radar/camera_scheme/camera_detector.py`
- **功能**：相机采集 → YOLO 检测 → 透视变换定位

### `air_target_node` — 空中目标检测

- **源码**：`src/hnurm_radar/hnurm_radar/air_scheme/air_target_node.py`
- **配置**：`configs/main_config.yaml` → `air_target` 段
- **功能**：背景减除 → DBSCAN 聚类 → 卡尔曼跟踪

### `ekf_node` — 卡尔曼滤波

- **源码**：`src/ekf/ekf/ekf_node.py`
- **功能**：
  1. 维护 7 个独立 EKF 滤波器（地面 1-5 + 哨兵 + 空中）
  2. 订阅 `/location` → 滤波 → 发布 `/ekf_location_filtered`
  3. 发布 `/ekf_diagnostics`（`EkfDiagnosticsArray`）：新息、抖动、检测率、协方差等诊断指标

### `judge_messager` — 裁判系统通信

- **源码**：`src/hnurm_radar/hnurm_radar/shared/judge_messager.py`
- **配置**：`configs/main_config.yaml` → `communication` 段
- **功能**：
  - **发送**：`0x0305` 敌方坐标，`0x0301→0x0121` 双倍易伤
  - **接收**：`0x0001` 比赛状态，`0x0003` 血量，`0x020C` 标记进度

---

## 数据流图

```
方案二（主力）数据流：

/livox/lidar ──→ lidar_listener ──┬──→ /lidar_pcds ──→ registration_node ──→ TF(map→livox)
                                  │
                                  └──→ /target_pointcloud ──→ radar ──→ /location
                                                                ↑
/image ──→ detector ──→ /detect_result ─────────────────────────┘
                                                                         ↓
                                               ekf_node ←── /location
                                                  ├──→ /ekf_location_filtered
                                                  │        ├──→ judge_messager → 串口
                                                  │        └──→ display_panel → 可视化 + /map_view
                                                  └──→ /ekf_diagnostics
                                                           └──→ display_panel (协方差椭圆)
                                                           └──→ Foxglove (Plot 面板)
```

```
方案一（纯相机）数据流：

相机(HKCam/USB) ──→ camera_detector ──→ /location ──→ ekf_node ──┬──→ /ekf_location_filtered
                                                                 │        ├──→ judge_messager
                                                                 │        └──→ display_panel
                                                                 └──→ /ekf_diagnostics → Foxglove
```

```
空中方案数据流：

/livox/lidar ──→ lidar_listener ──┬──→ /lidar_pcds ──→ registration_node ──→ TF(map→livox)
                                 │
                                 └──→ /target_pointcloud ──→ air_target_node ──→ /location
                                                                                     ↓
                                                             ekf_node ←── /location
                                                                ├──→ /ekf_location_filtered
                                                                └──→ /ekf_diagnostics
```

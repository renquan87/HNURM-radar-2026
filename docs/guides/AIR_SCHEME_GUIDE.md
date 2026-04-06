# 空中机器人检测方案（air_scheme）使用说明与详细解释

> 本文档描述 hnurm_radar 项目方案三：纯激光雷达空中机器人定位。
> 该方案为本项目原创实现，卡尔曼滤波设计参考了 HITS-radar-2024 的 KalmanFilter.cpp。

---

## 目录

1. [方案概述](#1-方案概述)
2. [数据流总览](#2-数据流总览)
3. [模块详解](#3-模块详解)
   - 3.1 [air_config.py — 配置管理](#31-air_configpy--配置管理)
   - 3.2 [point_cloud_processor.py — 点云预处理](#32-point_cloud_processorpy--点云预处理)
   - 3.3 [background_subtractor.py — 背景减除](#33-background_subtractorpy--背景减除)
   - 3.4 [cluster_detector.py — DBSCAN 聚类检测](#34-cluster_detectorpy--dbscan-聚类检测)
   - 3.5 [air_kalman_filter.py — 卡尔曼滤波器](#35-air_kalman_filterpy--卡尔曼滤波器)
   - 3.6 [target_tracker.py — 多目标跟踪器](#36-target_trackerpy--多目标跟踪器)
   - 3.7 [air_target_node.py — ROS2 节点](#37-air_target_nodepy--ros2-节点)
4. [坐标系说明](#4-坐标系说明)
5. [配置参数详解](#5-配置参数详解)
6. [敌我区分与 ID 约定](#6-敌我区分与-id-约定)
7. [使用说明（快速上手）](#7-使用说明快速上手)
8. [调试工具](#8-调试工具)
9. [常见问题与排查](#9-常见问题与排查)
10. [参数调优指南](#10-参数调优指南)
11. [已修复的历史 Bug](#11-已修复的历史-bug)
12. [本项目 air_scheme 与 HITS-radar-2024 全面对比](#12-本项目-air_scheme-与-hits-radar-2024-全面对比)
13. [当前问题诊断与优化计划](#13-当前问题诊断与优化计划)
14. [详细 Q&A：逐条解答（2026-03-07 更新）](#14-详细-qa逐条解答2026-03-07-更新)
15. [Rosbag 离线测试方案](#15-rosbag-离线测试方案)
16. [基于规则手册的完整空中机器人检测逻辑](#16-基于规则手册的完整空中机器人检测逻辑)
17. [从近到远的行动计划清单](#17-从近到远的行动计划清单)

---

## 1 方案概述

空中机器人检测方案（方案三）**仅使用激光雷达**，不依赖相机。

**核心思路**：
1. 激光雷达扫描全场，输出三维点云
2. 对点云做 ROI 裁剪 + 降采样，用背景减除滤掉地面/墙体等静态点
3. 提取空中高度区域（Z 轴高度过滤），只保留飞行高度范围的点
4. 对剩余点做 DBSCAN 聚类，每个簇视为一个候选无人机
5. 对候选目标做卡尔曼滤波 + 跨帧跟踪，稳定输出位置
6. 通过 TF 变换将雷达坐标系转为赛场坐标系，发布到 `/location` 话题

**卡尔曼滤波设计参考**：  
`air_kalman_filter.py` 的过程噪声矩阵初始化方式（`P₀ = Q × init_p_times`）、动态过程噪声放大（`cov_factor`）、协方差增长控制（`stop_p_time`）等参数命名和公式参考了 HITS-radar-2024 的 `KalmanFilter.cpp` 实现思路；其余模块（预处理、聚类、背景减除、跟踪器、ROS节点）均为本项目独立实现。

---

## 2 数据流总览

```
/livox/lidar (PointCloud2)
       ↓
   lidar_node（话题转发，已有节点）
       ↓
/lidar_pcds (PointCloud2，累积点云)
       ↓
   air_target_node._pcds_callback()
       ↓  缓存到 _latest_points
   air_target_node._process_callback()（定时器驱动，默认 10 Hz）
       ↓
   [0] 背景预加载（可选，首次 TF 就绪时从 bg_pcd_file 加载）
       ↓
   process_frame_pcd(pcd)
       ├── [1] AirPointCloudProcessor.crop_roi()      ROI 裁剪
       ├── [2] AirPointCloudProcessor.downsample()    体素降采样
       ├── [3] AirBackgroundSubtractor.learn()        学习背景（地面区域，预加载后跳过）
       ├── [4] AirBackgroundSubtractor.filter()       背景减除
       ├── [5] AirPointCloudProcessor.filter_height() 高度过滤（取空中区域）
       ├── [6] AirClusterDetector.detect()            DBSCAN 聚类
       ├── [7] AirTargetTracker.update()              卡尔曼跟踪
       │        ├── _push_single()     马氏距离匹配
       │        ├── _try_separate()    遮挡分裂检测
       │        ├── _do_loose_query()  松弛搜索恢复丢失目标
       │        └── _force_combine()   强制合并过近目标
       └── [8] 坐标变换 lidar→map + 敌我区分
                   ↓
           ┌──────┴──────────────────────┐
           ↓                             ↓
/location (Locations)            air_debug/* (调试话题)
    ↓                              ├── air_debug/air_points      空中区域点云
ekf_node → display_panel           ├── air_debug/cluster_points  聚类点云(按ID着色)
                                   └── air_debug/cluster_markers 包围盒+标注
```

---

## 3 模块详解

### 3.1 `air_config.py` — 配置管理

**职责**：定义所有配置数据类（dataclass），并从 `main_config.yaml` 的 `air_target` 段解析配置。

**关键数据类**：

| 类名 | 用途 |
|------|------|
| `AirROIConfig` | ROI 区域范围（x/y/z 各轴最小最大值） |
| `AirHeightFilterConfig` | 空中区域高度过滤范围（`z_min`, `z_max`） |
| `AirAdaptiveHeightConfig` | 高度自适应过滤参数 |
| `AirPreprocessConfig` | 预处理综合配置（ROI + 体素 + 高度） |
| `AirClusterConfig` | DBSCAN 聚类参数 |
| `AirTargetFilterConfig` | 目标筛选参数（点数/尺寸/置信度） |
| `AirKalmanConfig` | 卡尔曼参数（过程噪声/观测噪声/速度衰减等） |
| `AirTrackingConfig` | 跟踪器参数（最大丢失帧/匹配距离/确认帧数等） |
| `AirLooseQueryConfig` | 松弛查询参数 |
| `AirBackgroundConfig` | 背景减除参数 |
| `AirBufferConfig` | 多帧缓冲参数 |
| `AirTargetConfig` | 顶层综合配置（含 strict_dual_uav 等） |

**入口函数**：
```python
cfg = YAML().load(open("configs/main_config.yaml"))
air_cfg = load_air_target_config(cfg)
# air_cfg.preprocessing.height_filter.z_max → -0.5
```

---

### 3.2 `point_cloud_processor.py` — 点云预处理

**职责**：对原始点云做 ROI 裁剪、体素降采样、高度过滤（含自适应）。

**关键方法**：

#### `crop_roi(pcd)`
```
根据配置的 x/y/z 范围裁剪点云。
去除视野外、墙体内的无效点。
使用 numpy 布尔索引一次完成，效率高。
```

#### `downsample(pcd)`
```
体素降采样（voxel_size 默认 0.05m）。
降低后续 DBSCAN 的输入规模，显著提速。
```

#### `filter_height(pcd, tracked_heights=None)`
```
提取空中区域点云（z_min ≤ z ≤ z_max）。
可选传入已跟踪目标的高度列表，启用自适应范围。

雷达坐标系中（Z 轴向上为正，激光雷达通常安装高于地面）：
  z = -1.5 → 雷达下方 1.5m（较高空域）
  z = -0.5 → 雷达下方 0.5m（较低空域）
```

#### `get_adaptive_height_range(tracked_heights)`
```
根据已知目标高度动态调整过滤范围：
  z_min = min(tracked_z) - margin
  z_max = max(tracked_z) + margin
限制在 [min_span, max_span] 之间，防止过窄或过宽。
```

---

### 3.3 `background_subtractor.py` — 背景减除

**职责**：建立静态背景体素模型，过滤地面/墙壁等固定点，保留动态目标。

**核心原理**：
- 体素网格：将 3D 空间划分为小方格（voxel_size=0.15m）
- 每帧统计每个体素被点云命中的次数
- 超过 `occupy_threshold`（默认 5 次）视为静态背景
- `filter()` 时，命中背景体素的点被过滤掉

**关键参数 `learn_z_min`（曾是一个重大 Bug 的根源）**：

```
正确设置：learn_z_min = height_filter.z_max（如 -0.5）

含义：只学习 z ≥ -0.5（地面/低空）的点作为背景。
     z < -0.5 的空中区域点跳过学习，不会被学入背景模型。

旧代码的错误设置：learn_z_max = height_filter.z_min（-1.5）
错误含义：学习 z ≤ -1.5 的点——恰好是无人机飞行高度区域！
结果：悬停无人机被标记为静态背景 → 下一帧被过滤 → 检测失败 → 坐标漂移
```

**`filter()` 的分区策略**：
```
点云按 z 分成两区：
  空中区域（z < learn_z_min）→ 全部保留，不做背景检查
  地面区域（z ≥ learn_z_min）→ 查背景体素，过滤静态点
```

**`load_from_pcd()` 预加载模式**：
```
若已有预建点云地图（如 data/lab_pcds.pcd），可跳过在线学习：
  1. 从 PCD 加载点云 → ROI 裁剪 + 降采样
  2. 体素化后直接设为已确认背景（计数 = occupy_threshold）
  3. is_ready = True，后续 learn() 自动跳过
  优势：第 1 帧即可检测，无需等待 10 帧学习，启动时视野内可以有无人机
```

---

### 3.4 `cluster_detector.py` — DBSCAN 聚类检测

**职责**：对预处理后的空中点云做 DBSCAN 聚类，输出候选无人机目标列表。

#### `AirTarget` 数据类

```python
@dataclass
class AirTarget:
    center:     np.ndarray   # 中心坐标 [x, y, z]（雷达坐标系）
    bbox_min:   np.ndarray   # 包围盒最小点
    bbox_max:   np.ndarray   # 包围盒最大点
    num_points: int          # 点数
    size:       np.ndarray   # 尺寸 [dx, dy, dz]
    points:     np.ndarray   # 原始点云数组
    confidence: float        # 检测置信度（0~1）
    source:     str          # 来源（"cluster" 或 "loose_query"）
```

#### DBSCAN 的 Z 轴压缩（z_zip 特性）

```
问题：无人机机身在水平面（X/Y 方向）展开，Z 方向很薄。
     标准 DBSCAN 在 XYZ 等权时，Z 方向稀疏使得点云容易被分裂为多个小簇。

解决：对点云做 Z 轴压缩后聚类：
  points_compressed[:, 2] *= z_zip   # 默认 z_zip=0.5，Z 轴缩小一半
  聚类后 center/bbox 使用原始坐标（不受压缩影响）
```

#### Open3D vs sklearn 后端

```
backend: "open3d"  → 使用 Open3D 的 cluster_dbscan()，速度约 10~30ms/帧
backend: "sklearn" → 使用 sklearn 的 DBSCAN，速度约 100~200ms/帧
```

#### 置信度计算 `_compute_confidence(num_points, size)`

```
综合考虑：点数得分 + 尺寸合理性得分 
  pts_score  = (num_points - min_points) / (max_points - min_points)，钳制到 [0,1]
  size_score = 1 - |size - ideal_size| / ideal_size，钳制到 [0,1]
  confidence = (pts_score + size_score) / 2
```

#### `loose_query(pcd, center, expand_xy, expand_z, eps_scale, min_samples_scale)`

```
在目标上次位置附近做宽松聚类搜索：
  1. 在 center ± expand_xy（xy 方向）、± expand_z（z 方向）的长方体内裁剪点云
  2. 临时放大 eps（× eps_scale），减小 min_samples（× min_samples_scale）
  3. 重新聚类，返回找到的子目标列表

作用：跟踪目标短暂消失（遮挡/转向/点数骤降）后，用更宽松条件重新找回它
```

---

### 3.5 `air_kalman_filter.py` — 卡尔曼滤波器

**职责**：对单个跟踪目标做 2D 卡尔曼滤波，状态向量为 `[x, y, vx, vy]`。

**卡尔曼参数设计参考** HITS-radar-2024 `KalmanFilter.cpp` 的以下思路：
- 初始协方差 `P₀ = Q × init_p_times`（而非极小值）
- 动态过程噪声放大：速度越大，Q 越大（`cov_factor`）
- 丢失帧数超过阈值后停止 P 增长，防止矩阵数值爆炸（`stop_p_time`）
- 速度衰减模拟空气阻力（`decay_rate`）

**状态方程**（线性运动模型，含速度衰减）：

```
X_{k+1} = A × X_k

A = [[1, 0,          1, 0         ],
     [0, 1,          0, 1         ],
     [0, 0, 1-decay,  0           ],
     [0, 0,          0, 1-decay   ]]

decay_rate = 1e-4  （速度缓慢衰减，模拟空气阻力）
```

**观测方程**（H = I，直接观测位置和估算速度）：

```
Z_k = [x_obs, y_obs, vx_obs, vy_obs]
vx_obs = (x_obs - x_prev) / (lost_time + 1)   # 位置差估算速度
```

#### P₀ 为什么必须初始化为较大值

```
马氏距离 = sqrt( d^T × (H × P_ × H^T + R)^{-1} × d )

若 P_ 极小（旧代码 P₀ ≈ 1e-7 量级）：
  → H × P_ × H^T ≈ 0
  → 分母只有 R（5e-2 量级）
  → 马氏距离偏大，检测点无法匹配到已有目标
  → 每帧都产生新目标 → ID 每帧跳变 → 坐标漂移

P₀ = Q × init_p_times（如 ×20）：
  初始不确定性大 → 第一帧检测顺利匹配 → 连续跟踪
  之后 P 随观测逐帧收敛到合理量级
```

#### `set_global_params()` 类方法

```python
# 在节点初始化时调用一次，所有 KF 实例共享矩阵
AirKalmanFilter.set_global_params(
    q_pos=1e-7, q_vel=5e-6, q_pv=5e-6,
    r_pos=5e-2, r_vel=5e-2,
    decay_rate=1e-4, max_velocity=10.0,
    cov_factor=2.5, stop_p_time=40, init_p_times=20,
)
```

---

### 3.6 `target_tracker.py` — 多目标跟踪器

**职责**：维护多个 `TrackedAirTarget` 的生命周期，通过马氏距离匹配检测结果与历史目标。

#### `TrackedAirTarget` 数据类

```python
@dataclass
class TrackedAirTarget:
    id:             int              # 唯一跟踪 ID（从 0 递增）
    kf:             AirKalmanFilter  # 卡尔曼滤波器实例
    lost_count:     int              # 连续丢失帧数（0 = 本帧被匹配到）
    total_seen:     int              # 累计被观测帧数
    last_detection: AirTarget        # 最近一次检测结果
    height:         float            # 估计高度 z（雷达坐标系）
    confidence:     float            # 最近置信度
```

#### `update()` 主循环逻辑

```
每帧调用 tracker.update(detections, pcd, detector, loose_query_params)

步骤 1：逐检测目标贪心匹配（_push_single）
  ├── 计算所有跟踪目标与检测目标间的马氏距离
  ├── 检测 AABB 是否包含多个跟踪目标 → 尝试分裂（_try_separate）
  └── 匹配最近目标 → 卡尔曼更新；无匹配 → 新建目标

步骤 2：对短暂丢失目标做松弛查询（_do_loose_query）
  └── 在上次位置周围以宽松参数重新聚类，恢复短暂消失的目标

步骤 3：未被匹配的目标做无观测预测（update_no_observation）
  └── 卡尔曼仅用运动模型推算位置，lost_count++

步骤 4：强制合并过近目标（_force_combine）
  └── 两目标距离 < force_combine_dist 时，保留 total_seen 更大的

步骤 5：删除丢失过久的目标
  └── lost_count > max_lost_frames → 删除

步骤 6：返回已确认的活跃目标
  └── total_seen >= confirm_frames AND lost_count == 0
```

#### 确认窗口（confirm_frames）

```
目的：防止单帧噪声检测被立即发布到小地图。

confirm_frames = 2：
  第 1 帧检测到某位置 → total_seen=1，不发布
  第 2 帧同一位置再次检测到 → total_seen=2，开始发布

单帧噪声（地面反射、电磁干扰等）下一帧消失 → total_seen 永远不会到 2 → 不发布
```

#### 分裂检测（_try_separate）

```
触发条件：一个检测 AABB 内包含多个已有跟踪目标（两架无人机靠近重叠）

处理：
  1. 裁剪出该 AABB 内的局部点云
  2. 临时收紧 eps（×0.5）和 min_pts（×2）重新聚类
  3. 若分裂出 ≥2 个子簇 → 分别作为独立目标匹配
  4. 若分裂失败 → 按常规匹配（视为一个目标）
```

#### 松弛查询（_do_loose_query）

```
触发条件：目标 lost_count ∈ [1, combine_limit]（短暂丢失）

处理：
  1. 在上次位置 ± expand_xy（xy）/ ± expand_z（z）范围内裁剪点云
  2. 用宽松 eps（× eps_scale=1.4）、宽松 min_pts（× 0.6）重聚类
  3. 若找到候选且距离合理 → 直接更新目标，lost_count 清零
```

---

### 3.7 `air_target_node.py` — ROS2 节点

**职责**：将上述所有模块集成为可运行的 ROS2 节点。

#### 关键初始化顺序

```python
# 1. 先注册全局卡尔曼参数（必须在 Tracker 实例化之前）
AirKalmanFilter.set_global_params(...)

# 2. 创建 Tracker
self.tracker = AirTargetTracker(...)

# 3. 背景减除：learn_z_min = height_filter.z_max（只学地面区域）
learn_z_min = air_cfg.preprocessing.height_filter.z_max  # 如 -0.5
self.bg_subtractor = AirBackgroundSubtractor(learn_z_min=learn_z_min)

# 4. 背景预加载（可选，配置了 bg_pcd_file 时自动触发）
#    TF 就绪后：读取 PCD → 逆变换到 livox 帧 → load_from_pcd()
#    跳过在线学习，第 1 帧就能检测
```

#### `process_frame_pcd(pcd)` 接口

```python
locations, stats = node.process_frame_pcd(pcd_o3d)
# 接受 Open3D 点云对象，不依赖文件路径
# 可在集成测试、可视化脚本中直接调用
# 返回 (List[Location], stats_dict)
```

#### TF 变换

```
使用 TF2 查询 livox → map 的变换（由已有 registration 节点提供）。
每秒尝试刷新一次；TF 就绪后：
  1. 若配置了 bg_pcd_file，立即加载预建背景（PCD 逆变换到 livox 帧）
  2. 开始正常检测流程
```

#### 敌我区分

```
赛场 X 坐标分区（对应 map 坐标系）：
  x ∈ [0, field_split_x) → 蓝方半区 → label="Blue"
  x ∈ [field_split_x, 28] → 红方半区 → label="Red"

field_split_x 默认 14.0（赛场中线），可在 main_config.yaml 调整。
```

---

## 4 坐标系说明

```
雷达坐标系（livox frame）：
  X: 前方
  Y: 左方
  Z: 向上为正，向下为负

  激光雷达通常安装在赛场四角或顶部，Z 轴指向地面的点为负值。

  空中目标高度区间（需根据实际安装高度调整）：
    z = -1.5 → 雷达下方 1.5m（较高空域）
    z = -0.5 → 雷达下方 0.5m（较低空域）
  
  地面区间：
    z ≈ 0    → 雷达安装高度平面
    z > 0    → 雷达正上方（几乎不存在点）

赛场坐标系（map frame）：
  X: 沿赛场长边（0~28m）
  Y: 沿赛场短边（0~15m）

TF 变换来源：registration 节点（ICP 配准结果，livox→map）
```

---

## 5 配置参数详解

路径：`configs/main_config.yaml`，段落：`air_target`

### 顶层参数

```yaml
air_target:
  enabled: true                 # 是否启用空中机器人检测
  publish_rate: 10.0            # 发布频率（Hz），建议 10~20Hz
  strict_dual_uav: false        # 模式开关（详见第 6 节）
  max_targets: 0                # 非严格模式时最大目标数；0=不限制
  field_split_x: 14.0           # 赛场中线 X 坐标（用于敌我半区判断）
```

### 预处理参数

```yaml
preprocessing:
  roi:
    x_min/x_max: -1.0 ~ 30.0   # 前后范围（沿雷达 X 轴）
    y_min/y_max: -9.0 ~ 9.0    # 左右范围（沿雷达 Y 轴）
    z_min/z_max: -3.5 ~ 0.5    # 高度范围（粗裁剪）
  voxel_size: 0.05              # 体素降采样（越小精度越高、越慢）
  height_filter:
    z_min: -1.5                 # 空中区域下界（高空方向，负值更大）
    z_max: -0.5                 # 空中区域上界（低空方向）
                                # ⚠️ 需根据实际雷达安装高度标定后调整
  adaptive_height:
    enabled: true               # 是否启用自适应高度收缩
    margin: 0.25                # 在已知目标高度上下各扩展 0.25m
    min_span: 0.5               # 最小区间 0.5m
    max_span: 2.0               # 最大区间 2.0m
```

### 聚类参数

```yaml
clustering:
  eps: 0.3                      # DBSCAN 邻域半径（m）
                                # 太小：无人机点云被分裂为多簇
                                # 太大：多架无人机被合并为一簇
  min_samples: 5                # 成簇最少点数
  z_zip: 0.5                    # Z 轴压缩系数
  backend: "open3d"             # 建议使用 open3d（更快）
  split_merged: true            # 是否尝试分裂疑似合并的大簇
  split_size_xy: 1.0            # 大于此水平尺寸（m）才尝试分裂
  split_min_gap: 0.25           # 分裂要求的最小子簇间距
  split_min_points: 12          # 执行分裂所需的最小点数
```

### 目标筛选参数

```yaml
target_filter:
  min_points: 5                 # 最少点数（过少则视为噪声）
  max_points: 200               # 最多点数（过多则视为地面/人/建筑）
  min_size: 0.1                 # 最小尺寸 m（过小则视为噪声）
  max_size: 1.5                 # 最大尺寸 m（过大则视为非无人机）
  confidence_threshold: 0.15    # 置信度阈值，低于此值丢弃
```

### 跟踪参数

```yaml
tracking:
  enabled: true
  max_lost_frames: 120          # 连续丢失此帧数后删除目标
                                # 10Hz 下 120帧 = 12秒
  match_distance: 6.0           # 马氏距离匹配阈值（无量纲）
  force_combine_dist: 0.5       # 两目标距离< 0.5m时强制合并（m）
  cc_thres: 0.05                # 包围盒邻近容差（m）
  combine_limit: 15             # 允许松弛查询的最大丢失帧数
  separate_limit: 8             # 触发分裂检测的最大丢失帧数
  confirm_frames: 2             # 发布需要的最少连续观测帧数
  kalman:
    q_pos: 1.0e-7               # 位置过程噪声（小=相信运动模型）
    q_vel: 5.0e-6               # 速度过程噪声
    q_pv:  5.0e-6               # 位置-速度交叉噪声
    r_pos: 5.0e-2               # 位置观测噪声（小=相信观测）
    r_vel: 5.0e-2               # 速度观测噪声
    decay_rate: 1.0e-4          # 速度衰减（每帧速度×(1-decay_rate)）
    max_velocity: 10.0          # 最大速度限幅（m/s）
    cov_factor: 2.5             # 动态噪声放大系数（速度越大Q越大）
    stop_p_time: 40             # 丢失超过40帧后停止协方差增长
    init_p_times: 20            # P₀ = Q × 20（初始不确定性大）
```

### 松弛查询参数

```yaml
loose_query:
  enabled: true
  max_lost_frames: 5            # 只对丢失≤5帧的目标做松弛查询
  expand_xy: 1.0                # XY 搜索扩展半径（m）
  expand_z: 0.5                 # Z 轴搜索扩展量（m）
  eps_scale: 1.4                # eps 放大系数
  min_samples_scale: 0.6        # min_samples 缩小系数
```

### 背景减除参数

```yaml
background:
  enabled: true
  voxel_size: 0.15              # 背景体素大小（m）
  occupy_threshold: 5           # 命中5次以上视为静态背景
  learning_frames: 10           # 前10帧用于建立背景模型
  bg_pcd_file: "data/lab_pcds.pcd"  # 预建PCD地图路径（空=""则用在线学习）
```

> 💡 **预加载 vs 在线学习**：设置 `bg_pcd_file` 后，节点启动时会从 PCD 加载背景，第 1 帧就能检测，
> 无需等待学习帧，且启动时视野内可以有无人机。设为空字符串则回退到原始在线学习模式。

---

## 6 敌我区分与 ID 约定

### 模式开关：`strict_dual_uav`

#### `strict_dual_uav: false`（推荐，实验室/调试阶段）

```
所有检测到的无人机都发布，每个跟踪目标有独立 ID：
  label="Red"  → ID = 600 + (track_id % 400)
  label="Blue" → ID = 1600 + (track_id % 400)

优点：
  - 可同时显示多架无人机（实验室可能同时存在多台）
  - 可观察哪些是真实无人机，哪些是被误检的物体

max_targets: 0  → 不限数量
max_targets: 4  → 最多发布4个（按置信度排序）
```

#### `strict_dual_uav: true`（比赛模式）

```
最多发布2架（红方1架 + 蓝方1架）：
  红方空中机器人 ID = 6    (RoboMaster 2026 裁判系统协议)
  蓝方空中机器人 ID = 106

选择策略：每半区内置信度最高的跟踪目标作为代表。
若某半区内无跟踪目标，则不发布该方。
```

### 赛场中线调整

```yaml
field_split_x: 14.0   # 赛场 X 轴中线（28m / 2）
```

---

## 7 使用说明（快速上手）

### 7.1 启动检测节点

```bash
# 使用 launch 文件
ros2 launch src/hnurm_bringup/launch/hnurm_air_launch.py

# 或手动启动
source install/setup.bash
ros2 run hnurm_radar air_target_node
```

### 7.2 典型配置场景

**场景1：实验室多机测试（推荐）**
```yaml
air_target:
  strict_dual_uav: false     # 显示所有检测到的目标
  max_targets: 0             # 不限制数量
  confirm_frames: 2          # 连续2帧才发布（防噪声）
```

**场景2：正式比赛**
```yaml
air_target:
  strict_dual_uav: true      # 最多红1蓝1，ID 固定为 6/106
```

**场景3：检测不到无人机（高度设置问题）**
```yaml
preprocessing:
  height_filter:
    z_min: -2.0   # 扩大搜索范围
    z_max: -0.3
```

**场景4：误检太多**
```yaml
target_filter:
  min_points: 10             # 提高点数要求
  confidence_threshold: 0.3  # 提高置信度阈值
tracking:
  confirm_frames: 3          # 更严格的确认要求
```

### 7.3 查看发布结果

```bash
ros2 topic echo /location          # 原始检测坐标
ros2 topic echo /location_ekf      # EKF 平滑后坐标
ros2 topic hz /location            # 确认发布频率
```

### 7.4 背景建模注意事项

```
✅ 推荐方式：使用预建PCD加载背景（bg_pcd_file）
   在 main_config.yaml 中设置 bg_pcd_file 指向预建点云地图（如 data/lab_pcds.pcd），
   节点启动后 TF 就绪即自动加载，第 1 帧就能检测，无任何限制。

⚠️ 回退方式：在线学习（bg_pcd_file 为空时）
   节点启动后前 learning_frames=10 帧（约 1 秒）用于背景建模。
   在此期间请确保：
   1. 激光雷达视野内没有无人机
   2. 场地环境稳定（无人频繁走动）

   背景建模完成后，地面/墙体等静态点自动过滤，检测精度显著提升。
```

---

## 8 调试工具

### 8.1 RViz 实时调试话题（推荐 ✅）

`air_target_node` 在运行时自动发布 3 个调试话题，可直接在 RViz 中查看检测到的聚类点云和包围盒：

| 话题 | 类型 | 说明 |
|------|------|------|
| `air_debug/air_points` | `PointCloud2` | 高度过滤后的空中区域候选点云 |
| `air_debug/cluster_points` | `PointCloud2` | 聚类后的目标点云（`intensity` 字段 = 聚类 ID） |
| `air_debug/cluster_markers` | `MarkerArray` | 包围盒 + 文本标注（点数、置信度） |

> 类似 HITS 的 `pc_detector/pc_filtered` 和 `pc_detector/markers` 话题。

**RViz 操作步骤**：
1. 重新编译：`colcon build --packages-select hnurm_radar`
2. 启动系统：`ros2 launch hnurm_bringup hnurm_air_launch.py`
3. 打开 RViz：`rviz2`
4. Fixed Frame 设为 **livox**
5. 添加话题：
   - **Add → PointCloud2** → Topic: `/air_debug/cluster_points`，Color Transformer 选 **Intensity**（不同聚类自动着色）
   - **Add → MarkerArray** → Topic: `/air_debug/cluster_markers`（显示包围盒和标注如 `#0 n=35 conf=0.82`）
   - （可选）**Add → PointCloud2** → Topic: `/air_debug/air_points`（查看全部空中区域候选点）

**命令行快速验证**：
```bash
ros2 topic echo /air_debug/cluster_points --once   # 确认有数据
ros2 topic echo /air_debug/cluster_markers --once   # 查看标注文本
ros2 topic hz /air_debug/cluster_points             # 确认发布频率
```

### 8.2 保存/查看聚类 PCD 快照

使用 `scripts/air_save_debug_pcd.py` 可以保存实时检测结果为 PCD 文件，离线逐帧分析。

```bash
# 模式一：订阅 debug 话题保存（需先启动 air_target_node）
python3 scripts/air_save_debug_pcd.py --subscribe --frames 5 --view

# 模式二：直接订阅 /lidar_pcds 原始点云，离线运行检测流程
python3 scripts/air_save_debug_pcd.py --offline --frames 5 --view
```

保存目录默认为 `test_output/air_debug/`，包含：
- `air_points_XXXX.pcd` — 空中区域点云
- `cluster_points_XXXX.pcd` — 全部聚类点云
- `cluster_XXXX_idN.pcd` — 第 N 个聚类的独立 PCD

### 8.3 离线可视化脚本

```bash
# 单帧可视化（Open3D 窗口）
python3 scripts/air_visualize_clusters.py \
    --pcd data/lab_pcds.pcd \
    --show-raw                    # 同时显示原始点云（灰色）

# 多帧序列（含跟踪轨迹）
python3 scripts/air_visualize_clusters.py \
    --pcd data/frame_000.pcd \
    --frames data/frame_*.pcd

# 只看聚类，不启用跟踪
python3 scripts/air_visualize_clusters.py \
    --pcd data/lab_pcds.pcd \
    --no-track
```

> ⚠️ 注意：`lab_pcds.pcd` 是 1900 万点的完整地图，不是单帧数据。
> 背景减除需要多帧学习才能工作，建议使用 `--offline` 模式或 RViz 实时查看。

**颜色说明**：
| 颜色 | 含义 |
|------|------|
| 灰色点云 | `--show-raw` 时的原始点云 |
| 中灰色点云 | 高度过滤后的空中区域点 |
| 彩色点云（红/蓝/绿…） | 各聚类簇 |
| 彩色 AABB | 对应簇的包围盒 |
| 黄色小球 | 卡尔曼估计的目标中心（只在跟踪模式下显示）|

### 8.4 参数网格搜索

```bash
python3 scripts/air_param_grid_search.py \
    --pcd data/lab_pcds.pcd \
    --eps_list 0.2 0.3 0.4 0.5 \
    --min_samples_list 3 5 7 10
```

### 8.5 跟踪鲁棒性验证

```bash
python3 scripts/validate_air_tracking_sequence.py \
    --pcd_dir /path/to/recorded_frames \
    --glob "*.pcd"
# 输出：平均检测数、平均跟踪数、丢失事件数
```

---

## 9 常见问题与排查

### Q1：小地图上无人机标识一直漂动/跳变

按概率排序，逐项检查：

1. **卡尔曼初始协方差设置**：确认 `init_p_times: 20` 而非极小值
2. **背景减除方向**：确认代码中 `learn_z_min = height_filter.z_max`（非 `z_min`）
3. **确认窗口**：确认 `confirm_frames: 2`
4. **TF 变换稳定性**：`ros2 run tf2_ros tf2_echo map livox`，观察输出是否连续抖动
5. **高度过滤是否覆盖到地面点**：用可视化脚本检查 `--show-raw` 后肉眼确认

### Q2：检测不到无人机

1. 运行 `air_visualize_clusters.py --show-raw`，确认原始点云中能看到无人机点
2. 检查 `height_filter.z_min/z_max` 是否覆盖了无人机实际飞行高度
3. 降低 `min_points`（试设为 3）
4. 增大 `eps`（试设为 0.4~0.5）
5. 检查背景减除是否正常：若用在线学习模式，启动时无人机是否还在场地内（推荐改用 `bg_pcd_file` 预加载）

### Q3：误检太多

1. 提高 `min_points`（5 → 8 → 10）
2. 缩小 `max_size`（1.5 → 1.0）
3. 提高 `confidence_threshold`（0.15 → 0.3）
4. 提高 `confirm_frames`（2 → 3 或 4）
5. 确认背景减除在正常运行（推荐使用 `bg_pcd_file` 预加载，或确保在线学习时视野内无运动物体）

### Q4：实验室有多台无人机，如何同时显示

```yaml
strict_dual_uav: false
max_targets: 0             # 不限制数量
```

各目标按位置所在半区分配 ID（600+N 或 1600+N），在小地图上显示不同颜色。

### Q5：卡尔曼参数调整方向

```
q_pos 小（如 1e-7）：相信运动模型，轨迹平滑但响应慢
q_pos 大（如 1e-4）：不相信运动模型，响应快但可能抖动

r_pos 小（如 1e-3）：相信激光雷达测量，直接跟随检测
r_pos 大（如 0.5） ：不相信测量，更依赖运动预测

实际建议：
  悬停/低速无人机 → 适当减小 q_pos，轨迹更平滑
  快速机动无人机 → 适当增大 q_pos，响应更快
  观测噪声大（点云抖动）→ 适当增大 r_pos
```

---

## 10 参数调优指南

### 步骤 1：录制点云序列

```bash
ros2 bag record /lidar_pcds -o lab_session_1
```

### 步骤 2：网格搜索最优聚类参数

```bash
python scripts/air_param_grid_search.py \
    --pcd data/lab_pcds.pcd \
    --eps_list 0.2 0.25 0.3 0.35 0.4 \
    --min_samples_list 3 4 5 6 8
```

选择使检测数最接近已知无人机数量的参数。

### 步骤 3：多帧跟踪验证

```bash
python scripts/validate_air_tracking_sequence.py --pcd_dir /path/to/frames/
```

观察 `平均跟踪数` 是否稳定，`丢失事件数` 是否偏高。

### 典型场景参数参考

| 场景 | eps | min_samples | z_min | z_max |
|------|-----|-------------|-------|-------|
| 实验室低空悬停（≤0.5m高）| 0.3 | 5 | -0.8 | -0.3 |
| 实验室中高空（1~1.5m）| 0.3 | 4 | -1.5 | -0.5 |
| 赛场全场覆盖 | 0.3 | 5 | -1.5 | -0.3 |
| 多机混飞区分 | 0.25 | 5 | -1.5 | -0.3 |

---

## 11 已修复的历史 Bug

本文档对应的代码版本对原始 air_scheme 实现修复了以下问题：

| # | Bug 描述 | 影响 | 修复方式 |
|---|---------|------|---------|
| 1 | `AirBackgroundSubtractor` 用 `learn_z_max=z_min`，把无人机悬停位置学为背景 | 悬停无人机被过滤，产生漂移 | 改为 `learn_z_min=z_max`，只学地面区域 |
| 2 | `AirKalmanFilter` 初始协方差 `P₀≈0`，马氏距离计算异常 | 每帧建新目标，ID 跳变，坐标漂移 | `P₀ = Q × init_p_times`（×20）|
| 3 | `tracker.update()` 无确认窗口，单帧噪声立即发布 | 噪声点随机出现在小地图上 | 增加 `confirm_frames=2` |
| 4 | `tracker.update()` 不接受 `pcd` 和 `detector` 参数 | 松弛查询/分裂功能无法调用 | 接口扩展为 `update(detections, pcd, detector)` |
| 5 | 旧代码无 `process_frame_pcd(pcd)` 接口 | 集成测试、离线验证脚本无法调用 | 新增该方法，接受 Open3D 点云对象 |
| 6 | 无敌我区分逻辑，所有目标用相同 ID | 无法区分红蓝方，裁判系统对接困难 | 按赛场 X 坐标半区分配 label 和 ID |

---

## 12 本项目 air_scheme 与 HITS-radar-2024 全面对比

> HITS-radar-2024（哈尔滨工业大学深圳 HITS 战队 2024 赛季雷达站开源方案）是本项目空中目标检测方案的主要参考。
> 以下逐模块列出所有相同点和不同点。

### 12.1 总体架构对比

| 对比维度 | HITS-radar-2024 | 本项目 air_scheme |
|----------|----------------|-------------------|
| **语言** | C++ (ROS2 Humble) | Python 3.10 (ROS2 Humble) |
| **目标类型** | 地面机器人（全车种） | 空中机器人（无人机） |
| **状态维度** | 2D (x, y) + Z 投影地面高度图 | 2D (x, y) + 直接使用点云 z 坐标 |
| **坐标系** | 赛场坐标系 (world frame)，点云先变换再处理 | 雷达坐标系 (livox frame) 处理，发布时才变换到 map |
| **节点数** | 单节点 (`pc_detector`) 完成全部处理 | 多节点：`lidar_node` + `air_target_node` + `ekf_node` |
| **依赖** | Open3D (C++)、Eigen、TBB | Open3D (Python)、NumPy |
| **ROS2 接口** | Component Node（可组合） | 独立 Python Node |
| **构建系统** | CMake + ament_cmake | setuptools + ament_python |

### 12.2 点云预处理对比

| 对比项 | HITS | 本项目 | 说明 |
|--------|------|--------|------|
| **背景模型** | 3D STL Mesh（赛场模型） + 射线投射 | 运行时体素网格自动学习 | HITS 用已知赛场模型做射线-体素遮挡过滤，离线构建；本项目在线学习背景 |
| **背景判定** | `is_occupied()` 布尔网格，被 mesh 遮挡的体素视为背景 | 体素命中计数 ≥ `occupy_threshold` 视为背景 | HITS 是几何遮挡；本项目是统计频率 |
| **VoxelGrid 分区** | 不分区，所有点统一过滤 | 按 `learn_z_min` 分区：空中区域全保留，地面区域过滤 | 本项目针对空中目标特化 |
| **ROI 裁剪** | VoxelGrid 范围隐式裁剪（超出范围视为被占据） | 显式 NumPy 布尔索引裁剪（ROI x/y/z 范围） | 效果等价 |
| **体素降采样** | 收到点云后直接处理（C++ 层面快） | Open3D `voxel_down_sample(0.05)` | HITS 无显式降采样，靠 VoxelFilter 限频率 |
| **高度过滤** | 无高度过滤（地面机器人无需） | Z 轴区间过滤 `[z_min, z_max]` + 自适应 | 本项目特有：只保留飞行高度范围的点 |
| **自适应高度** | 无 | 根据已跟踪目标高度动态收缩过滤范围 | 本项目特有 |
| **多帧累积** | `pc_buffer` 累积 N 帧点云后一次聚类 | 可选 `buffer.enabled` 累积多帧 | HITS 默认 100 帧(约 10s)；本项目默认关闭 |
| **TF 坐标变换** | 收到点云立即变换到 world 坐标系 | 聚类在雷达坐标系完成，发布时才变换 | HITS 在预处理阶段就变换，本项目延迟到输出阶段 |
| **Z 投影** | Z_Map：利用 STL mesh 射线投射建立赛场地面高度图 | 直接使用点云 z 坐标（不投影地面） | HITS 输出的 z 是地面高度而非实际 z |

### 12.3 DBSCAN 聚类对比

| 对比项 | HITS | 本项目 | 说明 |
|--------|------|--------|------|
| **Z 轴压缩** | ✅ `z_zip=0.5`，Transform 矩阵实现 | ✅ `z_zip=0.5`，NumPy 数组乘法 | 相同思路 |
| **聚类实现** | 手写 `NormalDBSCAN`（BFS + BucketSet） | Open3D `cluster_dbscan()` 或 sklearn | HITS 自实现 DBSCAN，TBB 并行邻域搜索，效率高 |
| **距离加权聚类** | ✅ `DifferingDBSCAN`：min_points 与到雷达距离平方成反比 | ❌ 无 | HITS 远处目标自动降低点数门槛 |
| **三级参数** | `normal.eps=0.2`、`loose.eps=0.25`、`strict.eps=0.15` | 单级 `eps=0.3` + 松弛查询时临时放大 | HITS 三级固定参数；本项目运行时动态缩放 |
| **点数默认值** | `normal.min_points=8`，`loose=5`，`strict=12` | `min_samples=5`（松弛时 ×0.6=3，分裂时 ×2=10） | HITS 默认更大（地面车体积大） |
| **eps 默认值** | 0.2（normal） | 0.3 | 本项目空中目标点稀疏，需更大 eps |
| **大簇分裂** | ❌ 无（由 `seperate` 在跟踪层处理） | ✅ `split_merged`：投影间隙主成分分析分裂 | 本项目增加了预聚类阶段的主动分裂 |
| **目标置信度** | ❌ 无 | ✅ 基于点数和尺寸计算 0~1 置信度 | 本项目特有 |
| **目标筛选** | 无显式筛选（所有聚类都进入跟踪） | 点数/尺寸/置信度三重过滤 | 本项目更严格 |

### 12.4 卡尔曼滤波器对比

| 对比项 | HITS `KalmanFilter.cpp` | 本项目 `air_kalman_filter.py` | 说明 |
|--------|-------------------------|-------------------------------|------|
| **状态向量** | `[x, y, vx, vy]` (4D) | `[x, y, vx, vy]` (4D) | ✅ 相同 |
| **Q 矩阵** | `[[q_pos, 0, q_pv, 0], ...]` | 相同结构 | ✅ 相同 |
| **R 矩阵** | `diag(r_pos, r_pos, r_vel, r_vel)` | 相同 | ✅ 相同 |
| **A 矩阵** | `[[1,0,1,0],[0,1,0,1],[0,0,1-decay,0],[0,0,0,1-decay]]` | 相同 | ✅ 相同 |
| **P₀ 初始化** | `P = Q * p_times`（默认 `p_times=0` → P₀=0） | `P = Q * init_p_times`（默认 20） | ⚠️ **关键差异**：HITS 默认 `p_times=0`（P₀=0）；本项目修正为 20 |
| **动态 Q** | `real_Q[:2,:2] += v*v^T * cov_factor` | 相同公式 | ✅ 相同 |
| **stop_p_time** | `lost_time > stop_p_time` 时跳过 P 更新 | 相同逻辑 | ✅ 相同 |
| **速度限幅** | `cwiseMax/cwiseMin` | `np.clip` | ✅ 相同 |
| **速度衰减** | A 矩阵中 `1-decay_rate` | 相同 | ✅ 相同 |
| **观测速度** | `(Z - X[:2]) / (lost_time + 1)` | 相同公式 | ✅ 相同 |
| **K 计算** | `K = P_ * (P_ + R)^{-1}` (H=I) | 相同 | ✅ 相同 |
| **马氏距离** | `d^T * P[:2,:2]^{-1} * d`（用 P） | `d^T * P_[:2,:2]^{-1} * d`（用 P_） | ⚠️ 本项目用预测协方差 P_ 做马氏匹配 |
| **参数注册** | `read_params()` 静态函数设置全局变量 | `set_global_params()` 类方法设置类变量 | ✅ 同一模式 |
| **默认参数值** | q_pos=5e-5, q_vel=2e-5, r_pos=2e-3, cov_factor=1.0, decay=5e-3, stop_p=10 | q_pos=1e-7, q_vel=5e-6, r_pos=5e-2, cov_factor=2.5, decay=1e-4, stop_p=40 | ⚠️ 数值量级差异大（见下文说明） |

**默认参数差异说明**：

```
HITS 面向地面机器人（速度 ≤5m/s，观测频率高，点云密度大）：
  - q_pos = 5e-5   → 位置过程噪声较大（地面车加减速频繁）
  - r_pos = 2e-3   → 观测噪声极小（点云密度足够，聚类稳定）
  - decay = 5e-3   → 速度衰减大（地面车常急刹、转向）
  - cov_factor = 1.0 → 较小（地面车速度变化平缓）
  - stop_p = 10    → 仅 10 帧后停止协方差增长
  - p_times = 0    → 初始 P=0（HITS 用欧氏距离先粗匹配，P 小无影响）

本项目面向空中无人机（速度变化小，悬停为主，点云极稀疏）：
  - q_pos = 1e-7   → 位置过程噪声极小（无人机悬停/缓慢飞行）
  - r_pos = 5e-2   → 观测噪声较大（点云稀疏，聚类中心抖动）
  - decay = 1e-4   → 速度衰减极小（空中阻力小，惯性大）
  - cov_factor = 2.5 → 较大（补偿突然机动）
  - stop_p = 40    → 40 帧（4 秒）后停止（丢失窗口更长）
  - p_times = 20   → 初始 P 放大（纯马氏距离匹配需要合理初始不确定性）
```

### 12.5 多目标跟踪对比

| 对比项 | HITS `TargetMap` | 本项目 `AirTargetTracker` | 说明 |
|--------|-----------------|---------------------------|------|
| **匹配算法** | 贪心马氏距离匹配 | 贪心马氏距离匹配 | ✅ 相同策略 |
| **距离阈值** | `dist_thres=20.0`（马氏平方） | `match_distance=6.0`（马氏平方根） | HITS 比较平方值（400），本项目比较平方值（36） |
| **AABB 重叠检测** | `is_coincide()`（xy 二维） | `_aabb_coincide()`（xy 二维） | ✅ 相同 |
| **包含检测** | `is_contain()`（AABB 内含卡尔曼位置） | `_aabb_contains_pos()` | ✅ 相同 |
| **强制合并** | `combine_force()`：删除除 new_id 外的目标 | `_force_combine()`：保留 total_seen 更多的 | ⚠️ 合并策略不同 |
| **普通合并** | `combine()`：被合并目标需 `lost_time < combine_limit` | 无独立普通合并 | 本项目将此逻辑合并到 `_push_single` |
| **分裂检测** | `seperate()`：用 `strict.eps` 重聚类 | `_try_separate()`：用 `eps*0.5` + `min_pts*2` 重聚类 | 思路相同，参数来源不同 |
| **松弛查询** | `loose_query()`：对丢失目标在旧 AABB 扩展区域用 `loose.eps` 重聚类 | `_do_loose_query()`：在卡尔曼位置周围矩形区域用 `eps*1.4` 重聚类 | ⚠️ 搜索区域定义不同：HITS 用 AABB 扩展，本项目用卡尔曼估计位置 |
| **丢失处理** | `lost_time++`（无观测 KF 预测） | `lost_count++` + `kf.update_no_observation()` | ✅ 相同 |
| **目标删除** | `lost_time > last_frames`（默认 100） | `lost_count > max_lost_frames`（默认 120） | ⚠️ 阈值不同（但思路相同） |
| **确认窗口** | ❌ 无（新目标立即输出） | ✅ `confirm_frames=2` | 本项目增加 |
| **Z 高度估计** | Z_Map 地面射线投射 + `project_z` 偏移 | 直接使用点云聚类中心 z | ⚠️ HITS 投影到地面，本项目用实际 3D 高度 |
| **目标 ID** | 自增 `inc_id`（从 0 起） | 自增 `next_id` + 赛场半区编码 | 本项目带敌我区分 |
| **初始 lost** | `init_lost=12`（新目标初始 lost_time=0） | 无此参数（新目标 lost=0） | HITS 有 init_lost 参数但实际值为 0 |
| **数据结构** | `unordered_map<int, Target>` | `Dict[int, TrackedAirTarget]` | 等价 |
| **combine_dist** | 8.0（马氏距离平方） | 无此参数 | HITS 有基于马氏距离的普通合并 |
| **force_combine_dist** | 0.1（欧氏距离） | 0.5（欧氏距离） | ⚠️ 本项目阈值更大 |
| **cc_thres** | 0.05（AABB 重叠容差） | 0.05 | ✅ 相同 |
| **separate_limit** | 8 | 8 | ✅ 相同 |
| **combine_limit** | 15 | 15 | ✅ 相同 |

### 12.6 可视化与调试对比

| 对比项 | HITS | 本项目 |
|--------|------|--------|
| **调试点云话题** | `pc_detector/pc_filtered` (PointCloud2)，含 `clustered_label` + `tracking_id` 字段 | `air_debug/cluster_points` (PointCloud2)，`intensity` = 聚类 ID |
| **MarkerArray** | `pc_detector/markers`：卡尔曼球体(蓝) + 聚类重心球体(红) + 文本 ID | `air_debug/cluster_markers`：AABB 半透明包围盒(彩色) + 文本(点数+置信度) |
| **输出消息** | `radar_interface::msg::TargetArray`（含位置、速度、协方差完整信息） | `detect_result::msg::Locations`（仅位置+ID+标签） |
| **离线工具** | 无 | `air_visualize_clusters.py`、`air_save_debug_pcd.py`、`air_param_grid_search.py` |

### 12.7 本项目独有功能

| 功能 | 说明 |
|------|------|
| **敌我区分** | 按赛场 X 坐标半区分配 label="Red"/"Blue" 和裁判系统 ID (6/106) |
| **严格双机模式** | `strict_dual_uav=true` 时最多发布 2 架（红1蓝1） |
| **确认窗口** | `confirm_frames=2`，连续 N 帧观测才发布，防单帧噪声 |
| **自适应高度过滤** | 根据已跟踪目标高度动态收缩 z 过滤范围 |
| **目标置信度** | 基于点数和尺寸合理性计算 0~1 置信度并排序 |
| **主成分分裂** | 投影间隙分析分裂疑似合并的大簇 |
| **背景双模式** | 支持预建PCD加载（即时就绪）和在线学习（无需赛场 STL 模型）双模式 |
| **分区背景过滤** | 空中区域点直接保留，只过滤地面区域背景 |
| **EKF 下游** | 检测结果经 `ekf_node` 二次平滑后输出 |
| **离线调试工具** | PCD 快照保存、参数网格搜索、跟踪验证脚本 |
| **RViz 实时调试** | 包围盒 + 文本标注 + 聚类着色点云 |

### 12.8 HITS 独有功能

| 功能 | 说明 |
|------|------|
| **C++ 高性能** | 全 C++ 实现 + TBB 并行，适合高帧率场景 |
| **STL Mesh 背景** | 利用赛场 3D 模型做射线遮挡过滤，精度高 |
| **Z_Map 地面投影** | 将卡尔曼 XY 位置投影到赛场地面高度图，输出准确 z 坐标 |
| **距离加权 DBSCAN** | `DifferingDBSCAN`：min_points 与到雷达距离²成反比 |
| **三级聚类参数** | normal / loose / strict 三套 eps + min_points |
| **完整输出消息** | `TargetArray` 含位置、速度、协方差矩阵完整信息 |
| **Component Node** | 可组合节点，支持零拷贝进程内通信 |
| **多雷达融合** | 支持多个激光雷达同时接入融合 |
| **VoxelFilter 限频** | 通过采样率控制进入聚类的点数 |

---

## 13 当前问题诊断与优化计划

> 更新日期：2026-03-07
> 本章整合了团队讨论的 16 个问题点和规则手册 V1.3.0 数据，给出完整的问题分析、解决方案和行动路线。


### 13.1 规则手册关键数据摘要（V1.3.0）

> 以下数据全部来自 RoboMaster 2026 机甲大师超级对抗赛比赛规则手册 V1.3.0 和机器人制作规范手册 V1.3.0。

#### 赛场尺寸

| 项目 | 数值 |
|------|------|
| 赛场长×宽 | 28m × 15m |
| 围挡高度 | 2.4m（黑色钢制围挡） |
| 地面材质 | 木质结构，PVC 夹线地胶 |
| 赛场对称性 | 中心对称布局 |
| 地面倾斜 | 中轴线向两侧倾斜 1°~2° |

#### 空中机器人制作参数

| 项目 | 限制 | 备注 |
|------|------|------|
| 最大伸展尺寸 | 1700×1700×800mm | **不包括竖直刚性保护杆** |
| 最大收纳尺寸 | 1400×1400×1100mm | 不包括保护杆 |
| 最大重量 | 13kg | 含电池，不含裁判系统 |
| 裁判系统重量 | ~0.66kg（+激光检测≤400g） | — |
| 发射机构 | 1 个 17mm | — |
| 最大供电电压 | 52V | — |
| 最大供电容量 | 900Wh | — |

#### 飞行规则

| 项目 | 规则 |
|------|------|
| **最低飞行高度** | 最低点距地面 ≥ **1.5m** |
| **最高飞行限制** | 17mm 测速模块不可超过围挡最高处（**2.4m**） |
| **安全绳长度** | **2.4m**（回弹力恒定软线） |
| **卡环位置** | 距己方战场宽边约 **14m** |
| 飞行区域 | 停机坪及上方空域 + 梯形高地连接公路上方空域 |
| 桨叶保护罩 | **必须全覆盖**，42mm弹丸不可穿过网孔 |
| 保护杆 | 顶部距桨叶重心平面 350±50mm，最外沿距动力中心 <75mm |
| 航行外观灯 | 灯带总长 >1500mm，朝上安装，红蓝可切换 |
| 不得飞出赛场 | R50 明确规定 |

#### 雷达基座

| 项目 | 数值 |
|------|------|
| 平台面积 | 3.4m × 1.16m |
| 平台高度 | 距地面约 **2.5m** |
| 围栏高度 | 1.1m |
| 位置 | 平面中央与赛场中心轴线对齐 |

#### 雷达识别精度要求

| 等级 | 误差 d |
|------|--------|
| **准确** | d < 0.8m |
| 半准确 | 0.8m ≤ d < 1.6m |
| 错误 | d ≥ 1.6m |

> **关键推算**：雷达基座高 2.5m + 围栏 1.1m = 传感器约在 **3.0~3.5m** 高度。
> 空中机器人飞行高度 1.5m~2.4m。传感器到无人机的垂直距离约 **0.6~2.0m**。
> 在雷达坐标系中（Z 向下为负），无人机 Z 值大约在 **-0.6 到 -2.0** 之间。

#### 能量机关

| 项目 | 说明 |
|------|------|
| 位置 | 赛场中央，装配区正上方 |
| 结构 | 5 个均匀分布灯臂，电机驱动旋转 |
| 灯臂实际圆靶最大直径 | 308mm |
| **干扰风险** | 旋转灯臂可能在点云中产生动态目标，需在赛场中央区域做特殊过滤 |

### 13.2 当前核心问题 A：误识别

**现象**：air_debug 话题中 cluster_markers 显示了多个包围盒，但对应点云是墙壁边缘、天花板横梁、线缆等**固定结构物**。

**根因分析**：

| 误识别来源 | 为何通过了现有过滤 | 严重程度 |
|---|---|---|
| 背景建模不完整 | 在线学习只学地面区域（z >= z_max），空中高度的静态结构从未被学为背景 | ⭐⭐⭐⭐⭐ |
| 预加载PCD背景范围不足 | 预建PCD覆盖了全部高度，但体素大小0.15m可能遗漏细小结构 | ⭐⭐⭐ |
| DBSCAN 参数宽松 | eps=0.3, min_samples=5，远距离5个噪声点就能成簇 | ⭐⭐⭐ |
| 置信度阈值太低 | confidence_threshold=0.15，6个点也能通过 | ⭐⭐ |
| 无形状/运动学验证 | 只看点数和尺寸，不看形状、运动特征、反射率 | ⭐⭐⭐⭐（根本原因）|

**已完成的优化**：
- `background_subtractor.py` 的 `learn()` 和 `filter()` 方法已向量化优化，消除了逐点 Python for 循环，性能提升 5-10 倍
- 配置文件 `main_config.yaml` 已为所有参数补充了详细中文注释和调参说明


### 13.3 当前核心问题 B：漏识别

**现象**：无人机在空中但 air_debug/air_points 看不到点云，或点云过少（<5）无法聚类。

| 漏识别原因 | 说明 | 解决方向 |
|---|---|---|
| 高度范围不匹配 | 无人机实际 Z 值可能不在 [-1.5, -0.5] 内 | 用 RViz Publish Point 工具实测 Z 值后调整 |
| 体素降采样过度 | voxel_size=0.05 在远距离可能合并仅有的几个点 | 远距离目标可降至 0.03 |
| 背景 PCD 误吞 | 悬停位置与 PCD 高点体素重合 → 被当背景过滤 | 减小背景体素 voxel_size 或增大 PCD 降采样 |
| 点云密度随距离衰减 | Livox HAP 15m+ 空中目标可能仅 1-3 点/帧 | 启用多帧缓冲 buffer |
| 多帧缓冲未启用 | buffer.enabled=false，无法累积多帧 | 开启 buffer，3-5帧叠加 |

---

## 14 详细 Q&A：逐条解答（2026-03-07 更新）

> 本章逐条回答团队讨论中提出的 16 个具体问题。每个回答包含：问题描述、原理解释、解决方案、操作步骤。


### Q1：RViz 中点击点云看不到坐标，如何操作？

**问题**：在 RViz 中点击点云想查看 Z 坐标值，但点击后 "Clicked" 面板什么都没有。

**原因**：RViz 默认的鼠标交互模式是旋转视角，不是选点。你需要使用 **Publish Point** 工具。

**详细操作步骤**：

```
1. 打开 RViz2：
   rviz2

2. 设置 Fixed Frame 为 "livox"（顶部 Global Options → Fixed Frame）

3. 添加点云显示：
   左侧面板 → Add → By topic → 选择 /air_debug/air_points (PointCloud2)
   或者选择 /lidar_pcds 查看全部原始点云

4. 添加 Publish Point 工具（关键步骤！）：
   顶部工具栏 → 点击 "+" 按钮（或菜单 Panels → Add New Tool）
   → 选择 "Publish Point"
   → 确认工具栏上出现了一个十字准星图标

5. 点击工具栏上的 "Publish Point" 图标（十字准星），切换到选点模式

6. 现在用鼠标左键点击点云上的任意一点：
   → RViz 底部状态栏会显示该点的 (x, y, z) 坐标
   → 同时该坐标会发布到 /clicked_point 话题

7. 也可以在终端查看：
   ros2 topic echo /clicked_point
   → 每次点击都会输出 geometry_msgs/PointStamped，包含 x, y, z
```

**替代方法**（更精确）：
```bash
# 直接用命令行查看点云的 Z 值统计
ros2 topic echo /air_debug/air_points --once | grep -A3 "data"

# 或者用 Python 脚本打印特定区域的 Z 值范围
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('data/lab_pcds.pcd')
pts = np.asarray(pcd.points)
print(f'Z range: [{pts[:,2].min():.3f}, {pts[:,2].max():.3f}]')
print(f'Z mean: {pts[:,2].mean():.3f}')
"
```

**测量无人机高度的推荐流程**：
1. 将无人机悬停在预期飞行高度（或用杆子举到该高度）
2. 用 Publish Point 工具点击无人机点云上的几个点
3. 记录 Z 值范围，例如 [-1.2, -0.8]
4. 在 `main_config.yaml` 中设置：`z_min` = 最小值 - 0.3 余量, `z_max` = 最大值 + 0.3 余量


### Q2：背景建模不完整 — 如何把整个点云地图都设为背景？

**问题**：背景减除只学了地面区域，空中高度范围的静态结构（横梁、灯架）从未被学为背景。希望把实验室/赛场的完整点云地图都默认设为背景。

**解答**：

**当前代码已经支持这个功能！** 关键配置是 `bg_pcd_file`。

**工作原理**：
```
background_subtractor.py 的 load_from_pcd() 方法：
  1. 读取预建 PCD 文件（如 data/lab_pcds.pcd）
  2. 默认 use_all_points=True → 不受 learn_z_min 限制
  3. PCD 中所有高度的点全部作为背景（因 PCD 是纯静态场景）
  4. 每个体素直接设为 occupy_threshold（确认为背景）
  5. is_ready = True → 后续 learn() 被跳过

  也就是说：PCD 文件中包含的所有点（地面+空中静态结构）都会被设为背景。
  横梁、灯架、墙壁，只要在 PCD 采集时被扫到，就会成为背景。
```

**你需要确保的是**：
1. PCD 文件是在 **无人机不在场** 时采集的（否则无人机也被学进背景）
2. PCD 文件覆盖了实验室/赛场的 **完整静态结构**（含空中横梁等）
3. 配置路径正确：

```yaml
# configs/main_config.yaml
air_target:
  background:
    bg_pcd_file: "data/lab_pcds.pcd"    # 实验室
    # bg_pcd_file: "data/pcds.pcd"       # 比赛时改为赛场 PCD
```

**如果仍有遗漏结构怎么办**：
- 原因：PCD 采集时激光雷达可能没扫到某些角度的结构
- 解决：在不同角度多次采集 PCD，合并后作为背景
- 或者：在线学习模式下运行几分钟（无人机不在场），让系统补充学习遗漏结构

**比赛时操作流程**：
```
1. 到达赛场后，用激光雷达采集赛场完整点云 → 保存为 data/pcds.pcd
2. 确保采集时没有机器人在场（或至少空中没有无人机）
3. 修改 bg_pcd_file 路径指向赛场 PCD
4. 启动系统 → 第 1 帧就能用完整背景过滤
```


### Q3：高度过滤范围 1m 是否过宽？

**问题**：z_min=-1.5, z_max=-0.5 覆盖 1.0m 跨度，包含了很多结构物高度。但测试无人机大概 0.5m 高、1m 长。

**解答**：

1m 跨度对于当前测试场景是**合理的**。原因：
- 无人机本身高度约 0.3~0.5m（不含保护杆）
- 悬停时有上下漂移（±0.1~0.2m）
- 不同测试高度需要覆盖
- 加上上下余量各 0.2~0.3m，1m 是合理的

**但需要根据实际测量调整**：
- 如果你的无人机实际 Z 值在 [-1.2, -0.8]，那 [-1.5, -0.5] 有 0.3m 上下余量，OK
- 如果无人机 Z 值在 [-0.8, -0.4]，你应该改为 [-1.1, -0.1]
- **关键：先用 Q1 的方法测量实际 Z 值，再调整**

**赛场情况**：
```
雷达基座高度 ~2.5m + 传感器高度 ~0.5m = 传感器约 3.0m
无人机飞行高度: 1.5m ~ 2.4m（规则手册）
传感器到无人机垂直距离: 3.0 - 2.4 = 0.6m 到 3.0 - 1.5 = 1.5m
在雷达坐标系（Z 向下为负）: z ∈ [-1.5, -0.6]
跨度 0.9m，加余量取 [-1.8, -0.3] = 1.5m 跨度，这是赛场推荐值
```

### Q4：ROI 范围几乎覆盖全场，是否过大？

**问题**：x: [-15, 15], y: [-3, 28] 几乎覆盖全场，远距离点云密度极低但仍可能形成虚假小簇。赛场和实验室都大概 28m×15m，比赛时不也要覆盖全场吗？

**解答**：

**是的，比赛时确实需要覆盖全场**。ROI 不应该是主要的过滤手段——背景减除和聚类参数才是。

**当前 ROI 设置说明**：
- 这里的坐标是**雷达坐标系**（不是赛场坐标系），取决于雷达安装位置和朝向
- x: [-15, 15] 和 y: [-3, 28] 对应雷达视角下的空间范围
- 这些值需要在 TF 标定后根据实际雷达视野调整

**实验室阶段建议**：
```yaml
# 如果你只在距雷达 6-7m 处测试，可以暂时缩小 ROI 减少计算量
roi:
  x_min: -5.0    # 缩小到测试区域
  x_max: 10.0
  y_min: -5.0
  y_max: 5.0
```

**比赛时恢复全场覆盖**：
```yaml
roi:
  x_min: -15.0   # 全场覆盖
  x_max: 15.0
  y_min: -3.0
  y_max: 28.0
```

**关键认识**：ROI 只是第一层粗过滤，真正消除远距离虚假簇的手段是：
1. 背景 PCD 覆盖全场 → 远距离静态结构被过滤
2. 多帧缓冲 → 累积帧后，静态噪声和真实目标密度差异更明显
3. 确认窗口 confirm_frames → 偶发噪声簇无法连续出现 3 帧
4. 未来：距离自适应 min_samples（类似 HITS 的 DifferingDBSCAN）


### Q5：DBSCAN 参数怎么设置和调试？

**问题**：eps=0.3, min_samples=5，不太理解这些参数。怎么调试测试然后设置最好的参数？

**通俗解释**：

```
DBSCAN 算法的核心概念：

想象你在一个房间里撒了一把豆子（=点云中的点）。
DBSCAN 会做这件事：

  1. 随机选一粒豆子
  2. 以它为圆心，画一个半径 = eps 的圆（三维空间是球）
  3. 数一下圆里有多少粒豆子
  4. 如果 >= min_samples 粒 → 这粒豆子是"核心点"，和圆里的其他豆子属于同一堆
  5. 继续从圆里的其他核心点扩展...
  6. 最终所有互相连接的核心点和它们圆里的点组成一个"簇"（cluster）
  7. 没有被任何圆覆盖的孤立豆子 → 噪声，丢弃

  eps（邻域半径）：
    eps=0.3m → 两个点距离 < 0.3m 才算邻居
    越大 → 越容易把远的点归为同一簇（可能把两架无人机合成一个）
    越小 → 越容易把同一无人机的点分成多个簇

  min_samples（最少邻居数）：
    min_samples=5 → 一个核心点的 0.3m 半径内至少要有 5 个点
    越大 → 抗噪能力越强（但远距离目标点少时检测不到）
    越小 → 检测灵敏（但噪声也容易成簇）
```

**调试方法（推荐按顺序做）**：

```
方法 1：RViz 视觉调试（最直观）
  1. 启动系统，在 RViz 中显示 air_debug/cluster_markers
  2. 摆放无人机
  3. 观察：
     - 无人机被分成多个簇？ → eps 太小，增大到 0.35-0.4
     - 无人机和旁边物体被合成一个簇？ → eps 太大，减小到 0.2-0.25
     - 远距离无人机检测不到？ → min_samples 太大，降到 3-4
     - 空场时有虚假簇？ → min_samples 太小，增大到 8-10

方法 2：参数网格搜索（自动化）
  python3 scripts/air_param_grid_search.py \
      --pcd data/lab_pcds.pcd \
      --eps_list 0.2 0.25 0.3 0.35 0.4 \
      --min_samples_list 3 4 5 6 8 10

方法 3：后期各位置实测（最终调参）
  在赛场或实验室的不同距离（3m、6m、10m、15m、20m）
  分别放置无人机，记录每个距离下的最佳 eps/min_samples
  → 如果差异大，后续考虑实现距离自适应（DifferingDBSCAN）
```

**当前推荐参数**：

| 场景 | eps | min_samples | 说明 |
|------|-----|-------------|------|
| 实验室近距离(6-7m) | 0.3 | 5 | 当前默认值，合理 |
| 实验室远距离(15m+) | 0.35 | 3 | 远距离点稀疏，需放宽 |
| 赛场全场 | 0.3 | 5 | 配合多帧缓冲使用 |

### Q6：置信度阈值 0.15 是什么意思？

**问题**：confidence_threshold=0.15，即使只有 6 个点也能通过。不理解这个参数。

**通俗解释**：

```
置信度（confidence）是我们自己定义的一个 0~1 的评分，
用来衡量"这个聚类像不像无人机"。

计算公式：
  confidence = 0.6 × 点数评分 + 0.4 × 尺寸评分

点数评分（权重60%）：
  点越多 → 评分越高（线性映射到 0~1）
  min_points=5 时评分为 0，max_points=200 时评分为 1
  例：6个点 → (6-5)/(200-5) ≈ 0.005（几乎为 0）
  例：50个点 → (50-5)/(200-5) ≈ 0.23

尺寸评分（权重40%）：
  最大维度在 [min_size, max_size] 正中间时评分最高（1.0）
  越靠近边界评分越低
  例：最大维度 0.8m，范围 [0.1, 1.5] → 中间偏左 → 约 0.85
  例：最大维度 0.15m，范围 [0.1, 1.5] → 很靠近下界 → 约 0.07

阈值含义：
  confidence_threshold=0.15 → 置信度 > 0.15 才发布
  0.15 很低，意味着：
    - 6个点 + 合理尺寸（评分≈0.005×0.6+0.85×0.4=0.34）→ 通过
    - 6个点 + 极小尺寸（评分≈0.005×0.6+0.07×0.4=0.03）→ 不通过
    - 3个点 → 被 min_points 直接过滤，不计算置信度

实验室调试：0.15（宽松，多检出，方便发现问题）
正式比赛：0.25~0.35（严格，减少误检）
```


#### Q7：是否可以利用空中目标的形状特征、运动学特征或反射率来辅助判断？

**形状特征（PCA 主成分分析）**

空中机器人有几何特征上的约束（规则手册 V1.3.0）：
- 整机尺寸 ≤ 1700mm × 1700mm × 800mm
- 必须安装全包裹桨叶保护罩
- 典型形态：扁平的"盘状"或"方形框架+螺旋桨保护"

对聚类后的点云做 PCA（主成分分析），可以获取三个特征值 λ₁ ≥ λ₂ ≥ λ₃：

```
特征值比例    形态含义
────────────────────────────────
λ₁ ≈ λ₂ >> λ₃   扁平盘状（符合无人机特征）
λ₁ >> λ₂ ≈ λ₃   细长线状（不太可能是无人机）
λ₁ ≈ λ₂ ≈ λ₃   球状（可能是误检噪声）
```

指标定义：
- 线性度（Linearity）：(λ₁ - λ₂) / λ₁
- 平面度（Planarity）：(λ₂ - λ₃) / λ₁  
- 球度（Sphericity）：λ₃ / λ₁

无人机期望：**平面度较高**（0.3~0.7），线性度较低（<0.3），球度较低（<0.3）

实现位置：`cluster_detector.py` 的 `_calculate_confidence()` 方法中可增加 PCA 评分：

```python
import numpy as np

def _pca_shape_score(self, points: np.ndarray) -> float:
    """PCA形态评分：扁平状得分高"""
    if len(points) < 4:
        return 0.5  # 点太少无法做有效PCA
    
    centered = points - points.mean(axis=0)
    cov = np.cov(centered.T)  # 3x3协方差矩阵
    eigenvalues = np.linalg.eigvalsh(cov)  # 升序排列
    eigenvalues = eigenvalues[::-1]  # 改为降序 λ₁ ≥ λ₂ ≥ λ₃
    
    total = eigenvalues.sum()
    if total < 1e-10:
        return 0.0
    
    planarity = (eigenvalues[1] - eigenvalues[2]) / eigenvalues[0]
    linearity = (eigenvalues[0] - eigenvalues[1]) / eigenvalues[0]
    
    # 扁平状（无人机）得分高，线状（虚假目标）得分低
    score = planarity * 0.7 + (1.0 - linearity) * 0.3
    return np.clip(score, 0.0, 1.0)
```

**运动学特征**

空中机器人的运动特征区别于静态虚假目标：
- 速度范围：0.5~3.0 m/s（悬停时接近0，但有抖动）
- 位置抖动：即使悬停也有 ±5~10cm 的晃动（安全绳+气流）
- 加速度：受限于飞控，加速度通常 <5 m/s²

当前 Kalman 滤波器已跟踪速度 `[vx, vy]`，可直接提取：

```python
# 在 target_tracker.py 中已有速度信息
velocity = np.array([tracker.kalman.x[2], tracker.kalman.x[3]])
speed = np.linalg.norm(velocity)

# 判断是否为合理运动目标
is_moving = speed > 0.1  # 非完全静止
is_reasonable = speed < 5.0  # 速度合理（不是噪声跳变）
```

注意：运动特征更适合用于辅助跟踪中的目标验证，而非初始检测。

**反射率（Reflectivity/Intensity）**

Livox HAP 的 PointCloud2 消息中包含 `reflectivity` 字段（0~255）：
- 金属框架/螺旋桨保护罩：反射率通常 **20~80**
- 混凝土墙面：反射率通常 **80~200**
- 深色哑光材料：反射率通常 **5~30**

当前代码在 `point_cloud_to_o3d()` 中**没有解析 reflectivity 字段**。
若要使用，需修改点云解析代码，将 reflectivity 保留到 Open3D 点云的 colors 或自定义属性中。

建议优先级：
1. ⭐⭐⭐ 运动特征（已有Kalman速度，零成本提取）
2. ⭐⭐ PCA形态（需要约10行代码，计算量小）
3. ⭐ 反射率（需要修改点云解析管线，改动较大）

#### Q8：背景点云 PCD 文件是否可能"误吞"真实目标？如何避免？

**误吞机制分析**

背景减除（`background_subtractor.py`）的核心逻辑：
1. 离线阶段：加载 PCD 文件 → 每个点映射到体素 → 标记为背景体素
2. 在线阶段：新来的每个点 → 映射到体素 → 如果该体素是背景 → **丢弃**

"误吞"发生的条件：
```
无人机悬停位置的点 → 映射到的体素 → 恰好在 PCD 背景体素集合中
```

具体场景：
- PCD 文件包含了飞行区域某高度处的结构（如：顶部灯具、横梁、安全网吊点）
- 无人机恰好悬停在该结构的体素附近
- 背景减除会将无人机的点一起滤掉

**当前代码的风险点**

查看 `load_from_pcd()` 方法（已优化的向量化版本）：
```python
def load_from_pcd(self, pcd_file, use_all_points=True):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    # 所有点都映射为背景体素
    voxel_keys = (points / self.voxel_size).astype(np.int32)
    ...
    self.mask[keys] = True
```

`use_all_points=True` 表示 PCD 中**所有高度的点**都被标记为背景。
如果 PCD 采集时包含了高处的结构点（z 在飞行高度范围内），就会产生误吞。

**解决方案**

方案一：采集 PCD 时排除飞行高度区域
```yaml
# 建议修改 PCD 采集脚本，只保留安全高度以下的点
background_collection:
  z_max_for_pcd: -0.8  # 雷达坐标系中，只采集低于此高度的点作为背景
```

方案二：加载 PCD 后按高度过滤（推荐，改动小）
```python
def load_from_pcd(self, pcd_file, use_all_points=True, z_range=None):
    pcd = o3d.io.read_point_cloud(pcd_file)
    points = np.asarray(pcd.points)
    
    # 新增：可选高度过滤，排除飞行区域的背景点
    if z_range is not None:
        z_min, z_max = z_range
        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)
        points = points[mask]
    
    voxel_keys = (points / self.voxel_size).astype(np.int32)
    ...
```

方案三：定期检查 PCD 覆盖范围
```bash
# 用 Open3D 可视化 PCD 文件，检查是否有飞行高度的点
python3 -c "
import open3d as o3d
pcd = o3d.io.read_point_cloud('data/lab_pcds.pcd')
points = np.asarray(pcd.points)
print(f'Z范围: [{points[:,2].min():.2f}, {points[:,2].max():.2f}]')
print(f'飞行高度区域(-1.5~-0.5)的点数: {((points[:,2]>=-1.5)&(points[:,2]<=-0.5)).sum()}')
"
```

建议实施优先级：方案二（一行代码改动）> 方案三（定期检查）> 方案一（需重新采集）

---

#### Q9：远距离目标的点云密度衰减如何影响检测？如何应对？

**密度衰减原理**

Livox HAP 的点云密度随距离平方衰减：
```
密度 ∝ 1/r²

距离(m)    相对密度     典型无人机点数（估算）
──────────────────────────────────────────
5          1.00         80~120 点/帧
10         0.25         20~30 点/帧
15         0.11         9~13 点/帧
20         0.0625       5~7 点/帧
25         0.04         3~5 点/帧（接近检测极限）
```

Livox HAP 的扫描方式（非重复扫描花瓣形）使得多帧积累时密度有所改善，
但单帧密度衰减仍是主要问题。

**对当前检测管线的影响**

1. **DBSCAN 聚类失败**：远距离点稀疏，点间距 > eps(0.3m)，无法形成簇
2. **min_points 过滤**：远距离目标可能只有 3~5 个点，低于 min_points=5
3. **置信度过低**：point_count_score = (5-5)/(200-5) ≈ 0，总置信度极低
4. **尺寸失真**：稀疏点导致 bounding box 不准确

**应对策略**

策略一：距离自适应 DBSCAN 参数
```python
def _adaptive_dbscan(self, points, distance_from_lidar):
    """根据距离调整DBSCAN参数"""
    # 远距离时放大eps，降低min_samples
    scale = max(1.0, distance_from_lidar / 10.0)  # 10m以内不调整
    adaptive_eps = self.eps * scale
    adaptive_min = max(3, int(self.min_samples / scale))
    
    labels = np.array(
        self.pcd.cluster_dbscan(
            eps=adaptive_eps,
            min_points=adaptive_min,
            print_progress=False
        )
    )
    return labels
```

策略二：多帧点云积累（已有 frame_buffer 机制）
```yaml
# main_config.yaml 调大帧缓冲
air_target:
  frame_buffer_size: 5  # 积累5帧再聚类，远距离效果显著
```

策略三：距离自适应置信度阈值
```python
def _distance_adjusted_threshold(self, cluster_center, base_threshold=0.15):
    """远距离目标降低阈值要求"""
    dist = np.linalg.norm(cluster_center[:2])  # XY平面距离
    if dist > 15.0:
        return base_threshold * 0.6  # 远距离宽松40%
    elif dist > 10.0:
        return base_threshold * 0.8
    return base_threshold
```

策略四：分区域参数配置（进阶方案）
```yaml
# 将场地分为近、中、远三个区域，各自独立参数
detection_zones:
  near:   { range: [0, 10],  eps: 0.3, min_points: 5, threshold: 0.15 }
  mid:    { range: [10, 18], eps: 0.5, min_points: 4, threshold: 0.10 }
  far:    { range: [18, 28], eps: 0.8, min_points: 3, threshold: 0.08 }
```

推荐实施顺序：
1. 先用多帧积累（已有基础设施，改配置即可）
2. 再加距离自适应 eps（改动约20行）
3. 最后考虑分区域方案（架构变化较大）

#### Q10：多帧缓冲（frame_buffer）对检测有什么正面和负面影响？

**当前实现**

在 `air_target_node.py` 中，`frame_buffer_size` 控制积累多少帧点云后再做聚类：

```python
# 简化逻辑
self.frame_buffer = []

def _point_cloud_callback(self, msg):
    points = self._process_single_frame(msg)
    self.frame_buffer.append(points)
    
    if len(self.frame_buffer) >= self.config.frame_buffer_size:
        merged = np.vstack(self.frame_buffer)
        self.frame_buffer.clear()
        self._detect_and_track(merged)
```

**正面影响**

| 效果 | 说明 |
|:-----|:-----|
| 提高点密度 | N帧积累 → 点数约×N，远距离目标更容易形成聚类 |
| 改善聚类质量 | 更多点 → DBSCAN 簇更完整，bounding box 更准确 |
| 提高置信度 | point_count_score 显著提升 |
| 利用 Livox 非重复扫描 | HAP 每帧扫不同位置，多帧积累覆盖更完整 |

**负面影响**

| 问题 | 说明 |
|:-----|:-----|
| 延迟增加 | buffer_size=N → 延迟 ≈ (N-1) × 帧周期。若10Hz发布，buffer=5 → 额外400ms延迟 |
| 运动拖尾 | 快速移动目标在多帧中位置不同，聚类结果是"拖尾"形状，中心不准确 |
| 尺寸膨胀 | 移动目标的 bounding box 被拉长，尺寸过滤可能异常 |
| 形态失真 | PCA 形态分析结果不可靠（扁平物体变成长条状） |

**拖尾效应量化**

```
无人机速度: 2 m/s
帧率: 10 Hz
buffer_size: 5

拖尾长度 = 速度 × (buffer_size - 1) / 帧率
         = 2 × 4 / 10 = 0.8m

→ 一个直径 0.5m 的无人机，在多帧积累后"看起来"像 1.3m 的长条
→ 聚类中心偏移 ≈ 0.4m（在运动方向上落后于真实位置）
```

**推荐配置**

```yaml
# 实验室静态测试（无人机悬停/缓慢移动）
frame_buffer_size: 3~5  # 提高密度，延迟可接受

# 正式比赛（无人机快速移动）
frame_buffer_size: 1~2  # 减小延迟和拖尾
# 配合 Kalman 平滑来弥补单帧密度不足
```

进阶方案：运动补偿多帧积累
```python
def _motion_compensated_merge(self, frames, velocity_estimate):
    """根据速度估计补偿每帧的位移偏移"""
    merged = []
    dt = 1.0 / self.publish_rate
    for i, frame in enumerate(frames):
        age = (len(frames) - 1 - i) * dt  # 该帧距现在的时间
        offset = velocity_estimate * age   # 位移补偿
        compensated = frame - offset        # 将旧帧"推到"当前位置
        merged.append(compensated)
    return np.vstack(merged)
```
此方案需要 Kalman 已有速度估计，适合后期优化。

---

#### Q11：纯 LiDAR 检测 vs 相机辅助检测，当前阶段该如何选择？

**两种方案对比**

| 维度 | 纯 LiDAR | LiDAR + 相机 |
|:-----|:---------|:-------------|
| 数据来源 | 仅 Livox HAP 点云 | 点云 + RGB图像 |
| 距离信息 | 天然3D，精确距离 | 相机需要深度估计或融合 |
| 检测分辨率 | 远距离稀疏 | 相机远距离仍有较好分辨率 |
| 分类能力 | 仅几何特征（弱） | YOLO可识别类别（强） |
| 计算量 | 低（点云数千点） | 高（YOLO需要GPU推理） |
| 标定复杂度 | 无需标定 | 需要 LiDAR-Camera 外参标定 |
| 数据集需求 | 无需训练数据 | 需要无人机目标的YOLO训练数据 |
| 鲁棒性 | 不受光照影响 | 强光/逆光/暗光影响大 |
| 比赛场地照明 | N/A | 体育馆灯光可能造成炫光 |

**当前阶段建议：以纯 LiDAR 为主**

理由：
1. **没有训练数据**：目前没有RoboMaster无人机的YOLO训练数据集，从零标注耗时巨大
2. **标定工作量**：LiDAR-Camera外参标定在比赛现场难以精确完成
3. **算力限制**：雷达站NUC如果同时运行YOLO+点云处理，可能影响实时性
4. **LiDAR够用**：空中目标在背景减除后本身就很显著，纯几何方案有潜力做好

**未来相机介入的时机**

当纯 LiDAR 方案遇到以下瓶颈时，可考虑引入相机：
- 远距离（>15m）目标频繁漏检，且多帧积累仍不够
- 误检率始终无法降低（几何特征区分度不足）
- 需要识别无人机的**装甲板编号**以区分敌我

**如果未来需要相机融合，建议路径**

```
阶段1：采集比赛/训练赛视频，标注无人机目标
阶段2：训练 YOLO 小模型（YOLOv8n/YOLOv8s，速度优先）
阶段3：LiDAR 检测 ROI → 投影到图像 → 仅在 ROI 区域跑 YOLO（减少计算量）
阶段4：LiDAR 提供距离，YOLO 提供分类 → 融合决策
```

结论：**当前专注纯LiDAR方案，把几何特征用到极致**，相机方案作为预研储备。

#### Q12：HITS（哈工大深圳）的开源方案有哪些值得借鉴的设计？

**HITS-radar-2024 参考分析**

我们的 Kalman 滤波器 (`air_kalman_filter.py`) 已对齐 HITS 的 `KalmanFilter.cpp`。
以下是 HITS 方案中其他值得借鉴的设计：

**1. 目标跟踪器的生命周期管理**

HITS 方案使用了严格的 hit/miss 计数器：
```
新检出 → tentative（试探态，需连续N帧命中才升级）
tentative + 连续命中 → confirmed（确认态，才开始对外发布）
confirmed + 连续丢失 → deleted（删除态，清理资源）
```

我们当前的 `target_tracker.py` 使用了 `max_age` 丢失计数 + `min_hits` 确认计数，
基本对齐此设计。但可以进一步细化：

建议优化点：
- 增加 `tentative` 态的独立处理（目前只有 age 和 hits 计数，没有显式状态机）
- tentative 目标不对外发布，减少短暂误检的暴露

**2. Mahalanobis 距离匹配**

HITS 使用马氏距离（Mahalanobis distance）进行检测-跟踪匹配：
```python
# 马氏距离考虑了 Kalman 协方差，比欧氏距离更合理
# 如果 Kalman 对某方向不确定性大，该方向的偏差权重小
d_maha = sqrt((z - Hx)^T S^(-1) (z - Hx))  # S = H P H^T + R
```

我们的 `target_tracker.py` 中已实现了马氏距离匹配（`_mahalanobis_distance`），
这是正确的设计。

**3. 力组合/分裂机制**

HITS 方案有 `force_combine` 和 `separate` 逻辑：
- force_combine：当两个跟踪器过近时合并为一个（避免同一目标被重复跟踪）
- separate：当一个跟踪器内的检测突然分成两团时分裂

我们的 `target_tracker.py` 已实现了这两个机制，对齐 HITS 设计。

**4. 其他可借鉴点**

| 特性 | HITS 实现 | 我们的状态 | 建议 |
|:-----|:---------|:----------|:-----|
| EKF vs KF | 线性 KF（2D 匀速） | 已对齐 | 暂不需要 EKF |
| 门控（Gating） | 马氏距离阈值门控 | 已实现 | 可微调阈值 |
| loose_query | 宽松匹配查询 | 已实现 | 用于检测缺失时的容错 |
| 坐标变换 | TF2 | 已实现 | 注意比赛现场TF校准 |
| 多目标数量限制 | 最大跟踪数 | 未限制 | 建议加上 max_targets=3 |

建议新增限制：
```python
# target_tracker.py 中增加
MAX_ACTIVE_TARGETS = 3  # 场上最多3台无人机（RMUC规则：双方各1台，但实际场上最多2台同时飞行，留余量）
```

---

#### Q13：在没有实际无人机的情况下，如何进行有效的检测功能静态测试？

**静态测试方案总览**

```
方法1：手持物体模拟                    [难度：★☆☆ 推荐度：★★★]
方法2：RViz 手动添加虚拟点云           [难度：★★☆ 推荐度：★★☆]
方法3：rosbag 录制回放                 [难度：★★☆ 推荐度：★★★]
方法4：编写点云发布模拟节点            [难度：★★★ 推荐度：★★☆]
```

**方法1：手持物体模拟（立即可用）**

用一个体积接近无人机的物体（纸箱、泡沫板等，约0.5m×0.5m）在空中缓慢移动：
1. 正常启动 LiDAR 和检测节点
2. 确保背景 PCD 已录制（无模拟物体时）
3. 将物体举到飞行高度（距地面 1.5~2.4m）缓慢移动
4. 观察 RViz 中 `/air_target/markers` 是否出现跟踪标记

优点：最接近真实场景，可直接验证整个管线
缺点：需要一个人配合，高度不好精确控制

**方法2：rosbag 录制回放（强烈推荐）**

步骤一：录制有目标的数据
```bash
# 方法1做的时候顺便录一段bag
ros2 bag record /livox/lidar -o test_resources/air_target_test_bag
```

步骤二：回放测试
```bash
# 终端1：回放bag（循环播放）
ros2 bag play test_resources/air_target_test_bag --loop

# 终端2：启动检测节点
ros2 launch hnurm_bringup air_target.launch.py
```

优点：可重复、可对比参数调优效果
缺点：首次需要录制

**方法3：点云模拟发布节点**

编写一个 Python 节点，发布包含模拟无人机点云的 PointCloud2 消息：
```python
# 示例：发布一个在空中缓慢移动的点团
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class FakeUAVPublisher(Node):
    def __init__(self):
        super().__init__('fake_uav_publisher')
        self.pub = self.create_publisher(PointCloud2, '/livox/lidar', 10)
        self.timer = self.create_timer(0.1, self.publish)  # 10Hz
        self.t = 0.0
    
    def publish(self):
        self.t += 0.1
        # 模拟无人机轨迹：从(5,0)飞到(5,10)
        cx, cy, cz = 5.0, self.t * 0.5, -1.0  # 雷达坐标系z轴向上为负
        
        # 生成无人机形状的点云（扁平盘状，约50个点）
        N = 50
        pts = np.random.randn(N, 3) * [0.3, 0.3, 0.05]  # XY方向0.3m，Z方向0.05m
        pts += [cx, cy, cz]
        
        # 加上背景噪声点（模拟未被完全滤除的静态点）
        bg = np.random.uniform([-5, -5, -2], [5, 5, 0], (200, 3))
        all_pts = np.vstack([pts, bg])
        
        # 发布
        header = ... # 设置时间戳和frame_id
        msg = pc2.create_cloud_xyz32(header, all_pts.tolist())
        self.pub.publish(msg)
```

此方案将在 Part13（Rosbag离线测试方案）中给出完整设计。

#### Q14：如何利用"帧间存在率"来验证目标持续性？

**概念定义**

帧间存在率（Frame Presence Rate）= 最近 N 帧中目标被检出的帧数 / N

```
例：最近 10 帧，目标被检出 8 帧 → 存在率 = 80%
```

**为什么有效**

- 真实无人机：持续存在于场景中，除非被遮挡，存在率通常 **>70%**
- 噪声/飞虫/灰尘：偶发出现，存在率通常 **<30%**
- 能量机关旋转臂：周期性出现，存在率 **40~60%**（可通过周期性分析排除）

**实现方案**

在 `target_tracker.py` 的每个 TrackedTarget 中维护一个滑动窗口：

```python
from collections import deque

class TrackedTarget:
    def __init__(self, ...):
        ...
        self.presence_window = deque(maxlen=20)  # 最近20帧
    
    def update_presence(self, detected: bool):
        self.presence_window.append(1 if detected else 0)
    
    @property
    def presence_rate(self) -> float:
        if len(self.presence_window) == 0:
            return 0.0
        return sum(self.presence_window) / len(self.presence_window)
    
    @property
    def is_reliable(self) -> bool:
        """只有存在率足够高才认为是可靠目标"""
        return (len(self.presence_window) >= 5 and 
                self.presence_rate > 0.5)
```

集成到发布逻辑：
```python
# air_target_node.py 中的发布判断
for target in self.tracker.get_active_targets():
    if target.is_reliable:  # 新增条件
        self._publish_target(target)
```

**与 min_hits 的区别**

| 特性 | min_hits | 帧间存在率 |
|:-----|:---------|:----------|
| 判断方式 | 累计命中次数 | 滑动窗口比例 |
| 容错性 | 不容忍中间丢帧 | 允许偶尔丢帧 |
| 适用场景 | 目标首次确认 | 持续跟踪可靠性评估 |
| 互补关系 | 初始确认 | 持续验证 |

建议：min_hits 用于目标从 tentative→confirmed，帧间存在率用于 confirmed 目标的持续可靠性评估。

---

#### Q15：PCA 形态分析的完整实现方案是什么？

**完整实现**

在 Q7 中给出了 PCA 的基本原理和评分函数。这里给出完整的集成方案。

**步骤一：在 `cluster_detector.py` 中添加 PCA 分析**

```python
def _analyze_cluster_shape(self, points: np.ndarray) -> dict:
    """
    对聚类点云做PCA形态分析
    
    返回:
        shape_info: {
            'eigenvalues': [λ1, λ2, λ3],  # 降序
            'linearity': float,    # 线性度
            'planarity': float,    # 平面度
            'sphericity': float,   # 球度
            'shape_score': float,  # 综合评分（扁平状高分）
            'shape_label': str     # 'planar'/'linear'/'spherical'/'unknown'
        }
    """
    result = {
        'eigenvalues': [0, 0, 0],
        'linearity': 0, 'planarity': 0, 'sphericity': 0,
        'shape_score': 0.5, 'shape_label': 'unknown'
    }
    
    if len(points) < 4:
        return result
    
    # PCA
    centered = points - points.mean(axis=0)
    cov = np.cov(centered.T)
    eigenvalues = np.sort(np.linalg.eigvalsh(cov))[::-1]  # 降序
    
    l1, l2, l3 = eigenvalues
    result['eigenvalues'] = eigenvalues.tolist()
    
    if l1 < 1e-10:
        return result
    
    # 计算形态指标
    result['linearity'] = (l1 - l2) / l1
    result['planarity'] = (l2 - l3) / l1
    result['sphericity'] = l3 / l1
    
    # 形态分类
    if result['planarity'] > 0.3 and result['linearity'] < 0.4:
        result['shape_label'] = 'planar'    # 扁平状（期望的无人机形态）
        result['shape_score'] = 0.7 + result['planarity'] * 0.3
    elif result['linearity'] > 0.5:
        result['shape_label'] = 'linear'    # 线状（可能是边缘/噪声）
        result['shape_score'] = 0.2
    elif result['sphericity'] > 0.4:
        result['shape_label'] = 'spherical' # 球状（点团噪声）
        result['shape_score'] = 0.3
    else:
        result['shape_label'] = 'unknown'
        result['shape_score'] = 0.5
    
    return result
```

**步骤二：集成到置信度计算**

```python
def _calculate_confidence(self, cluster_points, cluster_size):
    # 现有评分
    point_score = ...   # 60%权重
    size_score = ...    # 现有40%权重 → 改为25%
    
    # 新增PCA评分（15%权重）
    shape_info = self._analyze_cluster_shape(cluster_points)
    shape_score = shape_info['shape_score']
    
    confidence = (point_score * 0.50 + 
                  size_score * 0.30 + 
                  shape_score * 0.20)  # 权重重新分配
    return confidence, shape_info
```

**调试建议**

在 RViz MarkerArray 中显示 PCA 信息，方便调参：
```python
# 在 debug marker 中添加形态信息
text = f"shape={shape_info['shape_label']}\n"
text += f"P={shape_info['planarity']:.2f} L={shape_info['linearity']:.2f}"
```

---

#### Q16：目标的帧间尺寸一致性如何用于验证？

**原理**

真实的飞行中的无人机：
- 物理尺寸固定（≤1.7m×1.7m×0.8m）
- 在雷达视角下，尺寸变化主要来自距离变化和扫描角度
- 帧间尺寸变化应该**平滑且缓慢**

虚假目标（噪声聚类）：
- 尺寸随机跳变
- 帧间尺寸相关性低

**实现方案**

在 TrackedTarget 中维护尺寸历史：

```python
class TrackedTarget:
    def __init__(self, ...):
        ...
        self.size_history = deque(maxlen=10)  # 最近10帧的尺寸
    
    def update_size(self, bbox_size: np.ndarray):
        """bbox_size: [width, height, depth] 三个维度"""
        self.size_history.append(bbox_size)
    
    @property
    def size_consistency(self) -> float:
        """尺寸一致性评分：0~1，越高越一致"""
        if len(self.size_history) < 3:
            return 0.5  # 数据不足，中性评分
        
        sizes = np.array(self.size_history)
        # 计算每个维度的变异系数 (CV = std/mean)
        means = sizes.mean(axis=0)
        stds = sizes.std(axis=0)
        
        # 避免除零
        cv = np.where(means > 0.01, stds / means, 1.0)
        avg_cv = cv.mean()
        
        # CV < 0.2 → 很一致（评分高）
        # CV > 0.5 → 变化大（评分低）
        score = np.clip(1.0 - avg_cv * 2.0, 0.0, 1.0)
        return float(score)
    
    @property
    def max_dimension_in_range(self) -> bool:
        """最大维度是否在合理范围内"""
        if len(self.size_history) == 0:
            return True
        recent = self.size_history[-1]
        max_dim = max(recent)
        return 0.1 <= max_dim <= 2.0  # 留余量的无人机尺寸范围
```

**帧间尺寸跳变检测**

```python
def _check_size_jump(self) -> bool:
    """检测尺寸是否突然跳变（可能是跟踪器关联错误）"""
    if len(self.size_history) < 2:
        return False
    
    prev = np.array(self.size_history[-2])
    curr = np.array(self.size_history[-1])
    
    # 最大维度变化超过50%认为跳变
    max_prev = max(prev)
    max_curr = max(curr)
    if max_prev > 0.01:
        change_ratio = abs(max_curr - max_prev) / max_prev
        return change_ratio > 0.5
    return False
```

**集成到跟踪决策**

```python
# target_tracker.py 中的目标验证
def _validate_target(self, target: TrackedTarget) -> bool:
    """综合验证目标是否可信"""
    checks = [
        target.is_reliable,              # Q14: 帧间存在率
        target.size_consistency > 0.3,   # Q16: 尺寸一致性
        target.max_dimension_in_range,   # Q16: 尺寸合理性
        not target._check_size_jump(),   # Q16: 无突变
    ]
    return all(checks)
```

以上 Q14~Q16 三个特征可以组成**目标可靠性评估模块**，对 confirmed 目标进行持续验证，
在发现不可靠时降低其优先级或标记为待观察。

---

### 15. Rosbag 离线测试框架设计

#### 15.1 目标

建立一套可重复、可量化的离线测试框架，用于：
- 在没有实际无人机的情况下验证检测管线
- 参数调优时快速对比不同配置的效果
- 回归测试，确保代码修改不引入新问题

#### 15.2 文件结构

```
test_resources/
├── bags/
│   ├── lab_static_bg/           # 纯背景（无目标），用于测试误检率
│   │   └── metadata.yaml
│   ├── lab_handheld_slow/       # 手持物体缓慢移动，模拟悬停无人机
│   │   └── metadata.yaml
│   ├── lab_handheld_fast/       # 手持物体快速移动，模拟飞行无人机
│   │   └── metadata.yaml
│   └── competition_replay/      # 比赛/训练赛实录（如有）
│       └── metadata.yaml
├── configs/
│   ├── test_conservative.yaml   # 保守参数（低误检）
│   ├── test_balanced.yaml       # 平衡参数
│   └── test_aggressive.yaml     # 激进参数（低漏检）
├── ground_truth/
│   ├── lab_handheld_slow_gt.csv # 人工标注的真实目标位置
│   └── lab_handheld_fast_gt.csv
└── scripts/
    ├── run_offline_test.sh      # 一键测试脚本
    ├── evaluate_results.py      # 评估脚本（计算precision/recall）
    └── record_bag.sh            # 录制辅助脚本
```

#### 15.3 录制脚本

```bash
#!/bin/bash
# scripts/record_bag.sh
# 用法: ./record_bag.sh <场景名称>

SCENE_NAME=${1:-"unnamed"}
OUTPUT_DIR="test_resources/bags/${SCENE_NAME}"
TOPICS="/livox/lidar /tf /tf_static"

echo "开始录制场景: ${SCENE_NAME}"
echo "话题: ${TOPICS}"
echo "按 Ctrl+C 停止录制"

mkdir -p "${OUTPUT_DIR}"
ros2 bag record ${TOPICS} -o "${OUTPUT_DIR}" --max-cache-size 1000000000

echo "录制完成: ${OUTPUT_DIR}"
echo "文件大小: $(du -sh ${OUTPUT_DIR})"
```

#### 15.4 离线测试脚本

```bash
#!/bin/bash
# scripts/run_offline_test.sh
# 用法: ./run_offline_test.sh <bag路径> [配置文件]

BAG_PATH=${1:?"请指定bag路径"}
CONFIG=${2:-"configs/main_config.yaml"}
OUTPUT_DIR="test_output/$(date +%Y%m%d_%H%M%S)"

mkdir -p "${OUTPUT_DIR}"

echo "=== 离线测试 ==="
echo "Bag: ${BAG_PATH}"
echo "Config: ${CONFIG}"
echo "Output: ${OUTPUT_DIR}"

# 启动检测节点（后台）
ros2 launch hnurm_bringup air_target.launch.py \
    config_file:=${CONFIG} \
    &
DETECT_PID=$!

# 启动结果记录节点（记录检测输出到CSV）
ros2 run hnurm_radar record_detections \
    --ros-args -p output_file:="${OUTPUT_DIR}/detections.csv" \
    &
RECORD_PID=$!

sleep 2  # 等待节点启动

# 回放bag
ros2 bag play "${BAG_PATH}" --rate 1.0

sleep 2  # 等待最后的数据处理完

# 停止节点
kill ${DETECT_PID} ${RECORD_PID} 2>/dev/null
wait

echo "检测结果已保存到: ${OUTPUT_DIR}/detections.csv"

# 如果有ground truth，自动评估
GT_FILE="test_resources/ground_truth/$(basename ${BAG_PATH})_gt.csv"
if [ -f "${GT_FILE}" ]; then
    echo "发现Ground Truth，开始评估..."
    python3 scripts/evaluate_results.py \
        --detections "${OUTPUT_DIR}/detections.csv" \
        --ground_truth "${GT_FILE}" \
        --output "${OUTPUT_DIR}/evaluation.json"
fi
```

#### 15.5 评估指标

```python
# scripts/evaluate_results.py 核心逻辑
def evaluate(detections, ground_truth, distance_threshold=1.0):
    """
    评估检测结果
    
    指标:
    - Precision: TP / (TP + FP)  — 检出中有多少是对的
    - Recall:    TP / (TP + FN)  — 真实目标有多少被检出
    - F1 Score:  2 * P * R / (P + R)
    - MOTA:      Multi-Object Tracking Accuracy（多目标跟踪精度）
    - 平均位置误差: 检出目标与真实位置的平均距离
    - 延迟: 目标出现到首次检出的时间差
    
    匹配规则:
    - 检测结果与GT的欧氏距离 < distance_threshold 视为匹配
    - 使用匈牙利算法做最优匹配
    """
    results = {
        'total_gt': len(ground_truth),
        'total_det': len(detections),
        'true_positives': 0,
        'false_positives': 0,
        'false_negatives': 0,
        'avg_position_error': 0.0,
        'avg_detection_delay': 0.0,
    }
    # ... 实现匹配和计算
    return results
```

#### 15.6 Ground Truth 标注格式

```csv
# test_resources/ground_truth/lab_handheld_slow_gt.csv
# timestamp_sec, x_world, y_world, z_world, target_id, label
0.0, 3.5, 2.0, 1.8, 1, uav
0.1, 3.5, 2.1, 1.8, 1, uav
0.2, 3.5, 2.2, 1.8, 1, uav
...
```

简易标注方法：在 RViz 回放 bag 时用 Publish Point 工具点击目标位置，记录到CSV。

---

### 16. 基于规则手册的完整空中目标检测逻辑设计

#### 16.1 设计原则

根据 RMUC 2026 规则手册 V1.3.0 的空中机器人规格约束，设计一套**多层级过滤+多维度评分**的检测逻辑。

核心思路：**宁可多检一些候选目标交给跟踪器验证，也不要在检测阶段就丢掉真实目标。**

#### 16.2 完整检测管线

```
                         ┌──────────────────┐
  /livox/lidar ─────────▶│  PointCloud2解析  │
  (PointCloud2)          │  point_cloud_to_o3d()
                         └────────┬─────────┘
                                  │ Open3D点云
                                  ▼
                         ┌──────────────────┐
                    L1   │  ROI空间裁剪      │  根据比赛场地尺寸
                         │  crop_roi()       │  只保留场地范围内的点
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                    L2   │  体素降采样       │  voxel_size=0.05m
                         │  voxel_downsample()│ 减少点数，均匀化密度
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                    L3   │  背景减除         │  基于预录PCD的体素背景模型
                         │  bg_subtractor    │  过滤静态场景点
                         │  .filter()        │  ⚠ 注意避免误吞飞行区域
                         └────────┬─────────┘
                                  │ 前景点云（动态/未知点）
                                  ▼
                         ┌──────────────────┐
                    L4   │  高度过滤         │  z_min=-1.5, z_max=-0.5
                         │  height_filter()  │  只保留飞行高度范围的点
                         └────────┬─────────┘  （雷达坐标系，向上为负）
                                  │
                                  ▼
                         ┌──────────────────┐
                    L5   │  [多帧缓冲]       │  可选：积累N帧提高密度
                         │  frame_buffer     │  ⚠ 注意延迟和拖尾
                         └────────┬─────────┘
                                  │ 合并后的前景点云
                                  ▼
                         ┌──────────────────┐
                    L6   │  DBSCAN聚类       │  eps自适应距离
                         │  cluster_dbscan() │  min_samples自适应
                         └────────┬─────────┘
                                  │ 聚类列表
                                  ▼
                         ┌──────────────────┐
                    L7   │  聚类过滤与评分    │  多维度：
                         │  cluster_filter   │  ├ 点数评分 (50%)
                         │                   │  ├ 尺寸评分 (30%)
                         │                   │  └ PCA形态评分 (20%)
                         └────────┬─────────┘
                                  │ 候选检测列表
                                  ▼
                         ┌──────────────────┐
                    L8   │  多目标跟踪       │  Kalman预测+马氏距离匹配
                         │  target_tracker   │  ├ 新目标→tentative
                         │                   │  ├ 连续命中→confirmed
                         │                   │  └ 连续丢失→deleted
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                    L9   │  目标可靠性验证    │  ├ 帧间存在率 >50%
                         │  reliability      │  ├ 尺寸一致性 >0.3
                         │  validation       │  ├ 速度合理性 <5m/s
                         │                   │  └ 尺寸范围检查
                         └────────┬─────────┘
                                  │ 验证通过的目标
                                  ▼
                         ┌──────────────────┐
                    L10  │  坐标变换         │  LiDAR坐标 → 世界坐标
                         │  TF2 transform   │  通过 /tf 话题
                         └────────┬─────────┘
                                  │
                                  ▼
                         ┌──────────────────┐
                    L11  │  比赛区域约束      │  排除已知的非飞行区域：
                         │  competition_zone │  ├ 能量机关区域（场地中央）
                         │  filter           │  ├ 己方基地上方
                         │                   │  └ 安全绳固定点附近
                         └────────┬─────────┘
                                  │
                                  ▼
                    ┌─────────────┴─────────────┐
                    │                           │
                    ▼                           ▼
           ┌──────────────┐            ┌──────────────┐
           │  发布 /location│           │  发布 debug   │
           │  (裁判系统)    │           │  markers      │
           │  Float64MultiArray│       │  (RViz可视化)  │
           └──────────────┘            └──────────────┘
```

#### 16.3 各层级的规则手册依据

| 层级 | 功能 | 规则依据 |
|:-----|:-----|:---------|
| L1 ROI | 场地28m×15m | 比赛场地规格，含2.4m高挡板 |
| L4 高度 | 1.5m~2.4m | 无人机飞行高度限制 + 安全绳长度2.4m |
| L6 DBSCAN | 聚类尺寸约束 | 无人机尺寸 ≤1.7m×1.7m×0.8m |
| L7 尺寸 | min_size~max_size | 考虑保护罩后的典型尺寸0.3~1.7m |
| L8 跟踪 | 最大目标数 | RMUC规则：每方1台UAV |
| L9 速度 | <5m/s合理 | 室内飞行速度约束 |
| L11 区域 | 能量机关排除 | 场地中央能量机关可能被误检 |

#### 16.4 比赛特定区域排除

```python
# 比赛场地坐标系（世界坐标，单位：m）
EXCLUSION_ZONES = [
    {
        'name': 'energy_mechanism_red',
        'center': [14.0, 3.75],   # 红方能量机关位置（估算）
        'radius': 2.0,
        'z_range': [0.5, 3.0],
    },
    {
        'name': 'energy_mechanism_blue', 
        'center': [14.0, 11.25],  # 蓝方能量机关位置（估算）
        'radius': 2.0,
        'z_range': [0.5, 3.0],
    },
    {
        'name': 'tether_clamp',
        'center': [14.0, 7.5],    # 安全绳夹具位置（场地中线）
        'radius': 1.0,
        'z_range': [2.0, 3.5],
    },
]

def is_in_exclusion_zone(x, y, z, zones=EXCLUSION_ZONES):
    for zone in zones:
        cx, cy = zone['center']
        dist_xy = np.sqrt((x-cx)**2 + (y-cy)**2)
        if (dist_xy < zone['radius'] and 
            zone['z_range'][0] <= z <= zone['z_range'][1]):
            return True, zone['name']
    return False, None
```

注意：以上坐标为估算值，需要在比赛现场根据实际场地测量校准。

---

### 17. 行动计划清单

#### 17.1 近期行动（1~2周内，实验室可完成）

**优先级 P0：立即可做，零风险**

| # | 任务 | 预计耗时 | 改动文件 | 说明 |
|:--|:-----|:---------|:---------|:-----|
| 1 | 用 RViz Publish Point 确认坐标系 | 30min | 无 | 点击场景中已知位置，确认 z_min/z_max 的正确方向 |
| 2 | 检查背景 PCD 的 Z 范围 | 15min | 无 | 运行 Q8 中的检查脚本，确认 PCD 是否覆盖飞行区域 |
| 3 | 在 main_config.yaml 中添加注释 | 已完成 | main_config.yaml | ✅ 已在本次对话中完成 |
| 4 | background_subtractor.py 向量化优化 | 已完成 | background_subtractor.py | ✅ 已在本次对话中完成 |

**优先级 P1：小改动，显著收益（1~3天）**

| # | 任务 | 预计耗时 | 改动文件 | 说明 |
|:--|:-----|:---------|:---------|:-----|
| 5 | 根据 RViz 确认结果修正 z_min/z_max | 1h | main_config.yaml | 参考 Q3 分析 |
| 6 | 根据实验室尺寸修正 ROI 范围 | 1h | main_config.yaml | 参考 Q4 分析 |
| 7 | 添加 PCA 形态评分 | 2h | cluster_detector.py | 参考 Q15 完整代码 |
| 8 | 提取 Kalman 速度做运动特征判断 | 1h | target_tracker.py | 参考 Q7 运动特征部分 |
| 9 | 背景 PCD 加载时增加高度过滤选项 | 30min | background_subtractor.py | 参考 Q8 方案二 |
| 10 | 添加 max_active_targets 限制 | 30min | target_tracker.py | 参考 Q12，限制为3 |

**优先级 P2：中等改动，完善体系（3~7天）**

| # | 任务 | 预计耗时 | 改动文件 | 说明 |
|:--|:-----|:---------|:---------|:-----|
| 11 | 实现帧间存在率验证 | 2h | target_tracker.py | 参考 Q14 |
| 12 | 实现尺寸一致性验证 | 2h | target_tracker.py | 参考 Q16 |
| 13 | 实现综合目标验证函数 _validate_target | 1h | target_tracker.py | 整合 Q14+Q16 |
| 14 | 录制实验室 rosbag（手持物体） | 1h | 无（录到test_resources/） | 参考 Q13 方法1+2 |
| 15 | 搭建离线测试脚本框架 | 3h | scripts/ | 参考第15章 |
| 16 | 添加 tentative 状态机 | 3h | target_tracker.py | 参考 Q12 HITS生命周期 |

#### 17.2 中期行动（2~4周，赛前准备）

| # | 任务 | 预计耗时 | 说明 |
|:--|:-----|:---------|:-----|
| 17 | 距离自适应 DBSCAN 参数 | 4h | 参考 Q9 策略一，远距离放大 eps |
| 18 | 距离自适应置信度阈值 | 2h | 参考 Q9 策略三 |
| 19 | 运动补偿多帧积累（进阶） | 6h | 参考 Q10 进阶方案 |
| 20 | 编写点云模拟发布节点 | 4h | 参考 Q13 方法3，用于无硬件测试 |
| 21 | 建立评估指标脚本 | 4h | 参考 15.5 评估指标 |
| 22 | 比赛场地区域排除配置 | 2h | 参考 16.4，需现场测量坐标 |
| 23 | 参数配置文件分场景版本 | 2h | lab / competition_conservative / competition_aggressive |

#### 17.3 长期行动（比赛后 / 下赛季储备）

| # | 任务 | 预计耗时 | 说明 |
|:--|:-----|:---------|:-----|
| 24 | 反射率(reflectivity)特征集成 | 8h | 修改点云解析管线，Q7 反射率部分 |
| 25 | 相机辅助检测预研 | 2~4周 | Q11 未来路径，需要先采集训练数据 |
| 26 | 分区域独立参数配置 | 8h | Q9 策略四，架构调整较大 |
| 27 | 在线背景自适应学习 | 1周 | background_subtractor 增加在线更新能力 |
| 28 | 多雷达协同检测 | 2~4周 | 多台 HAP 数据融合，需要额外标定 |

#### 17.4 比赛现场快速调试清单

赛前 30 分钟检查流程：

```
□ 1. 确认 LiDAR 数据正常
      ros2 topic hz /livox/lidar    → 期望 10Hz
      ros2 topic echo /livox/lidar --once | head -5

□ 2. 确认 TF 变换正常
      ros2 run tf2_ros tf2_echo world livox_frame
      → 应该有稳定输出，无报错

□ 3. 录制新的背景 PCD（比赛场地！）
      → 确保场地无人员走动
      → 录制 30~60秒
      → 检查 PCD Z范围不覆盖飞行高度

□ 4. RViz 快速验证
      → 打开 Publish Point 工具
      → 点击场地地面 → 确认 Z 值方向
      → 点击挡板顶部 → 确认高度约 2.4m
      → 确认坐标系与配置一致

□ 5. 参数微调
      → z_min/z_max 根据现场确认结果调整
      → ROI 根据雷达安装位置调整
      → confidence_threshold: 先用 0.15 观察，再逐步提高

□ 6. 手持物体测试
      → 在飞行高度区域移动一个 0.5m 物体
      → 确认 /air_target/markers 有响应
      → 确认 /location 话题有输出

□ 7. 记录测试 bag
      → 录一段有手持目标的 bag 备用
      → 比赛间隙可用于离线分析
```

---

### 18. 参数速查表

```yaml
# ============================================================
#  空中目标检测 - 参数速查表
#  配置文件: configs/main_config.yaml → air_target 节
# ============================================================

# --- 点云预处理 ---
voxel_size: 0.05          # 降采样体素大小(m)，越小精度越高但点越多
roi:
  x: [-5.0, 20.0]        # ROI X范围(m)，需根据场地调整
  y: [-8.0, 8.0]         # ROI Y范围(m)，需根据场地调整
  z: [-3.0, 0.5]         # ROI Z范围(m)，包含飞行高度+余量

# --- 背景减除 ---
background:
  voxel_size: 0.1         # 背景体素大小(m)，越小越精细但内存越大
  bg_pcd_file: "data/lab_pcds.pcd"  # 背景PCD文件路径
  # ⚠ 比赛时需更换为比赛场地PCD

# --- 高度过滤 ---
height_filter:
  z_min: -1.5             # 飞行高度下界(雷达坐标系)
  z_max: -0.5             # 飞行高度上界(雷达坐标系)
  # ⚠ 取决于雷达安装高度和坐标系方向
  # 用 RViz Publish Point 点击实际飞行高度位置确认

# --- 多帧缓冲 ---
frame_buffer_size: 3      # 积累帧数
  # 实验室: 3~5（提高密度）
  # 比赛:   1~2（减少延迟）

# --- DBSCAN 聚类 ---
clustering:
  eps: 0.3                # 邻域半径(m)
  min_samples: 5          # 最小点数
  # 调参口诀: 先调eps看簇形，再调min_samples控大小

# --- 目标过滤 ---
target_filter:
  min_points: 5           # 最少点数
  max_points: 200         # 最多点数（超出可能是墙面残留）
  min_size: 0.1           # 最小尺寸(m)
  max_size: 1.5           # 最大尺寸(m)
  confidence_threshold: 0.15  # 置信度阈值
  # 实验室: 0.15（宽松）  比赛: 0.25~0.35（严格）

# --- Kalman 跟踪 ---
tracking:
  max_age: 5              # 连续丢失N帧后删除跟踪器
  min_hits: 3             # 连续命中N帧后确认目标
  mahalanobis_threshold: 9.21  # 马氏距离门控（卡方分布95%分位）

# --- 发布 ---
publish_rate: 10.0        # 发布频率(Hz)
```

---

### 19. 本章更新记录

| 日期 | 更新内容 |
|:-----|:---------|
| 2026-03 | 新增第13~18章：16个问题深度分析、离线测试框架、完整检测逻辑、行动计划、参数速查 |
| 2026-03 | 代码优化：background_subtractor.py 向量化（learn/filter/load_from_pcd） |
| 2026-03 | 配置注释：main_config.yaml 添加 clustering 和 target_filter 详细注释 |

---

*文档结束。如有问题请联系雷达站组。*

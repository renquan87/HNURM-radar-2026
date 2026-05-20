# 空中机器人检测集成方案

> 基于 `air_target_radar` 仿真项目集成到 `hnurm_radar` 主项目  
> 日期：2026-03-02

---

## 1. 概述

新增**方案三：纯激光雷达空中机器人定位**，与现有方案一（纯相机）和方案二（激光雷达+相机）并行运行。

- **输入**：`/lidar_pcds`（来自 `lidar_node` 的累积点云）  
- **输出**：`/location`（`Locations` 消息，空中机器人赛场坐标）  
- **不依赖相机**，不新增坐标变换代码，复用现有 `registration` 节点的 ICP TF 变换

### 数据流

```
/livox/lidar → lidar_node → /lidar_pcds → air_target_node → /location → ekf_node → /ekf_location_filtered
                                                                                       ↓
                                                                              display_panel (可视化)
```

空中与地面机器人共用同一个点云、同一套 TF 变换（`livox → map`），无需额外标定。

---

## 2. 新建文件

所有文件位于 `src/hnurm_radar/hnurm_radar/air_scheme/`：

| 文件 | 功能 |
|---|---|
| `__init__.py` | 包初始化 |
| `air_config.py` | 从 `main_config.yaml` 读取配置（dataclass） |
| `point_cloud_processor.py` | ROI 裁剪、体素降采样、高度过滤（提取空中区域） |
| `cluster_detector.py` | DBSCAN 聚类 + Z 轴压缩（HITS 特性）+ 目标尺寸过滤 |
| `background_subtractor.py` | map 坐标系 KDTree 背景减除；兼容旧的 PCD/online 体素模式 |
| `air_kalman_filter.py` | 4 状态 [x, y, vx, vy] 卡尔曼滤波（含 cov_factor、stop_p_time） |
| `target_tracker.py` | 多目标跟踪（Mahalanobis 匹配 + 强制合并 + 丢失超时） |
| `air_target_node.py` | 主 ROS2 节点，串联以上模块，订阅点云，发布坐标 |

新增：`air_target_node.process_frame_pcd(pcd)` 接口，支持直接输入 Open3D 点云对象（不依赖文件路径）。

---

## 3. 修改文件

| 文件 | 修改内容 |
|---|---|
| `configs/main_config.yaml` | 新增 `air_target:` 配置段（预处理/聚类/跟踪/背景减除参数） |
| `src/hnurm_radar/setup.py` | 添加 `air_target_node` 入口点 |
| `src/hnurm_bringup/launch/hnurm_air_launch.py` | 编排 `lidar_node`、`air_target_node`、`display_panel`、`ekf_node`、`judge_messager` |
| `src/hnurm_radar/hnurm_radar/Car/Car.py` | 支持 ID 6/106（红蓝空中机器人） |
| `src/ekf/ekf/ekf_node.py` | 扩展至 14 个红蓝独立 EKF slot，含空中 ID 映射 |
| `src/hnurm_radar/hnurm_radar/shared/display_panel.py` | 空中机器人显示标注 " UAV" 后缀 |

### 当前限制

| 文件 | 说明 |
|---|---|
| `judge_messager.py` | `hnurm_air_launch.py` 已启动该节点，但发送映射仍只包含地面 1-5 与哨兵 7/107；空中 ID 6/106 暂不经 0x0305 发送 |

---

## 4. 坐标变换方案

**不新写坐标变换代码**，复用现有 TF 方案：

```
                    registration 节点（ICP 配准）
                           ↓
              发布 TF: livox → map
                           ↓
air_target_node 通过 tf2_ros 查询 TF
                           ↓
       聚类中心 [x,y,z]lidar → [x,y,z]field
```

- **赛场模式**：`registration` 节点用赛场 PCD 做 ICP，TF 精度高  
- **实验室模式**：`registration` 节点用实验室 PCD 做 ICP，同一套代码  
- TF 变换后，赛场坐标 Z 分量即为目标离地高度

原仿真项目的线性变换（`x_scale/y_scale/x_offset/y_offset`）和手动雷达安装参数（`radar.height/position_x/position_y`）均已移除，因为在真实比赛中这些参数不可知，应由 ICP 自动解算。

---

## 5. 空中机器人 ID

按照 RoboMaster 裁判系统协议：

| 角色 | 红方 ID | 蓝方 ID |
|---|---|---|
| 空中机器人 | 6 | 106 |

### 敌我判断逻辑

赛场以中线（x = 14m）分割：
- 检测到的目标 x < 14m → 蓝方半场 → `id=106, label="Blue"`
- 检测到的目标 x ≥ 14m → 红方半场 → `id=6, label="Red"`

---

## 6. 关键算法

### 6.1 点云预处理

1. **ROI 裁剪**：限定雷达坐标系 XYZ 范围（排除场外点）
2. **体素降采样**：`voxel_size=0.05m`
3. **高度过滤**：仅保留 `z_min ~ z_max` 范围的点（空中区域，Z 向下为负值越大越高）

### 6.2 背景减除

- 默认 `source: "map"`：加载当前 `scenes.<name>.pcd_file` 点云地图，在 map 坐标系构建 KDTree
- 每帧实时点云通过 TF 变换到 map 帧，点到地图最近邻距离小于 `bg_threshold` 视为背景
- `source: "pcd"` 和 `source: "online"` 仍保留为旧模式，但比赛/常规调试优先使用 map 模式，避免赛前额外录制背景

### 6.3 DBSCAN 聚类

- Z 轴压缩（`z_zip`）：聚类时将 Z 坐标乘以 < 1 的系数，增大水平距离权重
- 这是 HITS 竞赛方案的关键优化，防止高度差导致同一目标被分成多簇

### 6.4 卡尔曼跟踪

- 4 状态 [x, y, vx, vy] 线性卡尔曼滤波
- `cov_factor`：初始协方差因子
- `stop_p_time`：连续无观测帧数达到此值后停止协方差衰减
- Mahalanobis 距离匹配 + 贪心关联 + 强制合并

### 6.5 进一步增强（PFA/HITS 对齐）

- Open3D DBSCAN（默认）替代 sklearn DBSCAN，单帧聚类延迟更低
- 遮挡分裂：当两个空中目标聚成一个大簇时，沿主轴最大间隙尝试二分
- loose query：目标短时丢失后，在上次位置附近放宽参数重检
- 高度自适应：基于最近跟踪高度动态收缩过滤区间
- 置信度过滤：按点数/尺寸给分，剔除低置信度候选
- 双机严格模式开关：
  - `strict_dual_uav=true`：最多两架（红1蓝1，ID固定6/106）
  - `strict_dual_uav=false`：不限制数量，全部发布到小地图用于误检排查

---

## 7. 配置参数

所有参数在 `configs/main_config.yaml` 的 `air_target:` 段：

```yaml
air_target:
  enabled: true                    # 总开关
  publish_rate: 10.0               # Hz

  preprocessing:
    roi: { x_min, x_max, y_min, y_max, z_min, z_max }
    voxel_size: 0.05
    height_filter: { z_min: -1.5, z_max: -0.5 }

  clustering:
    eps: 0.15                      # DBSCAN 邻域半径
    min_samples: 5
    z_zip: 0.5                     # Z 轴压缩

  target_filter:
    min_points: 500
    max_points: 5000
    min_size: 1.0
    max_size: 3.5
    confidence_threshold: 0.1

  tracking:
    enabled: true
    max_lost_frames: 10
    match_distance: 50.0
    kalman: { q_pos, q_vel, ... }

  background:
    enabled: true
    source: "map"
    bg_threshold: 0.15
    voxel_size: 0.10
    learning_frames: 10

  buffer:
    enabled: false
    frame_count: 3
```

---

## 8. EKF 扩展

`ekf_node.py` 当前维护 14 个 EKF slot：

| slot 编号 | 映射 |
|---|---|---|
| 0~4 | 红方地面 1~5 |
| 5 | 红方哨兵 7 |
| 6 | 红方空中 6 |
| 7~11 | 蓝方地面 101~105 |
| 12 | 蓝方哨兵 107 |
| 13 | 蓝方空中 106 |

ID 映射表 `transform_to_th` 使用 1-based slot 编号，红蓝方不再共用同一组数组下标。

---

## 9. 待办事项

- [ ] 裁判系统空中机器人通信协议确认后，更新 `judge_messager.py` 的发送映射与帧格式

### 新增验证/调参工具

- `scripts/validate_air_tracking_sequence.py`：动态点云序列回放，输出检测/跟踪鲁棒性统计
- `scripts/air_param_grid_search.py`：遍历 `eps/min_samples/z_min/z_max` 参数组合
- `scripts/air_visualize_clusters.py`：Open3D 可视化原始点云、聚类结果和包围盒

---

## 10. 来源

| 模块 | 来源 |
|---|---|
| DBSCAN + Z 轴压缩 | HITS-radar-2024 竞赛方案 |
| 卡尔曼滤波 (cov_factor/stop_p_time) | HITS-radar-2024 TargetMap |
| 背景减除 | air_target_radar 仿真项目 |
| TF 坐标变换 | 本项目 lidar_scheme/radar.py（已有方案） |

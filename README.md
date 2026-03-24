# HNURM Radar 2026

RoboMaster 2026 雷达站项目，包含三种方案：地面方案一（纯相机）、地面方案二（相机+激光雷达）、空中方案（纯激光雷达）。

---

## 目录

- [三种方案总览](#三种方案总览)
- [前置准备](#前置准备)
- [地面方案一：纯相机透视变换 (camera_scheme)](#地面方案一纯相机透视变换-camera_scheme)
- [地面方案二：相机+激光雷达融合 (lidar_scheme)](#地面方案二相机激光雷达融合-lidar_scheme)
- [空中方案：纯激光雷达 (air_scheme)](#空中方案纯激光雷达-air_scheme)
- [通用节点说明](#通用节点说明)
- [可视化与调试](#可视化与调试)
- [辅助工具脚本](#辅助工具脚本)
- [关键配置说明](#关键配置说明)
- [依赖安装](#依赖安装)

---

## 三种方案总览

| 维度 | 方案一（纯相机） | 方案二（相机+雷达） | 空中方案（纯雷达） |
|------|:---------------:|:------------------:|:-----------------:|
| **目标类型** | 地面机器人 | 地面机器人 | 空中无人机 |
| **硬件需求** | 仅相机（海康/USB） | 相机 + Livox HAP | 仅 Livox HAP |
| **定位方式** | 透视变换 2D→2D | 点云投影+聚类 2D→3D | 背景减除+DBSCAN 3D |
| **核心节点** | `camera_detector` | `lidar_node` + `detector_node` + `radar_node` | `lidar_node` + `air_target_node` |
| **Launch 文件** | 无专用（手动启动） | `hnurm_radar_launch.py` | `hnurm_air_launch.py` |
| **精度** | 依赖标定质量 | 直接 3D 测量，精度高 | 取决于点云密度 |

三种方案共享下游流程：`→ ekf_node（卡尔曼滤波）→ judge_messager（裁判系统通信）→ display_panel（可视化）`

---

## 前置准备

### 1. 环境激活

所有方案启动前都需要执行：

```bash

# 移除 MVS 库路径，避免 libusb 冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# source ROS 和工作空间
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 2. 场景切换

编辑 `configs/main_config.yaml`：

```yaml
global:
  my_color: "Red"            # 己方颜色："Red" 或 "Blue"
  is_debug: True             # 调试模式
  scene: "lab"               # "competition"=赛场(28×15m) | "lab"=实验室
  debug_coordinate_publish: true  # 调试=true; ⚠ 比赛时必须设为 false!
```

### 3. 串口设备

裁判系统通信需要串口设备，确认已连接：

```bash
ls /dev/ttyUSB* /dev/ttyACM*
```

串口配置在 `configs/main_config.yaml` 的 `communication` 段：

```yaml
communication:
  port: '/dev/ttyUSB0'   # 串口设备路径
  bps: 115200            # 波特率
  timex: 0.01            # 超时时间
```

---

## 地面方案一：纯相机透视变换 (camera_scheme)

利用相机图像通过透视变换（Homography）将像素坐标映射到赛场坐标，**不需要激光雷达**。

### 首次使用：透视标定

方案一需要先进行透视标定，生成标定文件：

```bash
ros2 run hnurm_radar perspective_calibrator
```

标定结果自动保存在 `configs/perspective_calib.json`。

### 启动方式（仅支持逐个启动）

方案一没有专用 launch 文件，需要手动在多个终端中分别启动：

```bash
# === 终端 1：相机检测 + 透视变换定位 ===
ros2 run hnurm_radar camera_detector

# === 终端 2：EKF 扩展卡尔曼滤波 ===
ros2 run ekf ekf_node

# === 终端 3：裁判系统串口通信 ===
ros2 run hnurm_radar judge_messager

```

### 相机输入模式

在 `configs/main_config.yaml` 中配置：

```yaml
camera:
  mode: "hik"           # "hik"=海康工业相机 | "video"=USB摄像头/视频文件 | "test"=静态图片
  video_source: 0       # video 模式的输入源：0=USB摄像头, 或视频文件路径
  test_image: "test_resources/red1.png"  # test 模式图片
```

### 节点启动顺序

| 顺序 | 节点 | 功能 |
|:----:|------|------|
| 1 | `camera_detector` | 相机采集 + YOLO 检测 + 透视变换定位 |
| 2 | `ekf_node` | 接收坐标并进行卡尔曼滤波 |
| 3 | `judge_messager` | 读取滤波结果，通过串口发送给裁判系统 |
| 4 | `display_panel` | 小地图实时可视化 |

---

## 地面方案二：相机+激光雷达融合 (lidar_scheme)

使用 YOLO 在相机图像中检测目标，再利用激光雷达点云投影+聚类获取精确 3D 坐标。**这是比赛主力方案**。

### 启动方式 A：一键脚本

```bash
bash bringup.sh
```

脚本会自动完成以下操作：
1. 移除 MVS 库路径（避免 libusb 冲突）
2. 激活 Python 虚拟环境
3. 等待用户按键确认（确认相机和雷达就绪）
4. 在 3 个独立 gnome-terminal 窗口中依次启动：
   - **窗口 1**：`ros2 launch livox_ros_driver2 rviz_HAP_launch.py`（激光雷达驱动）
   - **窗口 2**：`ros2 launch hnurm_bringup hnurm_radar_launch.py`（主节点组）
   - **窗口 3**：`ros2 launch registration registration.launch.py`（ICP 点云配准 + RViz）

> ⚠ **关键手动步骤**：`registration` 启动后会弹出 RViz 窗口，**必须在 RViz 中点击 "2D Pose Estimate" 按钮并在点云地图上拖拽给出初始位姿估计**，ICP 才能正确配准并发布 `map` TF 变换。如果跳过此步骤，`radar_node` 会持续报错 `获取 TF 失败`。

### 启动方式 B：Launch 文件分步启动

在 3 个终端中**按顺序**执行：

```bash
# === 终端 1：激光雷达驱动（必须最先启动，提供原始点云 /livox/lidar）===
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# === 终端 2：主节点组（一次性启动 6 个节点，其中 lidar_node 会同时发布 /lidar_pcds 和 /target_pointcloud）===
ros2 launch hnurm_bringup hnurm_radar_launch.py

# === 终端 3：ICP 点云配准（必须在终端 2 之后，因为它订阅 /lidar_pcds）===
ros2 launch registration registration.launch.py
# ⚠ 启动后在弹出的 RViz 中点击 "2D Pose Estimate"，在点云地图上拖拽给出初始位姿
# 等待 registration 配准成功（error 值明显下降），此后 radar_node 即可获取 TF
```

> ⚠ **启动顺序很重要**：雷达驱动 → 主节点组（含 `lidar_node`，发布原始累积点云 `/lidar_pcds` 供 `registration` 使用，并发布背景减除后的 `/target_pointcloud` 供 `radar_node` 使用）→ registration（+ 手动给 2D Pose Estimate）。`registration` 订阅的是 `/lidar_pcds` 而非 `/livox/lidar`，`radar_node` 订阅的是 `/target_pointcloud`，如果在 `lidar_node` 之前启动，会因无数据导致配准和融合链路都无法正常工作。

`hnurm_radar_launch.py` 一次启动的 6 个节点：

| 顺序 | 节点 | 功能 |
|:----:|------|------|
| 1 | `lidar_node` | 激光雷达数据接收与点云预处理 |
| 2 | `detector_node` | YOLO 三阶段目标检测（装甲板+车辆+数字） |
| 3 | `radar_node` | 点云-图像融合定位 |
| 4 | `display_panel` | 小地图可视化面板 |
| 5 | `judge_messager` | 裁判系统串口通信 |
| 6 | `ekf_node` | 扩展卡尔曼滤波 |

### 启动方式 C：完全手动逐个启动

适用于需要单独调试某个节点的情况。**按以下顺序启动**：

```bash
# === 终端 1：激光雷达驱动（最先启动，发布 /livox/lidar）===
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# === 终端 2：激光雷达数据接收（订阅 /livox/lidar，发布 /lidar_pcds 和 /target_pointcloud）===
ros2 run hnurm_radar lidar_node

# === 终端 3：ICP 点云配准（订阅 /lidar_pcds，必须在 lidar_node 之后启动）===
ros2 launch registration registration.launch.py
# ⚠ 在 RViz 中点击 "2D Pose Estimate"，拖拽给出初始位姿，等待配准成功

# === 终端 4：YOLO 目标检测 ===
ros2 run hnurm_radar detector_node

# === 终端 5：点云-图像融合定位（订阅 /target_pointcloud）===
ros2 run hnurm_radar radar_node

# === 终端 6：EKF 滤波 ===
ros2 run ekf ekf_node

# === 终端 7（可选）：裁判系统通信（需串口设备）===
ros2 run hnurm_radar judge_messager

# === 终端 8（可选）：小地图可视化 ===
ros2 run hnurm_radar display_panel
```

### 背景地图采集（地面方案二）

地面方案二现在默认启用背景减除。首次部署到新场地、雷达安装位置变化、或背景环境明显变化后，建议先采集一份背景点云：

```bash
python3 scripts/lidar_collect_background.py \
  --topic /livox/lidar \
  --output data/background.pcd \
  --max-frames 120 \
  --voxel-size 0.1
```

采集完成后，[`configs/main_config.yaml`](configs/main_config.yaml) 的 `lidar.background_map_path` 默认就是 `data/background.pcd`，[`src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py`](src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py) 会在启动时自动加载，并将前景点云发布到 `/target_pointcloud`。

### 启动方式 D：离线视频调试

使用录制的视频代替实时相机，用于无硬件环境下调试：

```bash
# === 终端 1：激光雷达驱动 ===
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# === 终端 2：离线调试 launch（发布视频 + 检测 + 定位）===
ros2 launch hnurm_bringup hnurm_radar_video_launch.py
```

`hnurm_radar_video_launch.py` 启动的节点：`lidar_node` → `publish_video` → `detector_node` → `radar_node`

---

## 空中方案：纯激光雷达 (air_scheme)

纯激光雷达方案，通过点云背景减除 + DBSCAN 聚类 + 卡尔曼跟踪识别空中无人机。

### 启动方式 A：Launch 文件一键启动

在 3 个终端中**按顺序**执行：

```bash
# === 终端 1：激光雷达驱动（最先启动，发布 /livox/lidar）===
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# === 终端 2：空中方案主节点组（含 lidar_node，发布 /lidar_pcds）===
ros2 launch hnurm_bringup hnurm_air_launch.py

# === 终端 3：ICP 点云配准（订阅 /lidar_pcds，必须在终端 2 之后启动）===
ros2 launch registration registration.launch.py
# ⚠ 在 RViz 中点击 "2D Pose Estimate"，拖拽给出初始位姿，等待配准成功
```

`hnurm_air_launch.py` 一次启动的 4 个节点：

| 顺序 | 节点 | 功能 |
|:----:|------|------|
| 1 | `lidar_node` | 激光雷达数据接收与点云预处理 |
| 2 | `air_target_node` | 空中目标检测（背景减除+DBSCAN+卡尔曼跟踪） |
| 3 | `display_panel` | 小地图可视化面板 |
| 4 | `ekf_node` | 扩展卡尔曼滤波 |

### 启动方式 B：完全手动逐个启动

**必须按以下顺序启动**：

```bash
# === 终端 1：激光雷达驱动（最先启动，发布 /livox/lidar）===
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# === 终端 2：激光雷达数据接收（订阅 /livox/lidar，发布 /lidar_pcds）===
ros2 run hnurm_radar lidar_node

# === 终端 3：ICP 点云配准（订阅 /lidar_pcds，必须在 lidar_node 之后启动）===
ros2 launch registration registration.launch.py
# ⚠ 在 RViz 中点击 "2D Pose Estimate"，拖拽给出初始位姿，等待配准成功

# === 终端 4：空中目标检测 ===
ros2 run hnurm_radar air_target_node

# === 终端 5：EKF 滤波 ===
ros2 run ekf ekf_node

# === 终端 6（可选）：裁判系统通信（需串口设备）===
ros2 run hnurm_radar judge_messager

# === 终端 7（可选）：小地图可视化 ===
ros2 run hnurm_radar display_panel
```

### 空中方案专属配置

在 `configs/main_config.yaml` 的 `air_target:` 段：

```yaml
air_target:
  enabled: True                # 是否启用空中检测
  publish_rate: 10.0           # 检测发布频率 (Hz)
  strict_dual_uav: false       # true=比赛模式(红1蓝1) | false=调试模式(不限数量)
  max_targets: 0               # 最大目标数，0=不限制
  field_split_x: 14.0          # 赛场中线 x 坐标 (m)
```

---

## 通用节点说明

### judge_messager（裁判系统串口通信）

所有方案都需要此节点与裁判系统通信。

| 功能 | 说明 |
|------|------|
| **发送** | `0x0305` 敌方 6 个机器人赛场坐标（cm），`0x0301→0x0121` 触发双倍易伤 |
| **接收** | `0x0001` 比赛状态，`0x0003` 血量，`0x020C` 标记进度，`0x020E` 双倍易伤，`0x0105` 飞镖 |
| **订阅话题** | `ekf_location_filtered` |
| **串口配置** | 由 `configs/main_config.yaml` 的 `communication` 段控制 |
| **双倍易伤** | 检测到英雄在指定区域连续出现达到阈值后自动触发 |

```bash
ros2 run hnurm_radar judge_messager
```

> ⚠ 需要串口设备已连接（`/dev/ttyUSB0` 或 `/dev/ttyACM0`），否则会报错。无串口时仅其他节点可独立运行。

### ekf_node（扩展卡尔曼滤波）

维护 7 个独立卡尔曼滤波器：

| 滤波器索引 | 机器人 ID |
|:---------:|----------|
| 0~4 | 地面 1-5 / 101-105 |
| 5 | 哨兵 7 / 107 |
| 6 | 空中机器人 6 / 106 |

```bash
ros2 run ekf ekf_node
```

### display_panel（小地图可视化面板）

在赛场地图上实时显示检测到的机器人位置。

```bash
ros2 run hnurm_radar display_panel
```

---

## 可视化与调试

| 方式 | 启动命令 | 说明 |
|------|---------|------|
| **display_panel 小地图** | 随 launch 自动启动，或 `ros2 run hnurm_radar display_panel` | 赛场地图实时显示机器人位置 |
| **RViz 点云调试** | `rviz2`，添加话题 `air_debug/cluster_points`、`air_debug/cluster_markers` | 查看空中方案点云聚类结果 |
| **ICP 配准 RViz** | `ros2 launch registration registration.launch.py` | 自带 RViz，使用 `configs/icp.rviz` 配置 |
| **Camera 窗口** | 方案一/二自动弹出 | 相机画面 + 检测框 + 坐标标注 |
| **Foxglove Bridge** | 见下方命令 | Web 远程可视化 |

### 启动 Foxglove Bridge（远程可视化）

```bash
source install/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

然后在浏览器中打开 [Foxglove Studio](https://studio.foxglove.dev/)，连接到 `ws://localhost:8765`。

### 录包回放调试

```bash
# 录包
ros2 bag record /livox/lidar /detect_result /location /ekf_location_filtered

# 回放
ros2 bag play <bag_directory>
```

---

## 辅助工具脚本

| 脚本 | 用途 | 使用方式 |
|------|------|---------|
| `scripts/air_param_grid_search.py` | 空中方案 DBSCAN 参数网格搜索自动调优 | `python scripts/air_param_grid_search.py` |
| `scripts/air_save_debug_pcd.py` | 保存空中检测结果 PCD 快照 | `python scripts/air_save_debug_pcd.py` |
| `scripts/air_visualize_clusters.py` | Open3D 可视化空中目标聚类结果 | `python scripts/air_visualize_clusters.py` |
| `scripts/align_pcd_with_map.py` | 交互式点云-地图对齐工具 | `python scripts/align_pcd_with_map.py` |
| `scripts/calibrate_bev_coords.py` | BEV 地图坐标系交互式标定 | `python scripts/calibrate_bev_coords.py` |
| `scripts/check_points.py` | 地图像素坐标查看小工具 | `python scripts/check_points.py` |
| `scripts/prepare_lab_map.py` | 实验室地图裁剪与坐标系建立 | `python scripts/prepare_lab_map.py` |
| `scripts/test_yolo_3stage.py` | YOLO 三阶段推理效果测试 | `python scripts/test_yolo_3stage.py` |
| `scripts/validate_air_tracking_sequence.py` | 空中目标跟踪鲁棒性验证 | `python scripts/validate_air_tracking_sequence.py` |

### 标定工具（ROS 节点）

```bash
# 透视标定（方案一必需）
ros2 run hnurm_radar perspective_calibrator

# 制作掩膜
ros2 run hnurm_radar make_mask
```

---

## 关键配置说明

所有配置集中在 `configs/main_config.yaml`，主要段落：

| 配置段 | 控制内容 |
|--------|---------|
| `global` | 颜色、调试模式、场景选择 |
| `scenes` | 赛场/实验室场景切换（地图、PCD 文件路径） |
| `camera` | 相机输入模式（海康/USB/测试图片） |
| `car` | 地面目标生命周期 |
| `lidar` | 激光雷达过滤参数、话题名 |
| `communication` | 串口设备路径、波特率 |
| `air_target` | 空中方案全部参数（预处理/背景/聚类/跟踪） |

---

## 依赖安装

```bash
# Livox SDK 依赖
sudo apt install libpkgconf
sudo apt install libpcap-dev

# Python 串口通信（注意：是 pyserial，不是 serial）
pip install pyserial

# 其他 Python 依赖
pip install -r requirements.txt
```

---

## 快速参考：各方案最简启动命令

```bash
# ======================== 通用前置 ========================
source /data/venv/radar-env/bin/activate
source /opt/ros/humble/setup.bash
source install/setup.bash
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# ===================== 方案一（纯相机）=====================
ros2 run hnurm_radar camera_detector    # 终端 1
ros2 run ekf ekf_node                   # 终端 2
ros2 run hnurm_radar judge_messager     # 终端 3
ros2 run hnurm_radar display_panel      # 终端 4（可选）

# =================== 方案二（相机+雷达）====================
# 快速启动（一键脚本）：
bash bringup.sh
# ⚠ 启动后必须在 RViz 中给 2D Pose Estimate！

# 或者 launch 分步启动（按顺序）：
ros2 launch livox_ros_driver2 rviz_HAP_launch.py         # 终端 1：雷达驱动
ros2 launch hnurm_bringup hnurm_radar_launch.py          # 终端 2：主节点组（含 lidar_node）
ros2 launch registration registration.launch.py          # 终端 3：ICP 配准（⚠ RViz 给 2D Pose Estimate）

# =================== 空中方案（纯雷达）====================
ros2 launch livox_ros_driver2 rviz_HAP_launch.py         # 终端 1：雷达驱动
ros2 launch hnurm_bringup hnurm_air_launch.py            # 终端 2：空中方案（含 lidar_node）
ros2 launch registration registration.launch.py          # 终端 3：ICP 配准（⚠ RViz 给 2D Pose Estimate）
```

# 🚀 快速开始指南

本指南帮助新成员在 **15 分钟内**完成首次运行。

---

## 目录

- [环境准备](#环境准备)
- [方案二启动（主力方案）](#方案二启动主力方案)
- [方案一启动（纯相机）](#方案一启动纯相机)
- [空中方案启动](#空中方案启动)
- [离线视频调试](#离线视频调试)
- [常见问题](#常见问题)
- [辅助工具脚本](#辅助工具脚本)
- [交互式外参调整](#交互式外参调整)

---

## 环境准备

### 1. 激活环境

**每次打开新终端都需要执行：**

```bash
# （可选）如果使用了 Python 虚拟环境，先激活它
# source /data/venv/radar-env/bin/activate

# 移除 MVS 库路径，避免 libusb 冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# source ROS 和工作空间
source /opt/ros/humble/setup.bash
source install/setup.bash
```

> **注意**：Python 虚拟环境是可选的。如果依赖已安装在系统 Python 中，可跳过 `source .../activate` 步骤。

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

裁判系统通信需要串口设备：

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

### 4. 相机输入模式

在 `configs/main_config.yaml` 中配置：

```yaml
camera:
  mode: "hik"           # "hik"=海康工业相机 | "video"=USB摄像头/视频文件 | "test"=静态图片
  video_source: 0       # video 模式的输入源：0=USB摄像头, 或视频文件路径
  test_image: "test_resources/red1.png"  # test 模式图片
```

---

## 方案二启动（主力方案）

使用相机+激光雷达融合定位。**这是比赛主力方案**。

### 一键启动

```bash
bash bringup.sh
```

脚本自动完成：
1. 移除 MVS 库路径
2. 激活虚拟环境
3. 启动 3 个 gnome-terminal 窗口：
   - **窗口 1**：激光雷达驱动
   - **窗口 2**：主节点组（6 个节点）
   - **窗口 3**：ICP 点云配准 + RViz

> ⚠ **关键手动步骤**：`registration` 启动后会弹出 RViz 窗口，**必须点击 "2D Pose Estimate" 按钮并在点云地图上拖拽给出初始位姿估计**，ICP 才能正确配准。

### Launch 分步启动

在 3 个终端中**按顺序**执行：

```bash
# 终端 1：激光雷达驱动
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# 终端 2：主节点组
ros2 launch hnurm_bringup hnurm_radar_launch.py

# 终端 3：ICP 点云配准
ros2 launch registration registration.launch.py
# ⚠ 在 RViz 中点击 "2D Pose Estimate"
```

`hnurm_radar_launch.py` 启动的 6 个节点：

| 节点 | 功能 |
|------|------|
| `lidar_node` | 激光雷达数据接收与点云预处理 |
| `detector_node` | YOLO 三阶段目标检测 |
| `radar_node` | 点云-图像融合定位 |
| `display_panel` | 小地图可视化 |
| `judge_messager` | 裁判系统串口通信 |
| `ekf_node` | 扩展卡尔曼滤波 |

### 背景地图采集

首次部署到新场地时，需要先采集背景点云：

```bash
python3 scripts/lidar_collect_background.py \
  --topic /livox/lidar \
  --output data/pointclouds/background/background.pcd \
  --max-frames 120 \
  --voxel-size 0.1
```

---

## 方案一启动（纯相机）

利用透视变换（Homography）将像素坐标映射到赛场坐标，**不需要激光雷达**。

### 首次使用：透视标定

```bash
ros2 run hnurm_radar perspective_calibrator
```

标定结果保存在 `configs/perspective_calib.json`。

### 逐个启动

```bash
# 终端 1：相机检测 + 透视变换定位
ros2 run hnurm_radar camera_detector

# 终端 2：EKF 扩展卡尔曼滤波
ros2 run ekf ekf_node

# 终端 3：裁判系统串口通信
ros2 run hnurm_radar judge_messager

# 终端 4（可选）：小地图可视化
ros2 run hnurm_radar display_panel
```

---

## 空中方案启动

纯激光雷达方案，通过点云背景减除 + DBSCAN 聚类 + 卡尔曼跟踪识别空中无人机。

```bash
# 终端 1：激光雷达驱动
ros2 launch livox_ros_driver2 rviz_HAP_launch.py

# 终端 2：空中方案主节点组
ros2 launch hnurm_bringup hnurm_air_launch.py

# 终端 3：ICP 点云配准
ros2 launch registration registration.launch.py
# ⚠ 在 RViz 中点击 "2D Pose Estimate"
```

`hnurm_air_launch.py` 启动的 4 个节点：`lidar_node`、`air_target_node`、`display_panel`、`ekf_node`

空中方案配置在 `configs/main_config.yaml` 的 `air_target:` 段，详见 [空中方案指南](AIR_SCHEME_GUIDE.md)。

---

## 离线视频调试

通过配置文件切换为视频输入模式，无需额外的 launch 文件：

```yaml
# configs/main_config.yaml
camera:
  mode: "video"                        # 切换为视频文件输入
  video_source: "test_resources/xxx.mp4"  # 视频文件路径（相对于项目根目录）
```

然后按正常方式启动即可：

```bash
# 方案二（激光雷达+相机融合）
./bringup.sh

# 方案一（纯相机透视变换）
ros2 run hnurm_radar camera_detector
```

> **注意：** 使用完毕后记得将 `camera.mode` 改回 `"hik"` 以恢复实时相机输入。

---

## 可视化与调试

| 方式 | 说明 |
|------|------|
| **display_panel 小地图** | 赛场地图实时显示机器人位置 |
| **RViz 点云调试** | 添加话题 `air_debug/cluster_points` |
| **Camera 窗口** | 方案一/二自动弹出检测画面 |
| **Foxglove Studio** | 桌面应用 / Web 远程可视化（预配置布局） |
| **录包回放** | `ros2 bag record` / `ros2 bag play` |

### Foxglove Studio

系统已安装 Foxglove Studio 桌面应用和 foxglove_bridge。

**方式一：桌面应用（推荐，带完整配置面板）**

```bash
# 终端 1：启动 bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# 终端 2：启动桌面应用
foxglove-studio
# 或从系统菜单栏搜索 "Foxglove Studio" 打开
```

打开后选择 **Open connection** → `Foxglove WebSocket` → `ws://localhost:8765`

**方式二：浏览器版**

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
# 浏览器打开 https://app.foxglove.dev/ → 连接 ws://localhost:8765
```

**导入预配置布局：**

项目提供了预配置的 Foxglove 布局文件 `configs/foxglove_layout.json`，包含：
- 3D 点云面板（lidar_pcds / target_pointcloud / global_pcd_map / 空中聚类等）
- 检测画面面板（/detect_view）
- 小地图面板（/map_view — 需运行 `display_panel` 节点）
- EKF 滤波坐标时序图（X / Y 分离，7 个滤波器全覆盖）
- 原始检测坐标时序图（/location 未滤波数据）
- 原始消息查看器（/location / /ekf_location_filtered / /detect_result / air_debug/stats）

导入方法：Foxglove Studio → 左上角菜单 → **Import layout from file** → 选择 `configs/foxglove_layout.json`

### 录包

```bash
ros2 bag record /livox/lidar /detect_result /location /ekf_location_filtered
ros2 bag play <bag_directory>
```

---

## 常见问题

### Q: 启动报错 "get one frame fail"
**A:** 相机未连接或 MVS 库冲突。执行 `export LD_LIBRARY_PATH=...` 移除 MVS 路径。

### Q: radar_node 报 "获取 TF 失败"
**A:** registration 未配准。在 RViz 中点击 "2D Pose Estimate" 给初始位姿。

### Q: 串口连接失败
**A:** 检查 `/dev/ttyUSB0` 是否存在，确认 `configs/main_config.yaml` 中的 `communication.port`。

### Q: 检测不到目标 / 检测效果差
**A:** 检查模型权重路径（`configs/detector_config.yaml`），确认场景设置正确。

### Q: 如何在没有硬件的情况下测试？
**A:** 使用 `camera.mode: "video"` 或 `"test"` 模式，跳过相机和激光雷达。

---

## 辅助工具脚本

| 脚本 | 用途 | 使用方式 |
|------|------|---------|
| `scripts/test_yolo_3stage.py` | YOLO 三阶段推理效果测试 | `python3 scripts/test_yolo_3stage.py` |
| `scripts/benchmark_model.py` | 模型推理性能基准测试 | `python3 scripts/benchmark_model.py` |
| `scripts/export_models.py` | 模型导出 | `python3 scripts/export_models.py` |
| `scripts/align_pcd_with_map.py` | 交互式点云-地图对齐 | `python3 scripts/align_pcd_with_map.py` |
| `scripts/calibrate_lab_map.py` | 实验室地图标定 | `python3 scripts/calibrate_lab_map.py` |
| `scripts/lidar_collect_background.py` | 背景点云采集 | 见上方背景地图采集 |
| `scripts/air_param_grid_search.py` | 空中方案参数调优 | `python3 scripts/air_param_grid_search.py` |
| `scripts/air_save_debug_pcd.py` | 保存空中检测 PCD 快照 | `python3 scripts/air_save_debug_pcd.py` |
| `scripts/air_visualize_clusters.py` | 可视化空中目标聚类 | `python3 scripts/air_visualize_clusters.py` |
| `scripts/validate_air_tracking_sequence.py` | 空中跟踪鲁棒性验证 | `python3 scripts/validate_air_tracking_sequence.py` |
| `scripts/lidar_cam_projection_debug.py` | 雷达-相机外参调试（含交互式调整） | 见下方 [交互式外参调整](#交互式外参调整) |
| `scripts/derive_lidar_camera_extrinsic_from_homography.py` | 从 Homography 反推外参初值 | `python3 scripts/derive_lidar_camera_extrinsic_from_homography.py` |
| `scripts/record_initial_pose.py` | 记录 ICP 配准初始位姿 | `python3 scripts/record_initial_pose.py` |

### 标定工具（ROS 节点）

```bash
# 透视标定（方案一必需）
ros2 run hnurm_radar perspective_calibrator

# 制作掩膜
ros2 run hnurm_radar make_mask
```

---

## 交互式外参调整

当雷达-相机外参（`converter_config.yaml` 或 `converter_config_rosbag.yaml` 中的 R/T）不正确时，可以使用 `lidar_cam_projection_debug.py` 的 **overlay3d** 模式进行实时交互式调整。

### 前置条件

- rosbag 正在播放（或实时运行雷达+相机）
- registration 节点已完成 ICP 配准（如果使用动态外参模式）

### 启动

```bash
# 基本用法（使用静态外参）
python3 scripts/lidar_cam_projection_debug.py \
  --config configs/converter_config.yaml \
  --render-mode overlay3d

# rosbag 模式（使用动态外参 = solvePnP × TF）
python3 scripts/lidar_cam_projection_debug.py \
  --config configs/converter_config_rosbag.yaml \
  --render-mode overlay3d \
  --dynamic-extrinsic

# 推荐：累积 50 帧静态点云后冻结，使用稳定点云调整
python3 scripts/lidar_cam_projection_debug.py \
  --config configs/converter_config_rosbag.yaml \
  --render-mode overlay3d \
  --dynamic-extrinsic \
  --accumulate-frames 50

# 从零标定外参（忽略 YAML 外参，以雷达视角起点开始手动调整）
python3 scripts/lidar_cam_projection_debug.py \
  --config configs/converter_config_rosbag.yaml \
  --render-mode overlay3d \
  --identity-init \
  --accumulate-frames 50
```

> **`--identity-init` 说明**：忽略 YAML 配置中的 R/T 外参，改用标准坐标轴转换矩阵
> （Livox X=前→相机Z，Livox Y=左→相机-X，Livox Z=上→相机-Y）+ 零平移作为初始值。
> 相当于"站在雷达位置看"的视角起点，适合从零开始手动标定外参。

### 交互方式

启动后会弹出两个窗口：
- **overlay3d**：实时显示点云（深度着色）+ 相机图像叠加效果。点云颜色按深度渐变：**红色=近处，蓝色=远处**（JET colormap）
- **Extrinsic Adjustment**：包含 7 个滑块（tx/ty/tz/roll/pitch/yaw/alpha），作为辅助微调

#### 🖱️ 鼠标操作（主要交互方式，类似 Foxglove）

直接在 **overlay3d** 窗口上操作，无需切换窗口：

| 操作 | 功能 | 说明 |
|------|------|------|
| **左键拖拽** | 旋转（yaw / pitch） | 水平拖 = 左右旋转，垂直拖 = 上下旋转 |
| **右键拖拽** | 平移（tx / ty） | 平移点云位置 |
| **滚轮** | 缩放（tz） | 前后移动点云 |
| **Ctrl + 左键拖拽** | Roll 旋转 | 旋转画面倾斜角 |
| **中键拖拽** | 平移（同右键） | 备选平移方式 |

#### Trackbar 滑块（辅助微调）

| 滑块 | 范围 | 精度 | 说明 |
|------|------|------|------|
| tx / ty / tz | ±20.0m | 0.1m | 平移调整 |
| roll / pitch / yaw | ±90.0° | 0.1° | 旋转调整 |
| alpha | 0~100% | 1% | 图像透明度 |

#### 键盘快捷键（辅助）

| 按键 | 功能 |
|------|------|
| `A`/`D`, `W`/`S`, `R`/`F` | 平移 tx/ty/tz（步进式） |
| `J`/`L`, `I`/`K`, `U`/`O` | 旋转 yaw/pitch/roll（步进式） |
| `[` / `]` | 切换步长（COARSE 0.5m/2° → MEDIUM 0.1m/0.5° → FINE 0.02m/0.1°） |
| `+` / `-` | 调整图像透明度 |
| `0` | 重置所有增量为零 |
| `P` | 打印当前 R/T 到终端 |
| `Enter` | **保存**到 YAML 配置文件 |
| `Q` | 退出 |

### 点云累积模式

添加 `--accumulate-frames N` 参数后，脚本会累积前 N 帧点云数据，合并去重后冻结。冻结后的点云和图像都不再更新，可以稳定地调整外参。

```bash
# 累积 30 帧（约 3 秒 @10Hz）
--accumulate-frames 30

# 累积 100 帧（更完整的点云覆盖）
--accumulate-frames 100
```

HUD 中会显示累积状态：`[accum 15/50]`（累积中）或 `[FROZEN 50f]`（已冻结）。

### 使用流程

1. 启动后等待点云累积完成（如果使用了 `--accumulate-frames`）
2. 在 overlay3d 窗口用**鼠标左键拖拽**旋转点云，使方向大致对齐
3. 用**右键拖拽**平移点云位置，用**滚轮**调整深度
4. 用 **Ctrl+左键** 微调 roll 角度
5. 对齐后按 `P` 确认当前外参数值
6. 满意后按 **Enter** 保存到 YAML 文件

> **提示**：HUD 区域会实时显示当前增量。增量不为零时 HUD 文字变为绿色。点云深度着色帮助你区分远近物体。

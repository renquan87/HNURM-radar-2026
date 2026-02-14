# HNURM Radar 2026 — 使用说明

> 本文档说明如何部署、配置和运行 HNURM 雷达站系统。

---

## 1. 环境要求

### 1.1 硬件

| 组件 | 要求 |
|---|---|
| 计算平台 | x86_64 Linux，推荐 NVIDIA GPU（CUDA 支持） |
| 相机 | 海康工业相机（MV-CS 系列）或 USB 摄像头 |
| 激光雷达 | Livox HAP（仅相机+雷达方案需要） |
| 串口设备 | USB 转串口（默认 `/dev/ttyUSB0`），连接裁判系统 |

### 1.2 软件

| 软件 | 版本 |
|---|---|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble Hawksbill |
| Python | 3.10+ |
| CUDA | 11.x / 12.x（推荐） |
| OpenCV | 4.x（含 contrib） |

### 1.3 Python 依赖

```bash
pip install -r requirements.txt
```

主要依赖：
- `ultralytics` — YOLO 推理框架（项目内嵌本地修改版）
- `opencv-python` / `opencv-contrib-python`
- `numpy`, `cupy` — 数值计算与 GPU 加速
- `open3d` — 点云处理（相机+雷达方案）
- `pyserial` — 串口通信
- `ruamel.yaml` — YAML 配置解析
- `PyQt5` — 标定工具 GUI
- `filterpy` — 卡尔曼滤波
- `scipy` — 科学计算

---

## 2. 项目部署

### 2.1 克隆项目

```bash
cd /data/projects/radar
git clone https://github.com/renquan87/HNURM-RADAR2026.git hnurm_radar
cd hnurm_radar
```

> 项目代码中配置路径硬编码为 `/data/projects/radar/hnurm_radar/`，请确保项目位于此路径。若需部署到其他路径，需修改以下文件中的路径引用：
> - `configs/main_config.yaml`
> - `configs/detector_config.yaml`
> - `configs/converter_config.yaml`
> - `configs/HAP_config.json`
> - `src/hnurm_radar/hnurm_radar/camera_scheme/camera_detector.py` 中的常量
> - `src/hnurm_radar/hnurm_radar/shared/judge_messager.py` 中的配置路径
> - `src/hnurm_radar/hnurm_radar/lidar_scheme/detector_node.py` 中的路径
> - `src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py` 中的路径
> - `src/registration/params/default.yaml`

### 2.2 安装依赖

```bash
# Python 依赖
pip install -r requirements.txt

# ROS2 依赖（如缺少）
sudo apt install ros-humble-cv-bridge ros-humble-sensor-msgs
```

### 2.3 编译

```bash
source /opt/ros/humble/setup.bash
cd /data/projects/radar/hnurm_radar
colcon build --symlink-install
source install/setup.bash
```

> `bringup.sh` 是一键启动脚本（非单纯编译），它会激活虚拟环境、source ROS2 环境，然后在多个 gnome-terminal 窗口中分别启动相机+雷达方案的全部节点（`livox_ros_driver2`、`hnurm_radar_launch.py`、`registration`）。详见第 4.2 节。

---

## 3. 配置说明

所有配置文件位于 `configs/` 目录。

### 3.1 主配置 `main_config.yaml`

```yaml
global:
  my_color: 'Blue'      # 己方颜色：'Red' 或 'Blue'
  is_debug: true         # 调试模式开关（输出详细日志）

camera:
  mode: 'test'           # 相机模式：'test' / 'video' / 'hik'
  video_source: 0        # video 模式下的视频源（设备号或文件路径）
  test_image: '/data/projects/radar/hnurm_radar/test_resources/test1.png'

car:
  life_span: 20          # 车辆信息可信生命周期（帧数）

communication:
  port: '/dev/ttyUSB0'   # 串口设备路径
  bps: 115200            # 波特率
  timex: 0.01            # 超时时间

lidar:
  height_threshold: 5.5  # 点云高度过滤阈值
  min_distance: 1        # 最小接收距离（m）
  max_distance: 40       # 最大接收距离（m）
  lidar_topic_name: '/livox/lidar'  # 点云话题名

area:                    # 英雄区域检测配置（双倍易伤触发）
  hero_times_threshold: 2
  send_double_threshold: 10
  Blue: { ... }          # 蓝方时敌方英雄区域多边形
  Red: { ... }           # 红方时敌方英雄区域多边形
```

**相机模式说明**：

| 模式 | 说明 | 适用场景 |
|---|---|---|
| `test` | 使用静态图片反复推理 | 开发调试、无相机环境 |
| `video` | USB 摄像头或视频文件 | 录像回放、USB 相机测试 |
| `hik` | 海康工业相机 | 比赛现场 |

### 3.2 检测配置 `detector_config.yaml`

```yaml
path:
  stage_one_path: '/data/projects/radar/hnurm_radar/weights/stage_one.pt'
  stage_two_path: '/data/projects/radar/hnurm_radar/weights/stage_two.pt'
  stage_three_path: '/data/projects/radar/hnurm_radar/weights/stage_three.pt'
  tracker_path: '/data/projects/radar/hnurm_radar/configs/bytetrack.yaml'

params:
  stage_one_conf: 0.3    # 第一阶段检测置信度阈值
  stage_two_conf: 0.3    # 第二阶段分类置信度阈值
  stage_three_conf: 0.3  # 第三阶段分类置信度阈值
  life_time: 100         # 追踪器投票衰减周期
  labels: [...]          # 标签列表

filter:
  process_noise: 0.01    # 卡尔曼滤波过程噪声
  measurement_noise: 0.1 # 卡尔曼滤波观测噪声
  jump_threshold: 3.0    # 坐标跳变阈值（m）
  max_inactive_time: 3.0 # 滤波器超时清理时间（s）
```

### 3.3 激光雷达配置 `HAP_config.json`

Livox HAP 激光雷达连接参数，包含雷达 IP 地址等。仅在相机+雷达方案中使用。

### 3.4 坐标转换配置 `converter_config.yaml`

相机内外参、点云滤波参数、聚类参数等。仅在相机+雷达方案中使用。

---

## 4. 运行方式

### 4.1 方案一：纯相机透视变换方案

此方案不依赖激光雷达，仅使用相机图像 + 透视变换进行定位。

> 注意：方案一目前没有专用的 launch 文件，需要手动分别启动各节点。

#### 步骤 1：透视变换标定

首次使用或更换相机位置后，需要进行透视变换标定：

```bash
source install/setup.bash
ros2 run hnurm_radar perspective_calibrator
```

标定工具（PyQt5 GUI）操作流程：
1. 点击「开始标定」冻结画面
2. 在左侧相机图像上点击 4 个以上地面特征点
3. 在右侧赛场地图上依次点击对应位置
4. （可选）点击「切换到高地层」，标定高地区域的 4+ 个点
5. 点击「保存计算」，标定结果保存到 `configs/perspective_calib.json`

#### 步骤 2：启动系统

分别在不同终端中启动：

```bash
source install/setup.bash

# 终端 1：相机检测 + 透视变换定位
ros2 run hnurm_radar camera_detector

# 终端 2：EKF 坐标滤波
ros2 run ekf ekf_node

# 终端 3：裁判系统通信
ros2 run hnurm_radar judge_messager
```

### 4.2 方案二：相机 + 激光雷达方案（一键脚本）

此方案使用海康相机 + Livox HAP 激光雷达联合定位。推荐使用一键脚本启动：

```bash
# 推荐：一键脚本（自动在多个 gnome-terminal 窗口中启动全部节点）
bash bringup.sh
```

`bringup.sh` 会依次在独立终端中启动以下三个 launch：

1. `ros2 launch livox_ros_driver2 rviz_HAP_launch.py` — 激光雷达驱动 + RViz 可视化
2. `ros2 launch hnurm_bringup hnurm_radar_launch.py` — 主要节点组
3. `ros2 launch registration registration.launch.py` — 点云配准

其中 `hnurm_radar_launch.py` 启动以下节点：
- `lidar_node` — 雷达数据处理
- `detector_node` — YOLO 检测
- `radar_node` — 雷达主节点
- `display_panel` — 操作手面板
- `judge_messager` — 裁判系统通信
- `ekf_node` — EKF 坐标滤波

另有 `hnurm_radar_video_launch.py` 用于视频回放调试（不启动激光雷达驱动）：

```bash
ros2 launch hnurm_bringup hnurm_radar_video_launch.py
```

该 launch 启动：`lidar_node`、`publish_video`（视频文件模拟相机）、`detector_node`、`radar_node`

### 4.3 单独运行节点

所有节点均需先 `source install/setup.bash`。

#### 方案一（纯相机）相关节点

```bash
# 相机检测 + 透视变换定位（方案一核心节点）
ros2 run hnurm_radar camera_detector

# 透视变换标定工具
ros2 run hnurm_radar perspective_calibrator
```

#### 方案二（相机+雷达）相关节点

```bash
# YOLO 检测 + 点云匹配（方案二核心节点）
ros2 run hnurm_radar detector_node

# 雷达数据处理
ros2 run hnurm_radar lidar_node
```

#### 通用节点（两种方案共用）

```bash
# EKF 坐标滤波
ros2 run ekf ekf_node

# 裁判系统串口通信
ros2 run hnurm_radar judge_messager

# 操作手显示面板
ros2 run hnurm_radar display_panel
```

---

## 5. 比赛现场部署流程

### 5.1 赛前准备

1. 确认 `configs/main_config.yaml` 中 `my_color` 设置正确（`Red` 或 `Blue`）
2. 将 `camera.mode` 设置为 `hik`
3. 确认串口设备已连接裁判系统（默认 `/dev/ttyUSB0`，可在 `main_config.yaml` 中修改）
4. 确认 YOLO 模型权重文件存在于 `weights/` 目录

### 5.2 标定

1. 将相机固定到雷达站位置
2. 运行标定工具：`ros2 run hnurm_radar perspective_calibrator`
3. 选取赛场上可辨识的地面特征点（如场地线交叉点、标志物等）进行标定
4. 如有高地区域需要覆盖，切换到高地层继续标定
5. 保存标定结果

### 5.3 启动

```bash
bash bringup.sh
```

### 5.4 运行时监控

系统运行后会显示两个窗口：
- **CameraDetector** — 相机画面 + 检测框 + 赛场坐标标注
- **MiniMap** — 小地图实时显示机器人位置

裁判系统通信节点会在终端输出：
- 比赛剩余时间
- 敌方机器人血量
- 标记进度
- 双倍易伤状态

---

## 6. 串口通信说明

`judge_messager` 节点通过串口（默认 `/dev/ttyUSB0`，115200 波特率）与裁判系统通信。串口路径在 `configs/main_config.yaml` 的 `communication.port` 中配置。

### 6.1 发送数据

| 命令 ID | 说明 |
|---|---|
| `0x0305` | 敌方机器人小地图坐标（6 个机器人的 x, y） |
| `0x0301` → `0x0121` | 触发双倍易伤 |

### 6.2 接收数据

| 命令 ID | 说明 |
|---|---|
| `0x0001` | 比赛状态（剩余时间） |
| `0x0003` | 机器人血量信息 |
| `0x020C` | 雷达标记进度 |
| `0x020E` | 双倍易伤状态 |
| `0x0105` | 飞镖目标信息 |

通信协议遵循 RoboMaster 裁判系统 v1.4 协议，使用 CRC8（帧头校验）+ CRC16（整帧校验）。

---

## 7. 常见问题

### Q: 启动时报错 "未找到透视变换标定文件"
先运行标定工具：
```bash
ros2 run hnurm_radar perspective_calibrator
```
完成标定后重新启动。

### Q: 串口打开失败
检查串口设备是否存在：
```bash
ls -la /dev/ttyUSB* /dev/ttyACM*
```
如设备名不同，需修改 `configs/main_config.yaml` 中 `communication.port` 的值。也可能需要添加用户到 `dialout` 组：
```bash
sudo usermod -aG dialout $USER
```

### Q: 海康相机无法连接
1. 确认相机已通过网线连接并配置了正确的 IP
2. 确认已安装海康 MVS SDK
3. 检查 `Camera/HKCam.py` 中的相机索引

### Q: YOLO 推理速度慢
1. 确认 CUDA 和 cuDNN 已正确安装
2. 检查 `nvidia-smi` 确认 GPU 可用
3. 可在 `detector_config.yaml` 中调高置信度阈值减少检测数量

### Q: 小地图坐标偏差大
1. 重新运行标定工具，选取更多、更分散的特征点
2. 确保标定点覆盖赛场主要区域
3. 如有高地区域偏差，需单独标定高地层

### Q: colcon build 编译失败
```bash
# 清理编译缓存后重试
rm -rf build/ install/ log/
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

---

## 8. 目录权限说明

| 目录/文件 | 权限要求 |
|---|---|
| 串口设备（`/dev/ttyUSB0`） | 需要 `dialout` 组权限 |
| `configs/` | 读写（标定工具会写入 `perspective_calib.json`） |
| `weights/` | 只读 |
| `record/` | 读写（运行时录制） |
| `map/` | 只读 |

---

*文档版本: 2026-02-14*

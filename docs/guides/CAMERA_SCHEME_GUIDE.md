# HNURM Radar 2026 纯相机透视变换方案（README）

## 1. 框架总览

### 1.1 纯相机方案文件结构

```text
HNURM-radar-2026/
├── configs/
│   ├── detector_config.yaml                    # 检测/匹配主参数
│   ├── perspective_calib.json                  # Homography 透视标定结果
│   ├── raycast_calib.yaml                      # 射线求交 PnP 标定结果
│   └── main_config.yaml                        # 全局配置 (含坐标映射模式)
├── field/
│   ├── RMUC2025_National.PLY                   # 全国赛赛场 PLY 网格 (Solidworks 坐标)
│   ├── RMUC2025_Regional.PLY                   # 区域赛赛场 PLY 网格
│   ├── keypoint_6.txt                          # 全国赛 PnP 标定点 (6 个 3D 坐标)
│   └── keypoint_6_region.txt                   # 区域赛 PnP 标定点
├── scripts/
│   └── ...
└── src/
  ├── hnurm_radar/hnurm_radar/
  │   ├── camera_locator/
  │   │   ├── perspective_calibrator.py       # Homography 标定工具
  │   │   └── raycast_calibrator.py           # 射线求交 PnP 标定工具
  │   ├── camera_scheme/
  │   │   ├── camera_detector.py              # 主流程：检测/去重/映射/发布
  │   │   ├── hungarian_tracker.py            # 匈牙利匹配与轨迹状态管理
  │   │   └── guess_pts.py                    # GUESSING 长时外推（效果不好目前未发布GUESSING坐标）
  │   ├── filters/
  │   │   └── bbox_kalman.py                  # bbox 像素域卡尔曼滤波（二维）
  │   ├── mapping/                            # 坐标映射抽象层
  │   │   ├── base.py                         # CoordinateMapper 抽象基类
  │   │   ├── homography_mapper.py            # Homography 透视变换实现
  │   │   ├── raycast_mapper.py               # 射线-网格求交实现
  │   │   └── solidworks_to_field.py          # Solidworks→裁判系统坐标变换
  │   └── shared/
  │       ├── type.py                         # 共享数据结构与状态机枚举
  │       └── utils.py                        # IoU/NMS/坐标工具函数
  └── ekf/ekf/
    └── ekf_node.py                         # 坐标级 EKF 平滑
```

### 1.2 数据流（纯相机主链路）

```mermaid
相机输入帧 / 视频输入帧
          |
          v
camera_detector (YOLO三阶段检测)
          |
          v
   BoT-SORT 目标跟踪器
 (提取底层帧间 track_id)
          |
          v
    NMS + 简单去重
 (将 track_id 压入标准化 Detection)
          |
          v
 hungarian_tracker.update
 (卡尔曼滤波平滑 + ID关联推演)
          |
    +-----+-----+
    |           |
    v           v
bbox_kalman    轨迹状态机 (TRACKING / LOST / GUESSING)
(短时平滑与预测)    |
    |           v
    |          guess_pts (长时外推)
    v
取滤波框底边中心
    |
    v
pixel_to_field (路由: Homography 或 Raycast)
    |
    v
/location 原始坐标发布
    |
    v
ekf_node 坐标级滤波
    |
    v
/ekf_location_filtered
    |
    v
小地图绘制
```

## 2. 坐标映射方案

camera_detector 支持两种像素→赛场坐标映射后端，通过 `main_config.yaml` 中 `coordinate_mapping.mode` 配置切换。

### 2.1 方案 A: Homography 透视变换 (mode = "homography")

原理：通过 4+ 个像素↔地图对应点计算单应性矩阵 H，将像素直接投影到赛场平面。
支持地面层和高地层两套 H 矩阵，通过掩码图判定高度区域。

优点：标定简单直观，仅需 2D 对应点。
局限：对高度变化敏感，高地/地面切换依赖掩码精度，平地高地切换时坐标容易跳变。

标定产物：`configs/perspective_calib.json`

### 2.2 方案 B: 射线求交 (mode = "raycast")

原理：将像素反投影为三维射线，与赛场 PLY 三维网格模型求交，获取精确的 3D 交点后转换为赛场坐标。

优点：
- 天然支持高地/地面判断（通过交点 z 坐标），无需掩码图
- 对非平面区域（坡道、台阶）精度优于 Homography
- 交点包含高度信息 (field_z)，可用于后续业务逻辑

局限：
- 需要 PnP 标定外参（R/T），标定流程稍微复杂
- 依赖 open3d 库
- PLY 模型精度影响最终定位精度

内参输入: `configs/raycast_calib.yaml`(K:内参矩阵 dist_coeffs:畸变系数)
标定产物：`configs/raycast_calib.yaml`（R:旋转矩阵 T：平移矩阵）

**！！注意：视频输入模式当前的内参使用的是港科大的内参，若为“hik”模式的测试，请把`configs/raycast_calib.yaml`对应位置改为我们相机实际的内参**

### 2.3 配置切换

在 `configs/main_config.yaml` 中：

```yaml
coordinate_mapping:
  mode: "homography"          # "homography" | "raycast" | "compare"（对比两种方案误差，调试用）
  raycast_config: "configs/raycast_calib.yaml"
  highland_z_threshold: 0.4   # raycast 模式高地判定阈值 (m)
```

| 模式 | 说明 |
|---|---|
| `homography` | 使用 Homography 透视变换 (默认) |
| `raycast` | 使用射线求交 |
| `compare` | 同时运行两种方案，输出对比日志 (调试用，实际发布仍使用 homography) |

## 3. 启动方法

### 3.1 步骤 1A: Homography 透视变换标定

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

### 3.1 步骤 1B: 射线求交 PnP 标定

使用射线求交方案时，需要标定相机的 world-to-camera 外参 (R, T)。

```bash
# 方式 0: 直接运行（推荐）
# 默认自动读取内置路径（当前读取的为国赛相关mesh地图与关键点）：
# - ply: RMUC2025_National.PLY
# - keypoints: field/keypoint_6.txt（National）
# - 图像: 优先按脚本默认版本
ros2 run hnurm_radar raycast_calibrator

# 方式 1: 使用指定标定图像
ros2 run hnurm_radar raycast_calibrator -- --image path/to/camera_image.jpg

# 方式 2: 从视频取第一帧
ros2 run hnurm_radar raycast_calibrator -- --video path/to/video.mp4


# 若改为区域赛请切换raycast_calibrator对应的路径
ros2 run hnurm_radar raycast_calibrator -- --keypoints field/keypoint_6_region.txt
```

交互流程（以当前 GUI 为准）：
1. 左键依次标注 6 个点；右键撤销上一个点；`R` 重置全部；
2. 可用方向键微调“最新一个点”（仅在点击 `START CALC` 之前有效）；
3. 点满后点击 `START CALC` 开始计算；
4. 结果在面板显示 `Reprojection Error`；
5. 点击 `SAVE RESULT` 保存到 `configs/raycast_calib.yaml`；
6. `SHOW EXAMPLE` 按钮可弹出示例图 **注意：严格按照示例图的顺序与位置**。

说明：
- 当前标定阶段不按红/蓝方做 3D 点镜像；
- National 默认使用 `field/keypoint_6.txt`，Regional 使用 `field/keypoint_6_region.txt`；
- 标定点具体坐标以对应 keypoint 文件内容为准。

标定质量评估（经验值）：
- **< 3.0 px**：良好
- **3.0 ~ 8.0 px**：可用，建议复核
- **> 8.0 px**：建议重标定或检查点集/内参与分辨率一致性

### 3.2 步骤 2：启动系统

分别在不同终端中启动：

```bash
# 终端 1：相机检测 + 坐标映射定位
source install/setup.bash
ros2 run hnurm_radar camera_detector

# 终端 2：EKF 坐标滤波
source install/setup.bash
ros2 run ekf ekf_node

# 终端 3：裁判系统通信
source install/setup.bash
ros2 run hnurm_radar judge_messager
```

### 3.3 常用调试命令
```bash
# 相机侧原始坐标输出
ros2 topic echo /location

# EKF 后坐标输出
ros2 topic echo /ekf_location_filtered

# 话题频率
ros2 topic hz /location
ros2 topic hz /ekf_location_filtered
```

---

## 4. 参考与差异说明

本方案参考港科大雷达站思路，当前实现包含两种坐标映射后端：

- **Homography 方案**：2D 单应性矩阵（像素点 → 赛场平面坐标），标定简单但对高度变化敏感
- **Raycast 方案**：3D 射线-网格求交（参考港科大 `PixelToWorld`），精度高但需 PnP 标定
- 框架为 **SORT 风格多目标跟踪**：
  - YOLO 仅用于单帧检测；
  - `bbox_kalman` 用于框平滑与短时预测；
  - `hungarian_tracker` 负责检测-轨迹匹配与 ID 投票；
  - 长时丢失可进入 `GUESSING`（当前分支默认不发布 GUESSING 坐标到小地图，效果待优化）。

---

## 5. 处理链路（按实际代码）

1. `camera_detector.py` 获取图像并执行 YOLO 三阶段检测。  
2. 检测结果经 NMS 和简单去重。  
3. 结果送入 `hungarian_tracker.update(detections, dt)`：  
   - 轨迹预测（基于 `bbox_kalman`）  
   - 匈牙利匹配  
   - 轨迹状态迁移（TRACKING/LOST/GUESSING）  
   - 标签投票与稳定 ID 输出  
4. 对滤波后的 bbox，取底边中心点做 `pixel_to_field()`。  
5. `pixel_to_field()` 根据 `coordinate_mapping.mode` 路由：
   - `homography`：调用 `HomographyMapper`（多层 H 矩阵 + 掩码）
   - `raycast`：调用 `RaycastMapper`（Open3D 射线求交 + Solidworks 坐标变换）
   - `compare`：同时调用两者，输出 DEBUG 对比日志
6. 发布 `/location`（赛场坐标，raycast 模式 z 值携带物理高度）。  
7. `ekf_node` 订阅 `/location` 并输出 `/ekf_location_filtered`。  
8. 小地图显示发布坐标（GUESSING 发布策略受当前代码开关控制）。

---

## 6. 关键模块职责

- `camera_scheme/camera_detector.py`：推理、去重、映射路由、发布、可视化入口。  
- `camera_scheme/hungarian_tracker.py`：轨迹管理、匈牙利匹配、投票确认 ID。  
- `filters/bbox_kalman.py`：像素框卡尔曼滤波（短时预测与平滑）。  
- `camera_scheme/guess_pts.py`：GUESSING 状态外推（当前默认不作为主发布源）。  
- `mapping/base.py`：坐标映射器抽象基类 (`CoordinateMapper`)。
- `mapping/homography_mapper.py`：Homography 透视变换封装。
- `mapping/raycast_mapper.py`：射线-网格求交映射器。
- `mapping/solidworks_to_field.py`：Solidworks PLY 坐标→裁判系统坐标变换。
- `shared/type.py`：统一数据结构与状态枚举。  
- `ekf/ekf_node.py`：坐标级 EKF 平滑与发布。

---

## 7. 配置文件说明

### 7.1 detector_config.yaml

配置文件路径：`configs/detector_config.yaml`

#### `path`（模型与配置路径）
- `stage_one_path/stage_two_path/stage_three_path`：三阶段 YOLO 权重路径。  
- `tracker_path`：YOLO 内部 tracker 配置（当前主轨迹维护仍以本项目 `hungarian_tracker` 为主）。

#### `params`（检测基础参数）
- `labels`：类别列表。  
- `stage_one_conf/stage_two_conf/stage_three_conf`：各阶段置信度阈值。  
- `life_time`：目标生命周期（上层管理相关参数）。

#### `track`（匈牙利匹配核心参数）
- `iou_thr`：IoU 匹配门控阈值。  
- `dist_thr`：中心距离门控阈值（像素）。  
- `lost_thr`：漏检帧数阈值,进入短时预测LOST阶段的阈值（短时漏检仍维持稳定状态）。  
- `guess_thr`：进入更深丢失GUESSING阶段的阈值。  
- `max_miss`：轨迹最大连续漏检帧，超过后删除。  
- `publish_predict_when_no_det`：无检测时是否允许发布预测轨迹（受当前代码分支控制）。

> 时间换算说明：  
> 若按帧阈值配置，实际时长 = `阈值帧数 / 实际推理FPS`。  
> 例如 `max_miss=54`：  
> - 30 FPS ≈ 1.8s  
> - 40 FPS ≈ 1.35s  

#### 录制参数
- `is_record`：是否保存推理视频。  
- `record_fps`：录制输出帧率。

#### 已弃用配置
- `filter`（物理卡尔曼滤波）原先是对原始透视坐标进行滤波，与ekf重复滤波逻辑重复，目前在纯视觉主链路中已弃用

### 7.2 raycast_calib.yaml

配置文件路径：`configs/raycast_calib.yaml`

由 `ros2 run hnurm_radar raycast_calibrator` 写入/更新，包含：

| 字段 | 说明 |
|---|---|
| `K` | 相机内参矩阵 3x3 |
| `R` | world-to-camera 旋转矩阵 3x3 (PnP 输出) |
| `T` | world-to-camera 平移向量 3x1 |
| `dist_coeffs` | 畸变系数 [k1, k2, p1, p2, k3] |
| `mesh_path` | PLY 网格路径 |
| 文件末尾注释 | `# PnP 投影残差: xx.xxxx px`（写在最后一行） |

### 7.3 main_config.yaml (坐标映射相关)

```yaml
coordinate_mapping:
  mode: "homography"                        # 映射后端
  raycast_config: "configs/raycast_calib.yaml"
  highland_z_threshold: 0.4                 # 高地判定 z 阈值 (m)
```

---

## 8. 调参说明与量化方法

### 8.1 优先级建议
1.  **匹配稳定性**：`track.iou_thr`（重叠度）、`track.dist_thr`（中心点像素位移门控）。
2.  **生命周期**：`lost_thr`（平滑丢失）、`guess_thr`（外推阈值）、`max_miss`（彻底删除）。
3.  **滤波表现**：修改 `bbox_kalman.py` 中的过程噪声 $Q$（响应速度）与观测噪声 $R$（平滑度）。

### 8.2 Kalman 量化调优 (SOP)
为了避免盲目修改，推荐使用**残差分析法 (Innovation Analysis)**：
-   **残差定义**：$y = z - H\hat{x}$（即 YOLO 原始框坐标与 Kalman 预测框坐标之差）。
-   **响应速度不足（滞后）**：若机器人在转弯/加速时，残差 $y$ 持续且显著偏向一侧，说明系统过于依赖匀速模型。应**增大过程噪声 $Q$**（如 `_q_weight_pos`）。
-   **稳定性不足（抖动）**：若机器人静止时，残差 $y$ 剧烈跳变。说明系统过于依赖 YOLO 的瞬时输出。应**增大观测噪声 $R$**（如 `_r_weight_pos`）。
-   **维数均衡原则**：在透视变换方案中，**Y 轴（纵向）对坐标精度极度敏感**。建议给 Y 轴分配比 X 轴高 1.5~2 倍的 $R$ 权重，以换取坐标的垂直稳定性。

### 8.3 关键机制说明
-   **预测阻尼 (Damping)**：`F_k` 矩阵中的阻尼系数（如 0.95~0.99）能防止轨迹在 `LOST` 状态下带速无限飞出，解决"幽灵框"劫持他人 ID 的问题。
-   **尺度自适应 (Scale Factor)**：系统会根据 $w \times h$ 自动计算 $sf$ 因子，针对远距离小目标自动切换为"重滤波"模式。
-   **AKF 自适应噪声**：update 侧基于归一化像素残差动态调节 P 和 R；predict 侧在高速运动时放大速度过程噪声。
-   **速度直注入**：绕过 KF 隐状态结构性限制，将位置残差直接注入速度状态，加速速度学习（带死区和钳位保护）。

---

## 9. 当前已知问题

1. 转弯与长时间遮挡场景下，坐标连续性不佳。  
2. GUESSING 长时外推稳定性不足，坐标会飞出，当前默认不作为主发布结果。  
3. 多个机器人检测框交错会发生匈牙利错误匹配轨迹。  
4. 右上角高地发布坐标出现后退现象，可能由于检测框抖动导致机器人坐标透视落在平地与高地交错处，导致坐标抖动。
5. 射线求交方案的 PnP 标定精度依赖标定点选取质量，实际比赛中标定点可能不易辨识。

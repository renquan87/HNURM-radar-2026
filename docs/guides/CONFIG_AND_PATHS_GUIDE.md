# 参数与路径完全指南

> 本文档覆盖项目中**所有**配置文件、参数、路径的定义位置、流向、作用，以及你需要在什么场景下修改它们。

---

## 一、整体架构：参数

```
configs/main_config.yaml          ─┬─→ paths.py ─→ 所有 Python 节点
                                   │              （lidar_node, detector_node, radar_node,
                                   │               display_panel, judge_messager, perspective_calibrator）
                                   │
                                   └─→ registration.launch.py ─→ registration_node (C++)

configs/detector_config.yaml      ─→ detector_node.py（YOLO 模型路径、检测参数）
configs/converter_config.yaml     ─→ radar.py（相机内外参、点云聚类、场地范围）
configs/bin_cam_config.yaml       ─→ HKCam 海康相机驱动（相机硬件参数）
configs/bytetrack.yaml            ─→ detector_node.py → ByteTrack 跟踪器
configs/perspective_calib.json    ─→ camera_detector.py（透视变换矩阵，由标定工具生成）
configs/icp.rviz                  ─→ registration.launch.py → rviz2（点云可视化布局）

src/livox_ros_driver2/config/
  HAP_config.json                 ─→ rviz_HAP_launch.py → livox 驱动（雷达网络配置）

src/registration/params/
  default.yaml                    ─→ registration.launch.py → registration_node（ICP/Quatro 算法参数）
```

注意：项目根目录 `configs/HAP_config.json` 是一份**副本/备份**，livox 驱动实际读取的是 `src/livox_ros_driver2/config/HAP_config.json`。修改时两边都要改，或者只改 livox 那份。

---

## 二、场景切换机制

### 原理

`main_config.yaml` 中有一个 `global.scene` 字段，决定当前使用哪套资源文件。

```yaml
global:
  scene: "lab"          # "competition" = 赛场 | "lab" = 实验室
```

`scenes` 段定义了每个场景对应的文件路径：

```yaml
scenes:
  competition:
    std_map: "map/std_map.png"
    pfa_map: "map/pfa_map_2025.jpg"
    pcd_file: "data/pcds.pcd"
    downsampled_pcd: "data/pcds_downsampled.pcd"
    field_width: 28.0
    field_height: 15.0
  lab:
    std_map: "map/lab_std_map.png"
    pfa_map: "map/lab_map.png"
    pcd_file: "data/lab_pcds.pcd"
    downsampled_pcd: "data/lab_pcds_downsampled.pcd"
    field_width: 10.0
    field_height: 6.0
```

### 流向

1. `paths.py` 启动时读取 `main_config.yaml`，根据 `scene` 值选择对应的 `scenes.xxx` 配置
2. 生成 `STD_MAP_PATH`、`PFA_MAP_2025_PATH`、`PCD_FILE_PATH`、`PCD_DOWNSAMPLED_PATH`、`FIELD_WIDTH`、`FIELD_HEIGHT`
3. 所有 Python 节点通过 `from ..shared.paths import XXX` 获取这些路径
4. `registration.launch.py` 也独立读取 `main_config.yaml`，将 PCD 路径作为 ROS 参数传给 C++ 节点

### 你需要做什么

- 切换场景：只改 `global.scene` 的值
- 新增场景：在 `scenes` 下加一个新的配置块，准备好对应的文件
- 去赛场：`scene: "competition"`
- 回实验室：`scene: "lab"`

---

## 三、逐文件详解

### 3.1 configs/main_config.yaml — 主配置文件

**读取者**：`lidar_node.py`、`radar.py`（通过 YAML 直接读取）、`judge_messager.py`、`perspective_calibrator.py`、`paths.py`（场景切换）、`registration.launch.py`（场景切换）

| 配置段 | 参数 | 类型 | 默认值 | 作用 | 何时修改 |
|--------|------|------|--------|------|---------|
| `global.my_color` | 字符串 | `"Blue"` | 我方颜色，决定识别哪方机器人 | 换边时改 |
| `global.is_debug` | 布尔 | `True` | 调试模式开关 | 比赛时设 False |
| `global.scene` | 字符串 | `"lab"` | 当前场景 | 切换实验室/赛场 |
| `camera.mode` | 字符串 | `"test"` | 相机输入模式：`test`=静态图片, `video`=USB摄像头, `hik`=海康工业相机 | 标定工具使用 |
| `camera.video_source` | 整数/字符串 | `0` | video 模式的输入源 | 换摄像头时改 |
| `camera.test_image` | 字符串 | `"test_resources/test1.png"` | test 模式的图片路径 | 换测试图片时改 |
| `car.life_span` | 整数 | `20` | 机器人生命周期（帧），超过未刷新则认为不可信 | 一般不改 |
| `lidar.height_threshold` | 浮点 | `5.5` | 地面点高度阈值（当前代码中未启用） | 一般不改 |
| `lidar.min_distance` | 浮点 | `1` | 近距离滤除阈值(m) | 实验室空间小可以调小 |
| `lidar.max_distance` | 浮点 | `40` | 远距离滤除阈值(m) | 实验室可以调小到 10-15 |
| `lidar.lidar_topic_name` | 字符串 | `"/livox/lidar"` | 点云输入话题名 | 一般不改 |
| `lidar.background_map_path` | 字符串 | `"data/background.pcd"` | 背景地图 PCD 路径，`lidar_node.py` 启动时会通过 `resolve_path()` 解析后加载 | 更换场地背景地图时改 |
| `lidar.background_threshold` | 浮点 | `0.2` | 背景减除距离阈值(m)，点到背景最近距离不超过该值时视为背景 | 误检多/漏检多时微调 |
| `lidar.voxel_size` | 浮点 | `0.1` | 背景地图加载后的体素降采样大小(m) | 背景图过密或保留细节不足时调整 |
| `lidar.publish_projected` | 布尔 | `true` | 是否发布背景减除后的前景点云到 `/target_pointcloud` | 一般保持 true |
| `communication.port` | 字符串 | `"/dev/ttyUSB0"` | 裁判系统串口设备 | 换串口时改 |
| `communication.bps` | 整数 | `115200` | 串口波特率 | 一般不改 |
| `communication.timex` | 浮点 | `0.01` | 串口超时时间(s) | 一般不改 |
| `area.*` | 嵌套 | — | 英雄区域判定的多边形坐标 | 赛场规则变化时改 |

---

### 3.2 configs/detector_config.yaml — 检测器配置

**读取者**：`detector_node.py`（通过 `paths.py` 的 `DETECTOR_CONFIG_PATH` 定位文件，然后 YAML 读取）

| 配置段 | 参数 | 类型 | 默认值 | 作用 | 何时修改 |
|--------|------|------|--------|------|---------|
| `path.stage_one_path` | 字符串 | `"weights/stage_one.pt"` | 第一阶段 YOLO 模型（装甲板检测） | 换模型时改 |
| `path.stage_two_path` | 字符串 | `"weights/stage_two.pt"` | 第二阶段模型（ROI 裁剪/分类） | 换模型时改 |
| `path.stage_three_path` | 字符串 | `"weights/stage_three.pt"` | 第三阶段模型（数字识别） | 换模型时改 |
| `path.tracker_path` | 字符串 | `"configs/bytetrack.yaml"` | ByteTrack 跟踪器配置路径 | 一般不改 |
| `params.labels` | 列表 | `["B1"..."R7"]` | 标签列表（12 个机器人编号） | 一般不改 |
| `params.stage_one_conf` | 浮点 | `0.1` | 第一阶段置信度阈值 | 漏检多→调低，误检多→调高 |
| `params.stage_two_conf` | 浮点 | `0.6` | 第二阶段置信度阈值 | 同上 |
| `params.stage_three_conf` | 浮点 | `0.5` | 第三阶段置信度阈值 | 同上 |
| `params.life_time` | 整数 | `90` | 检测目标生命周期（帧） | 一般不改 |
| `filter.process_noise` | 浮点 | `0.01` | 坐标卡尔曼滤波过程噪声 | 越小越平滑但响应慢 |
| `filter.measurement_noise` | 浮点 | `0.1` | 观测噪声 | 越大越信赖预测值 |
| `filter.jump_threshold` | 浮点 | `3.0` | 单帧跳变阈值(m) | 超过则拒绝更新 |
| `filter.max_inactive_time` | 浮点 | `3.0` | 滤波器超时清理时间(s) | 一般不改 |
| `is_record` | 布尔 | `true` | 是否录制检测视频 | 调试时 true，比赛可关 |
| `record_fps` | 整数 | `60` | 录制帧率 | 一般不改 |

路径解析方式：`detector_node.py` 中调用 `resolve_path()` 将相对路径转为绝对路径（相对于 `PROJECT_ROOT`）。

---

### 3.3 configs/converter_config.yaml — 点云-图像转换配置

**读取者**：`radar.py`（通过 `paths.py` 的 `CONVERTER_CONFIG_PATH` 定位）

| 配置段 | 参数 | 作用 | 何时修改 |
|--------|------|------|---------|
| `params.max_depth` | 点云最大深度(m) | 一般不改 |
| `params.width` | 图像宽度(px) | 换相机/分辨率时改 |
| `params.height` | 图像高度(px) | 换相机/分辨率时改 |
| `calib.extrinsic.R` | 3×3 旋转矩阵 | **雷达-相机外参**，重新标定后必须更新 |
| `calib.extrinsic.T` | 3×1 平移向量 | **雷达-相机外参**，重新标定后必须更新 |
| `calib.intrinsic.cx/cy/fx/fy` | 相机内参 | 重新标定内参后更新 |
| `calib.distortion.data` | 畸变系数(5个) | 重新标定内参后更新 |
| `cluster.eps` | DBSCAN 邻域半径 | 聚类效果不好时调 |
| `cluster.min_points` | DBSCAN 最小点数 | 聚类效果不好时调 |
| `filter.nb_neighbors` | 统计滤波邻居数 | 一般不改 |
| `filter.std_ratio` | 统计滤波标准差倍数 | 一般不改 |
| `filter.voxel_size` | 体素降采样大小(m) | 一般不改 |
| `field.Red/Blue/test` | 场地四角坐标 | 换场地时改 |

**重要**：`R` 和 `T` 是整个系统最关键的标定参数。如果相机和雷达的相对位置变了（重新安装），必须重新标定并更新这里。标定工具是 `direct_visual_lidar_calibration_ws` 项目。

---

### 3.4 configs/bin_cam_config.yaml — 海康工业相机硬件参数

**读取者**：`HKCam` 类（`src/hnurm_radar/hnurm_radar/Camera/HKCam.py`）

| 参数 | 作用 | 何时修改 |
|------|------|---------|
| `id.right` | 相机序列号 | 换相机时改 |
| `id.new_cam` | 新相机序列号 | 换相机时改 |
| `param.Width/Height` | 分辨率 | 一般不改（4024×3036） |
| `param.ExposureTime` | 曝光时间(μs) | 光照变化时调（室内 25000，室外可能需要调低） |
| `param.Gamma` | 伽马校正 | 图像偏暗/偏亮时调 |
| `param.Gain` | 增益 | 图像偏暗时调高 |
| `param.BalanceRatioR/G/B` | 白平衡 | 色偏时调 |
| `param.show_width/show_height` | 显示窗口大小 | 一般不改 |
| `param.pyr_times` | 图像金字塔层数 | 一般不改 |

---

### 3.5 configs/bytetrack.yaml — 目标跟踪器参数

**读取者**：`detector_node.py` → ultralytics ByteTrack

| 参数 | 默认值 | 作用 | 何时修改 |
|------|--------|------|---------|
| `track_high_thresh` | `0.5` | 高置信度关联阈值 | 跟踪丢失频繁→调低 |
| `track_low_thresh` | `0.1` | 低置信度关联阈值 | 一般不改 |
| `new_track_thresh` | `0.6` | 新轨迹初始化阈值 | 一般不改 |
| `track_buffer` | `30` | 轨迹保持缓冲(帧) | 目标短暂遮挡后丢失→调大 |
| `match_thresh` | `0.8` | 匹配阈值 | 一般不改 |

---

### 3.6 configs/perspective_calib.json — 透视变换标定结果

**生成者**：`perspective_calibrator.py`（标定工具 GUI）
**读取者**：`camera_detector.py`（camera_scheme 方案）

这个文件是**自动生成**的，不要手动编辑。内容包括：
- `H_ground`：地面层 3×3 单应性矩阵（像素→赛场米坐标）
- `H_highland`：高地层 3×3 单应性矩阵
- `pixel_points_ground/highland`：标定时选的像素点
- `field_points_ground/highland`：对应的赛场坐标点

**何时重做**：换场地、换相机位置、换相机镜头后必须重新标定。

---

### 3.7 configs/icp.rviz — RViz 可视化配置

**读取者**：`registration.launch.py` → rviz2

这是 RViz2 的界面布局文件，定义了显示哪些话题、用什么颜色等。可以在 RViz 中手动调整后 `Ctrl+S` 保存。

---

### 3.8 src/registration/params/default.yaml — 点云配准算法参数

**读取者**：`registration.launch.py` → `registration_node`（C++ 节点）

| 参数 | 默认值 | 作用 | 何时修改 |
|------|--------|------|---------|
| `pointcloud_sub_topic` | `"/lidar_pcds"` | 实时点云话题 | 一般不改 |
| `pcd_file` | `"pcds.pcd"` | 点云地图文件名 | **被 launch 文件覆盖**，不需要手动改 |
| `downsampled_pcd_file` | `"pcds_downsampled.pcd"` | 降采样地图文件名 | **被 launch 文件覆盖** |
| `generate_downsampled_pcd` | `false` | 是否生成降采样地图 | 首次录制点云后设 true 运行一次 |
| `num_threads` | `7` | 并行线程数 | 设为 CPU 核心数 |
| `source_voxel_size` | `0.25` | 实时点云降采样大小(m) | 配准效果不好时调 |
| `map_voxel_size` | `0.25` | 地图降采样大小(m) | 配准效果不好时调 |
| `use_fixed` | `true` | 是否使用固定地图配准 | 一般 true |
| `use_quatro` | `true` | 是否使用 Quatro 鲁棒配准 | 一般 true |
| `m_rotation_max_iter` | `200` | Quatro 旋转最大迭代 | 一般不改 |
| `m_num_max_corres` | `100` | 最大匹配点对数 | 一般不改 |
| `m_normal_radius` | `0.5` | 法线计算半径(m) | 一般不改 |
| `m_fpfh_radius` | `1.2` | FPFH 特征半径(m) | 一般不改 |
| `m_distance_threshold` | `30.0` | 特征匹配距离阈值 | 一般不改 |
| `m_noise_bound` | `0.2` | 噪声界限 | 配准不稳定时调 |
| `m_rotation_gnc_factor` | `1.39` | GNC 增长因子 | 一般不改 |
| `m_rotation_cost_thr` | `0.0001` | 旋转收敛阈值 | 一般不改 |
| `m_estimate_scale` | `false` | 是否估计尺度 | 一般 false |
| `m_use_optimized_matching` | `true` | 优化匹配 | 一般 true |

**重要**：`pcd_file` 和 `downsampled_pcd_file` 在 `default.yaml` 中只是默认值。`registration.launch.py` 会从 `main_config.yaml` 的场景配置中读取实际路径，并通过 ROS 参数覆盖这两个值。所以你不需要改 `default.yaml` 中的文件名。

---

### 3.9 src/livox_ros_driver2/config/HAP_config.json — 雷达网络配置

**读取者**：`rviz_HAP_launch.py` → livox 驱动节点

| 参数 | 作用 | 何时修改 |
|------|------|---------|
| `HAP.host_net_info.cmd_data_ip` | 主机 IP（接收端） | 换电脑/网卡时改 |
| `lidar_configs[0].ip` | 雷达 IP | 换雷达时改 |
| `lidar_configs[0].pcl_data_type` | 点云数据类型 | 一般不改 |
| `lidar_configs[0].pattern_mode` | 扫描模式 | 一般不改 |
| `lidar_configs[0].blind_spot_set` | 盲区设置(mm) | 一般不改 |
| `lidar_configs[0].extrinsic_parameter` | 雷达外参(roll/pitch/yaw/x/y/z) | 多雷达时用，单雷达全 0 |

**注意**：livox 驱动读取的是 `src/livox_ros_driver2/config/HAP_config.json`，不是项目根目录的 `configs/HAP_config.json`。

---

## 四、paths.py — 路径中枢

`src/hnurm_radar/hnurm_radar/shared/paths.py` 是所有 Python 节点的路径来源。

### 自动推导项目根目录

```python
PROJECT_ROOT = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..")
)
# __file__ = src/hnurm_radar/hnurm_radar/shared/paths.py
# 向上 4 层 = 项目根目录
```

### 导出的路径常量

| 常量名 | 值（相对于 PROJECT_ROOT） | 使用者 |
|--------|--------------------------|--------|
| `PROJECT_ROOT` | `/data/projects/radar/hnurm_radar` | 所有 |
| `CONFIGS_DIR` | `configs/` | 所有 |
| `WEIGHTS_DIR` | `weights/` | detector_node |
| `MAP_DIR` | `map/` | display_panel, make_mask |
| `DATA_DIR` | `data/` | — |
| `RECORD_DIR` | `record/` | detector_node |
| `MAIN_CONFIG_PATH` | `configs/main_config.yaml` | lidar_node, radar, judge_messager, calibrator |
| `DETECTOR_CONFIG_PATH` | `configs/detector_config.yaml` | detector_node |
| `CONVERTER_CONFIG_PATH` | `configs/converter_config.yaml` | radar |
| `PERSPECTIVE_CALIB_PATH` | `configs/perspective_calib.json` | camera_detector, calibrator |
| `BYTETRACK_CONFIG_PATH` | `configs/bytetrack.yaml` | detector_node |
| `STD_MAP_PATH` | **场景决定** | display_panel |
| `PFA_MAP_2025_PATH` | **场景决定** | perspective_calibrator |
| `PCD_FILE_PATH` | **场景决定** | — |
| `PCD_DOWNSAMPLED_PATH` | **场景决定** | — |
| `FIELD_WIDTH` | **场景决定**（28.0 或 10.0） | perspective_calibrator |
| `FIELD_HEIGHT` | **场景决定**（15.0 或 6.0） | perspective_calibrator |
| `STAGE_ONE/TWO/THREE_PATH` | `weights/stage_*.pt` | detector_node |

### 工具函数

- `resolve_path(path_str)` — 相对路径→绝对路径（相对于 PROJECT_ROOT）；当前被 `detector_node.py`、`lidar_node.py`、`scripts/lidar_collect_background.py` 等复用，用于模型文件、背景地图等路径解析
- `get_scene_name()` — 返回当前场景名

---

## 五、EKF 节点参数（硬编码）

`src/ekf/ekf/ekf_node.py` 的参数**没有**通过配置文件设置，全部硬编码在代码中：

| 参数 | 值 | 位置 | 作用 |
|------|-----|------|------|
| `pval` | `0.001` | `EKFNode.__init__` | EKF 初始协方差 |
| `qval` | `1e-4` | 同上 | 过程噪声 |
| `rval` | `0.0005` | 同上 | 观测噪声 |
| `interval` | `0.05` | 同上 | 滤波步进间隔(s) |
| 定时器周期 | `0.05` | `create_timer(0.05, ...)` | 50ms 触发一次 |
| QoS depth | `10` | QoS 配置 | 消息队列深度 |

如果要调 EKF 参数，需要直接改 `src/ekf/ekf/ekf_node.py` 中的代码。

---

## 六、数据文件清单

### map/ 目录 — 地图图像

| 文件 | 用途 | 场景 |
|------|------|------|
| `std_map.png` | display_panel 底图 | competition |
| `lab_std_map.png` | display_panel 底图 | lab |
| `pfa_map_2025.jpg` | 透视标定用地图 | competition |
| `lab_map.png` | 透视标定用地图 | lab |
| `pfa_map_mask_2025.jpg` | 地图掩码 | 通用 |
| 其他 `pfa_map_*.jpg` | 历史版本/颜色分离版 | 备用 |

### data/ 目录 — 点云数据

| 文件 | 用途 | 场景 |
|------|------|------|
| `pcds.pcd` | 赛场点云地图 | competition |
| `pcds_downsampled.pcd` | 赛场降采样点云 | competition |
| `lab_pcds.pcd` | 实验室点云地图 | lab |
| `lab_pcds_downsampled.pcd` | 实验室降采样点云 | lab |
| `background.pcd` | 地面方案二背景地图，供 `lidar_node.py` 做背景减除 | 通常按当前场地采集 |

### weights/ 目录 — YOLO 模型

| 文件 | 用途 |
|------|------|
| `stage_one.pt` | 第一阶段：装甲板检测 |
| `stage_two.pt` | 第二阶段：ROI 分类 |
| `stage_three.pt` | 第三阶段：数字识别 |

---

## 七、启动流程中的参数传递

### bringup.sh 启动 3 个 launch

```
bringup.sh
  ├─ ros2 launch livox_ros_driver2 rviz_HAP_launch.py
  │    └─ 读取 src/livox_ros_driver2/config/HAP_config.json
  │    └─ 启动 livox 驱动 + rviz
  │
  ├─ ros2 launch hnurm_bringup hnurm_radar_launch.py
  │    └─ 启动 6 个 Python 节点（无额外参数传递，各节点自己读配置文件）
  │    └─ lidar_node    ← main_config.yaml (lidar段)
  │    └─ detector_node ← detector_config.yaml + bytetrack.yaml
  │    └─ radar_node    ← main_config.yaml + converter_config.yaml
  │    └─ display_panel ← paths.py → STD_MAP_PATH (场景决定)
  │    └─ judge_messager← main_config.yaml (communication段)
  │    └─ ekf_node      ← 硬编码参数
  │
  └─ ros2 launch registration registration.launch.py
       └─ 读取 main_config.yaml → 场景 → PCD 路径
       └─ 读取 default.yaml → 算法参数
       └─ PCD 路径通过 ROS 参数覆盖 default.yaml 中的默认值
       └─ 启动 registration_node (C++) + rviz2
```

---

## 八、常见操作速查

| 我想要... | 改哪里 |
|-----------|--------|
| 切换实验室/赛场 | `main_config.yaml` → `global.scene` |
| 换边（红/蓝） | `main_config.yaml` → `global.my_color` |
| 调检测灵敏度 | `detector_config.yaml` → `params.stage_*_conf` |
| 调坐标平滑度 | `detector_config.yaml` → `filter.*` |
| 调 EKF 滤波 | `src/ekf/ekf/ekf_node.py` → 硬编码参数 |
| 更新相机外参 | `converter_config.yaml` → `calib.extrinsic.R/T` |
| 更新相机内参 | `converter_config.yaml` → `calib.intrinsic.*` |
| 调相机曝光 | `bin_cam_config.yaml` → `param.ExposureTime` |
| 调雷达滤波距离 | `main_config.yaml` → `lidar.min_distance/max_distance` |
| 切换背景地图 | `main_config.yaml` → `lidar.background_map_path` |
| 调背景减除阈值 | `main_config.yaml` → `lidar.background_threshold` |
| 调背景地图降采样粒度 | `main_config.yaml` → `lidar.voxel_size` |
| 重新采集背景地图 | 运行 `python3 scripts/lidar_collect_background.py --topic /livox/lidar --output data/background.pcd --max-frames 120 --voxel-size 0.1` |
| 换雷达 IP | `src/livox_ros_driver2/config/HAP_config.json` → `lidar_configs[0].ip` |
| 换串口 | `main_config.yaml` → `communication.port` |
| 换 YOLO 模型 | `detector_config.yaml` → `path.stage_*_path`，并把 .pt 文件放到 `weights/` |
| 重做透视标定 | 运行 `ros2 run hnurm_radar perspective_calibrator`，自动写入 `perspective_calib.json` |
| 调点云配准参数 | `src/registration/params/default.yaml` |
| 调跟踪器参数 | `configs/bytetrack.yaml` |
| 新增一个场景 | `main_config.yaml` → `scenes` 下加一个块，准备对应的地图和点云文件 |
| 查看当前场景 | `main_config.yaml` → `global.scene` |
| 查看所有路径 | 看 `src/hnurm_radar/hnurm_radar/shared/paths.py` 的导出常量 |

---

## 九、参数优先级

当同一个参数在多处出现时，优先级如下：

1. ROS launch 参数覆盖（最高）— 如 `registration.launch.py` 中传入的 `pcd_file`
2. 配置文件中的值 — 如 `default.yaml` 中的 `pcd_file`
3. 代码中的默认值（最低）— 如 `declare_parameter("pcd_file", "/home/rm/...")` 中的默认路径

实际上目前只有 `pcd_file` 和 `downsampled_pcd_file` 存在这种覆盖关系。其他参数都是单一来源。

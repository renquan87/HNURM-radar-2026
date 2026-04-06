# 🏗️ 系统架构总览

本文档描述 HNURM Radar 2026 雷达站系统的整体架构、节点关系和数据流。

---

## 目录

- [系统架构图](#系统架构图)
- [三种方案架构](#三种方案架构)
- [目录结构](#目录结构)
- [关键模块说明](#关键模块说明)
- [配置系统](#配置系统)

---

## 系统架构图

### 总体架构（方案二）

```mermaid
graph TD
    subgraph 硬件层
        HAP[Livox HAP 激光雷达]
        CAM[海康工业相机]
        UART[串口 - 裁判系统]
    end

    subgraph 驱动层
        LIVOX[livox_ros_driver2<br/>激光雷达驱动]
        HKCAM[hnurm_camera_node<br/>相机驱动]
    end

    subgraph 感知层
        LIDAR[lidar_node<br/>点云预处理<br/>距离滤波+累积+背景减除]
        DET[detector_node<br/>YOLO 三阶段检测<br/>目标+装甲板+灰色]
        REG[registration_node<br/>ICP 点云配准]
    end

    subgraph 融合层
        RADAR[radar_node<br/>点云-图像融合定位]
    end

    subgraph 后处理层
        EKF[ekf_node<br/>扩展卡尔曼滤波<br/>7路独立滤波器]
    end

    subgraph 输出层
        JUDGE[judge_messager<br/>裁判系统通信]
        DISP[display_panel<br/>小地图可视化]
    end

    HAP -->|原始点云| LIVOX
    CAM -->|图像| HKCAM

    LIVOX -->|/livox/lidar| LIDAR
    HKCAM -->|/image| DET
    HKCAM -->|/camera_info| RADAR

    LIDAR -->|/lidar_pcds| REG
    LIDAR -->|/target_pointcloud| RADAR
    DET -->|/detect_result| RADAR
    REG -->|TF map→livox| RADAR

    RADAR -->|/location| EKF
    EKF -->|/ekf_location_filtered| JUDGE
    EKF -->|/ekf_location_filtered| DISP
    JUDGE -->|串口| UART
```

### 方案一（纯相机）

```mermaid
graph LR
    CAM[相机<br/>海康/USB] --> CD[camera_detector<br/>YOLO检测+透视变换]
    CD -->|/location| EKF[ekf_node]
    EKF -->|/ekf_location_filtered| JM[judge_messager]
    EKF -->|/ekf_location_filtered| DP[display_panel]
    JM -->|串口| REF[裁判系统]
```

### 空中方案

```mermaid
graph TD
    HAP[Livox HAP] --> LIVOX[livox_ros_driver2]
    LIVOX -->|/livox/lidar| LN[lidar_node]
    LN -->|/lidar_pcds| REG[registration_node]
    LN -->|/target_pointcloud| AIR[air_target_node<br/>背景减除+DBSCAN+卡尔曼]
    REG -->|TF| AIR
    AIR -->|/location| EKF[ekf_node]
    EKF -->|/ekf_location_filtered| JM[judge_messager]
    EKF -->|/ekf_location_filtered| DP[display_panel]
```

---

## 三种方案架构

### 方案一：纯相机透视变换

```
相机帧 → YOLO 三阶段推理 → 检测框底部中心 → 透视变换(H矩阵)
  → 赛场2D坐标 → EKF → 裁判系统
```

- **优点**：硬件简单，只需一个相机
- **缺点**：精度依赖标定质量，无 3D 信息
- **适用**：备用方案、快速部署

### 方案二：相机+激光雷达融合（主力）

```
相机帧 → YOLO 三阶段推理 → 2D 检测框
                                    ↓
点云 → 背景减除 → 前景点云 → 投影到图像平面 → 框内点云聚类
                                    ↓
                              3D 赛场坐标 → EKF → 裁判系统
```

- **优点**：精确的 3D 定位
- **缺点**：需要激光雷达、ICP 配准
- **适用**：比赛主力方案

### 空中方案：纯激光雷达

```
点云 → 坐标变换(TF) → 背景减除 → ROI 裁剪(高度滤波)
  → DBSCAN 聚类 → 卡尔曼跟踪 → 敌我分类 → EKF → 裁判系统
```

- **优点**：不需要相机，直接 3D 检测
- **缺点**：依赖点云密度，远距离效果差
- **适用**：空中无人机检测

---

## 目录结构

```
src/hnurm_radar/hnurm_radar/
├── camera_scheme/          # 方案一：纯相机
│   ├── camera_detector.py  #   相机检测+透视变换 ROS2 节点
│   └── camera_node.py      #   USB 相机发布节点
│
├── lidar_scheme/           # 方案二：相机+激光雷达
│   ├── lidar_node.py       #   激光雷达预处理节点
│   ├── detector_node.py    #   YOLO 检测节点
│   └── radar.py            #   点云-图像融合节点
│
├── air_scheme/             # 空中方案
│   ├── air_target_node.py  #   空中目标检测节点
│   └── air_kalman_filter.py#   空中目标卡尔曼滤波器
│
├── detection/              # 通用检测模块
│   └── yolo_pipeline.py    #   三阶段 YOLO 推理引擎（ROS 无关）
│
├── communication/          # 裁判系统通信
│   ├── serial_protocol.py  #   串口协议（CRC8/16、帧头帧尾）
│   ├── referee_receiver.py #   裁判系统数据接收（独立进程）
│   └── _deprecated.py      #   已弃用代码（保留参考）
│
├── core/                   # 核心抽象层
│   ├── base_detector.py    #   检测器基类 + Detection/FrameResult
│   ├── base_tracker.py     #   跟踪器基类 + TrackedTarget
│   └── sensor_interface.py #   传感器抽象接口
│
├── shared/                 # 共享工具
│   ├── paths.py            #   统一路径管理
│   ├── judge_messager.py   #   裁判系统通信 ROS2 节点
│   └── display_panel.py    #   小地图可视化节点
│
├── Camera/                 # 海康相机 SDK 封装
│   └── HKCam.py
│
├── Car/                    # 目标状态管理
│   └── Car.py              #   Car + CarList（投票确认）
│
└── camera_locator/         # 标定工具
    ├── perspective_calibrator.py
    └── make_mask.py
```

---

## 关键模块说明

### 三阶段 YOLO 推理管线

```mermaid
graph LR
    Frame[输入帧] --> S1[Stage 1<br/>目标检测<br/>YOLO track]
    S1 --> S2[Stage 2<br/>装甲板分类<br/>颜色+编号]
    S2 --> S3[Stage 3<br/>灰色装甲板<br/>二次分类]
    S3 --> OUT[最终结果<br/>ID+位置+置信度]
```

- **Stage 1**：YOLOv8 目标检测 + ByteTrack 跟踪
- **Stage 2**：对每个检测框裁剪 ROI → 分类模型识别颜色和编号
- **Stage 3**：对灰色/未识别的装甲板 → 专用分类模型

### EKF 滤波器架构

```
滤波器 0~4 → 地面机器人 1-5 (ID 1-5 / 101-105)
滤波器 5   → 哨兵 (ID 7 / 107)
滤波器 6   → 空中机器人 (ID 6 / 106)
```

每个滤波器独立运行，状态向量包含 (x, y, vx, vy)。

---

## 配置系统

```mermaid
graph TD
    MAIN[configs/main_config.yaml<br/>主配置文件] --> GLOBAL[global 段<br/>颜色/调试/场景]
    MAIN --> SCENE[scenes 段<br/>赛场/实验室切换]
    MAIN --> CAM_CFG[camera 段<br/>输入模式]
    MAIN --> LIDAR_CFG[lidar 段<br/>滤波参数]
    MAIN --> COMM[communication 段<br/>串口配置]
    MAIN --> AIR[air_target 段<br/>空中方案参数]

    DET_CFG[configs/detector_config.yaml<br/>检测器配置] --> WEIGHTS[模型路径]
    DET_CFG --> PARAMS[推理参数<br/>置信度/NMS阈值]
    DET_CFG --> GRAY[灰色装甲板阈值]

    PATHS[shared/paths.py<br/>路径管理] --> MAIN
    PATHS --> DET_CFG
    PATHS --> |resolve_path| SCENE
```

所有配置通过 `shared/paths.py` 统一管理路径解析，支持相对路径和场景切换。


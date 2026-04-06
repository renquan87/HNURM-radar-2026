# configs/ — 配置文件说明

本目录包含 HNU RM 雷达站系统的所有运行时配置文件。

## 配置文件清单

| 文件 | 用途 | 消费者 |
|------|------|--------|
| `main_config.yaml` | **全局主配置**：场景切换、颜色、调试开关、相机参数、激光雷达参数、通信串口、空中目标检测、双倍易伤区域 | 几乎所有节点 |
| `detector_config.yaml` | **检测管线配置**：三阶段模型路径/置信度、灰甲板阈值、推理分辨率、卡尔曼滤波、录像 | `YoloPipeline`、`detector_node`、`camera_detector` |
| `converter_config.yaml` | **激光雷达-相机融合配置**：外参R/T、内参fx/fy/cx/cy、畸变系数、聚类参数、深度图分辨率 | `Converter`、`radar.py`（方案二专用） |
| `bytetrack.yaml` | **ByteTrack 跟踪器配置**：关联阈值、跟踪缓冲 | `YoloPipeline`（通过 ultralytics） |
| `perspective_calib.json` | **透视变换标定数据**：地面/高地单应性矩阵H、标定点坐标 | `camera_detector`（方案一专用） |
| `HAP_config.json` | **Livox HAP 激光雷达硬件配置** | livox_ros_driver2 |
| `foxglove_layout.json` | **Foxglove Studio 预配置布局**：3D点云/相机图像/EKF坐标/日志等面板 | Foxglove Studio（手动导入） |

## 配置加载方式

所有 Python 模块通过 `shared/paths.py` 获取配置文件的绝对路径：

```python
from ..shared.paths import MAIN_CONFIG_PATH, DETECTOR_CONFIG_PATH, CONVERTER_CONFIG_PATH
```

配置使用 `ruamel.yaml` 或 `yaml.safe_load` 读取，所有 `.get()` 调用都带有默认值回退，
确保旧配置文件缺少新增字段时不会崩溃。

## 场景切换

修改 `main_config.yaml` 中的 `global.scene` 即可切换场景：

```yaml
global:
  scene: "competition"   # 赛场（28×15m）
  # scene: "lab"         # 实验室测试
```

每个场景在 `scenes:` 段下配置独立的地图路径、点云文件和场地尺寸。

## 子目录

| 目录 | 内容 |
|------|------|
| `archive/` | 弃用/备份的旧配置文件 |
| `calibration/` | BEV标定、地图对齐等标定产物（JSON格式） |

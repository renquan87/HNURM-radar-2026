# HNURM Radar 2026 — 项目锐评

> 锐评时间：2026-04-29
> 评价范围：全部源代码、配置文件、文档、测试

---

## 总体印象

这是一个 RoboMaster 2026 赛季的雷达站软件系统，支持三种检测方案（纯相机、相机+LiDAR 融合、纯 LiDAR 空中目标）。项目体量不小，代码量可观，**核心算法思路基本正确**（YOLO 三阶段推理、DBSCAN 聚类、EKF 平滑、ByteTrack 跟踪等选型合理）。但**代码质量、工程规范、架构设计存在大量问题**，属于典型的"比赛代码能跑就行"风格，离生产级/可维护的工程代码差距很大。

---

## 一、🚨 严重问题（可能导致运行时崩溃或逻辑错误）

### 1. [`coord_solver.py:5`](../../src/hnurm_radar/hnurm_radar/camera_locator/coord_solver.py:5) — 引用不存在的模块

```python
from global_variables import *
```

这个模块在整个项目中不存在。这个文件要么是死代码（从未被调用），要么一运行就 `ImportError` 崩溃。如果是死代码，应该删除；如果是活代码，这是严重 bug。

### 2. [`lidar_node.py:71`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py:71) — 已修复：背景地图路径不再硬编码

`lidar_node.py` 现在通过 [`shared/paths.py`](../../src/hnurm_radar/hnurm_radar/shared/paths.py) 的 `resolve_path()` 解析 `configs/main_config.yaml` 里的 `lidar.background_map_path`，这个问题已经从“硬编码绝对路径”修成了“可配置路径”。

### 3. [`radar.py:350-352`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py:350-352) — 开发者自己都困惑的代码

```python
center = center[:3]                    # 取前三维
center = np.hstack((center, np.array((1))))  # 为什么要齐次化两次？
```

开发者自己留下了注释"为什么要齐次化两次？"——说明这段逻辑连写的人自己都没想清楚。这是典型的"试出来的代码"，没有经过严谨推导。

### 4. [`_deprecated.py:71-94`](../../src/hnurm_radar/hnurm_radar/communication/_deprecated.py:71-94) — 语法错误

```python
def parse_frame(self, serial_port):
```

这个函数定义了 `self` 参数但**不是类方法**，只是一个模块级函数。如果有人调用 `parse_frame(port)` 会传错参数，如果当方法调用会报 `TypeError`。

### 5. [`Converter.py`](../../src/hnurm_radar/hnurm_radar/Lidar/Converter.py) — CuPy 硬依赖导致无 GPU 直接崩溃

整个 [`Converter.py`](../../src/hnurm_radar/hnurm_radar/Lidar/Converter.py) 重度依赖 CuPy（GPU 加速），但没有任何 fallback 到 NumPy 的逻辑。在比赛场地如果 GPU 驱动有问题、或者部署在无 GPU 的工控机上，整个 LiDAR 方案直接不可用。

---

## 二、🏗️ 架构设计问题

### 6. 三个方案代码严重重复，缺乏复用

- 纯相机方案（[`camera_detector.py`](../../src/hnurm_radar/hnurm_radar/camera_scheme/camera_detector.py)）和融合方案（[`detector_node.py`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/detector_node.py)）都实现了几乎相同的 YOLO 推理逻辑，但各自写了一遍。
- 空中方案（[`air_target_node.py`](../../src/hnurm_radar/hnurm_radar/air_scheme/air_target_node.py)）和地面融合方案（[`radar.py`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py)）都做了 TF 坐标变换、DBSCAN 聚类，但代码完全独立。
- 项目已经有了 [`core/`](../../src/hnurm_radar/hnurm_radar/core/) 抽象基类（`BaseDetector`、`BaseTracker`、`BaseSensor`），但**实际代码几乎没有继承它们**，抽象层形同虚设。

### 7. 两个 Kalman Filter 实现，互不兼容

- [`filters/kalman_filter.py`](../../src/hnurm_radar/hnurm_radar/filters/kalman_filter.py) — 基于 OpenCV 的 `KalmanFilter`，用于地面目标坐标平滑
- [`air_scheme/air_kalman_filter.py`](../../src/hnurm_radar/hnurm_radar/air_scheme/air_kalman_filter.py) — 手写 2D Kalman Filter，用于空中目标跟踪
- [`ekf/ekf_node.py`](../../src/ekf/ekf/ekf_node.py) — 第三个 EKF 实现，用于全局坐标融合

三个滤波器各有各的接口、参数体系、状态定义，没有任何共享基类。如果要修改滤波策略（比如从 CV 模型切到 CA 模型），需要改三个地方。

### 8. 线程模型混乱

- [`camera_detector.py`](../../src/hnurm_radar/hnurm_radar/camera_scheme/camera_detector.py) 中，`_sync_frame` 和 `_infer_loop` 分别在两个线程中运行，通过 `_frame_lock` 同步，但 `_get_frame` 和 `_infer_loop` 之间没有明确的帧同步机制（没有 condition variable 或 queue），存在竞态条件风险。
- [`Lidar.py`](../../src/hnurm_radar/hnurm_radar/Lidar/Lidar.py) 在独立线程中运行 ROS2 spin，与 ROS2 的线程安全模型冲突。
- [`judge_messager.py`](../../src/hnurm_radar/hnurm_radar/communication/judge_messager.py) 使用 `multiprocessing.shared_memory` 做跨进程通信，但 [`referee_receiver.py`](../../src/hnurm_radar/hnurm_radar/communication/referee_receiver.py) 中没有任何锁机制保护共享内存的读写。

### 9. 配置系统碎片化

配置分散在：
- [`main_config.yaml`](../../configs/main_config.yaml)（342 行，主配置）
- [`detector_config.yaml`](../../configs/detector_config.yaml)（40 行，检测器配置）
- [`bytetrack.yaml`](../../configs/bytetrack.yaml)（ByteTrack 配置）
- [`converter_config.yaml`](../../configs/converter_config.yaml) / [`converter_config_rosbag.yaml`](../../configs/converter_config_rosbag.yaml)（Converter 配置）
- [`HAP_config.json`](../../configs/HAP_config.json)（LiDAR 驱动配置）
- [`perspective_calib.json`](../../configs/perspective_calib.json)（标定数据）

同一个参数（如相机曝光时间）可能出现在多个文件中，修改时容易遗漏。

---

## 三、📝 代码质量与风格问题

### 10. 大量 TODO 和占位符

- [`setup.py`](../../src/hnurm_radar/setup.py): `description='TODO: Package description'`, `license='TODO: License declaration'`
- [`ekf/setup.py`](../../src/ekf/setup.py): 同上
- 多个文件中有 `pass` 或 `# TODO` 未实现的功能

### 11. 重复代码

- [`Tools/Tools.py`](../../src/hnurm_radar/hnurm_radar/Tools/Tools.py) 和 [`communication/_deprecated.py`](../../src/hnurm_radar/hnurm_radar/communication/_deprecated.py) 中有一个完全相同的 `Tools` 类（帧率控制），代码完全复制粘贴。
- [`radar.py:184-185`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/radar.py:184-185): 同一行 `self.get_logger().error(...)` 出现了两次。

### 12. 异常处理过于宽泛

几乎所有 try/except 都是：
```python
try:
    ...
except Exception as e:
    self.get_logger().error(f"xxx 失败: {e}")
```

这种写法会吞掉 `KeyboardInterrupt`、`SystemExit` 等重要信号，且无法区分"可恢复的暂时性错误"和"需要重启的致命错误"。

### 13. 魔法数字和硬编码

- [`air_target_node.py`](../../src/hnurm_radar/hnurm_radar/air_scheme/air_target_node.py) 中大量魔法数字（帧率阈值、距离阈值、时间阈值），没有命名常量。
- [`Car.py`](../../src/hnurm_radar/hnurm_radar/Car/Car.py) 中 `life_span_init = 20` 硬编码。
- [`display_panel.py`](../../src/hnurm_radar/hnurm_radar/shared/display_panel.py) 中地图尺寸、颜色值硬编码。

### 14. 类型注解缺失或不一致

虽然有部分类型注解（主要在 `core/` 和 `air_scheme/` 中），但大部分代码（`Camera/`、`Lidar/`、`Car/`、`communication/`）完全没有类型注解。`Converter.py` 中大量函数返回 `Any` 或没有返回值注解。

---

## 四、🧪 测试与质量保障

### 15. 测试几乎为零

- `tests/` 目录存在但只有占位文件。
- `ekf/test/` 下只有 ROS2 生成的模板测试（`test_copyright.py`、`test_flake8.py`、`test_pep257.py`），没有实际的功能测试。
- 核心算法（DBSCAN 聚类、Kalman 滤波、坐标变换、YOLO 推理管线）没有任何单元测试。
- 没有集成测试来验证三个方案的端到端流程。

### 16. `package.xml` 依赖声明不完整

[`package.xml`](../../src/hnurm_radar/package.xml) 只声明了 `rclpy`、`sensor_msgs`、`pcl_conversions`、`livox_ros_driver2`、`example_interfaces` 五个依赖。但实际代码依赖：
- `numpy`、`opencv-python`、`open3d`、`scikit-learn`、`cupy`、`PyQt5`、`ultralytics`、`torch`、`MvCamera` 等
- 这些只在 [`requirements.txt`](../../requirements.txt) 中列出，ROS2 的 `package.xml` 完全不包含 Python 包依赖

---

## 五、📋 文档问题

### 17. README 信息不足

- 没有架构图或数据流图
- 没有说明三个方案的切换方式
- 没有说明配置文件之间的关系
- 没有说明如何添加新的检测方案
- 启动脚本（`bringup.sh`）没有注释说明每个步骤的作用

### 18. 代码注释质量参差不齐

- 好的部分：`air_scheme/` 中的注释比较详细，说明了与 HITS 的对齐关系
- 差的部分：`Converter.py` 中大量函数没有 docstring，只有一行中文描述
- 误导性注释：部分注释已经过时，与实际代码行为不符

---

## 六、⚡ 性能与安全

### 19. 调试代码可能影响比赛

- [`ekf_node.py`](../../src/ekf/ekf/ekf_node.py) 中的 `debug_coordinate_publish` 模式在比赛时如果忘记关闭，会额外发布大量调试 topic，可能影响带宽和延迟。
- [`camera_detector.py`](../../src/hnurm_radar/hnurm_radar/camera_scheme/camera_detector.py) 中使用 `cv2.imshow()` 显示图像，这在无显示器的比赛场地会崩溃（OpenCV HighGUI 需要 GUI 环境）。

### 20. 内存管理风险

- [`lidar_node.py`](../../src/hnurm_radar/hnurm_radar/lidar_scheme/lidar_node.py) 中的 `PcdQueue` 没有大小限制检查，如果点云发布频率高于处理频率，内存会无限增长。
- [`air_target_node.py`](../../src/hnurm_radar/hnurm_radar/air_scheme/air_target_node.py) 中的帧缓冲区也没有上限保护。

### 21. 日志过于冗长

- 每个 TF 查询失败都打印 error 日志（`radar.py`、`air_target_node.py`），在启动初期 TF 还没就绪时会刷屏。
- 部分 debug 信息使用 `print()` 而非 ROS2 logger（`Converter.py`、`PointCloud.py`）。

---

## 七、💡 改进建议（按优先级排序）

| 优先级 | 改进项 | 说明 |
|--------|--------|------|
| P0 | 修复 `coord_solver.py` 的 `import` 错误 | 删除死代码或修复导入 |
| P0 | 消除硬编码路径 | 全部改用 `shared/paths.py` 的 `resolve_path()` |
| P0 | 修复 `_deprecated.py` 的语法错误 | 删除或修复 `parse_frame` |
| P1 | 统一三个 Kalman Filter | 提取公共基类，消除重复 |
| P1 | 补全 `package.xml` 依赖 | 至少声明所有 Python 运行时依赖 |
| P1 | 添加 GPU fallback | `Converter.py` 在 CuPy 不可用时回退到 NumPy |
| P1 | 移除 `cv2.imshow` 依赖 | 改用 ROS2 topic 发布图像 |
| P2 | 实现抽象基类的实际继承 | 让 `core/` 层的设计落地 |
| P2 | 添加关键算法的单元测试 | 特别是 Kalman Filter、DBSCAN、坐标变换 |
| P2 | 统一配置系统 | 合并分散的配置文件，或至少文档化依赖关系 |
| P2 | 修复重复代码 | 删除 `_deprecated.py` 中的 `Tools` 类 |
| P3 | 添加类型注解 | 特别是 `Camera/`、`Lidar/`、`communication/` 模块 |
| P3 | 规范化异常处理 | 区分致命错误和可恢复错误 |
| P3 | 添加架构文档 | 数据流图、方案切换说明、配置关系图 |

---

## 总结

**HNURM Radar 2026 是一个"算法思路正确、工程实现粗糙"的典型比赛项目。** 核心检测和跟踪算法的选型是合理的，三个方案覆盖了不同的比赛场景。但代码质量、架构设计、测试覆盖、文档完整性都处于"能跑就行"的水平。

最大的问题不是某个具体的 bug，而是**缺乏统一的工程规范**：三个方案各自为政、配置分散、重复代码多、抽象层形同虚设。如果项目要继续迭代到 2027 赛季，建议先做一轮**架构重构**，把三个方案的公共逻辑提取出来，统一配置、统一滤波、统一坐标变换，否则维护成本会越来越高。

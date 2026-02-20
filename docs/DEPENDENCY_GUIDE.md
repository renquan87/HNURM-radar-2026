# 项目依赖配置详解

> AI生成，仅供参考

> 本文档以 `hnurm_radar` 项目为实例，系统性地阐述项目依赖管理的核心概念、配置文件结构及其工作原理。适用于初次接触多语言混合 ROS 2 工程的开发者。


---

## 目录

1. [项目依赖概述](#1-项目依赖概述)
2. [技术栈总览](#2-技术栈总览)
3. [依赖的分层架构](#3-依赖的分层架构)
4. [第一层：操作系统级依赖](#4-第一层操作系统级依赖)
5. [第二层：ROS 2 框架层依赖](#5-第二层ros-2-框架层依赖)
6. [第三层：Python 依赖与虚拟环境](#6-第三层python-依赖与虚拟环境)
7. [第四层：C++ 依赖与 CMake 构建系统](#7-第四层c-依赖与-cmake-构建系统)
8. [第五层：项目内部包间依赖](#8-第五层项目内部包间依赖)
9. [核心配置文件逐一解析](#9-核心配置文件逐一解析)
10. [各子包依赖清单](#10-各子包依赖清单)
11. [依赖安装完整流程](#11-依赖安装完整流程)
12. [常见问题与排查](#12-常见问题与排查)
13. [依赖配置全景总结](#13-依赖配置全景总结)

---

## 1. 项目依赖概述

### 1.1 定义

**项目依赖**（Dependencies）是指项目在编译、运行或测试阶段所需引用的外部代码库、框架、工具链及系统组件的总称。

### 1.2 依赖管理需要解决的四个核心问题

| 问题 | 说明 |
|------|------|
| 依赖标识 | 明确项目需要哪些外部库及其名称 |
| 版本约束 | 指定兼容的版本范围，确保 API 一致性与稳定性 |
| 安装方式 | 确定通过何种包管理器或构建方式获取依赖（`pip`、`apt`、`cmake`、源码编译等） |
| 隔离策略 | 确定依赖的安装作用域（系统全局、虚拟环境、ROS workspace 等），避免版本冲突 |

### 1.3 本项目的特殊性

本项目是一个 ROS 2 机器人工程，同时包含 Python 包和 C++ 包，因此涉及**两套独立但协同工作的依赖管理体系**：

- Python 侧：通过 `pip` + 虚拟环境 + `requirements.txt` 管理
- C++ 侧：通过 `CMake` + `find_package()` + 系统包管理器管理
- 两者通过 ROS 2 的 `colcon` 构建系统和 `package.xml` 统一协调

---

## 2. 技术栈总览

| 技术领域 | 具体技术 | 实现语言 |
|---------|---------|---------|
| 机器人通信框架 | ROS 2 Humble Hawksbill | — |
| 目标检测 | Ultralytics (YOLOv8) + PyTorch | Python |
| 点云处理 | PCL (Point Cloud Library) | C++ |
| 点云配准 | Quatro + TEASER++ | C++ |
| 激光雷达驱动 | Livox SDK2 + livox_ros_driver2 | C++ |
| 状态估计 | 扩展卡尔曼滤波 (EKF) | Python |
| GPU 加速 | CUDA + TensorRT + CuPy | C/C++/Python |
| 图像处理 | OpenCV | Python |
| 数据可视化 | Matplotlib, Plotly, Dash | Python |

---

## 3. 依赖的分层架构

本项目的依赖呈现清晰的分层结构，上层依赖于下层：

```
┌──────────────────────────────────────────────────┐
│  第 5 层：项目内部包间依赖                         │
│  hnurm_radar ─→ detect_result, livox_ros_driver2 │
├──────────────────────────────────────────────────┤
│  第 4 层：C++ 第三方库                             │
│  PCL, Eigen3, TEASER++, OpenMP, TBB              │
├──────────────────────────────────────────────────┤
│  第 3 层：Python 第三方库                          │
│  PyTorch, OpenCV, NumPy, CuPy, Open3D ...        │
│  管理方式：虚拟环境 + requirements.txt             │
├──────────────────────────────────────────────────┤
│  第 2 层：ROS 2 框架                              │
│  rclpy, rclcpp, sensor_msgs, ament 构建系统       │
├──────────────────────────────────────────────────┤
│  第 1 层：操作系统与基础工具链                      │
│  Ubuntu 22.04, GCC, CMake, Python 3.10           │
└──────────────────────────────────────────────────┘
```

以下逐层详述。

---

## 4. 第一层：操作系统级依赖

操作系统级依赖通过 Ubuntu 的包管理器 `apt` 安装，作用于系统全局，所有用户和项目均可访问。

### 4.1 apt 包管理器

`apt`（Advanced Package Tool）是 Debian/Ubuntu 系统的标准包管理工具。其核心功能包括：

- 从官方软件源下载并安装预编译的二进制包
- 自动解析并安装传递性依赖
- 管理包的升级与卸载

### 4.2 本项目所需的系统级依赖

```bash
# 基础编译工具链
sudo apt install build-essential cmake git

# ROS 2 Humble 完整桌面版（含 rviz2、rqt 等可视化工具）
sudo apt install ros-humble-desktop

# PCL 点云库（C++ 点云处理的事实标准）
sudo apt install libpcl-dev

# Eigen3 线性代数库（矩阵运算、坐标变换）
sudo apt install libeigen3-dev

# Intel TBB 并行计算库（Quatro 使用）
sudo apt install libtbb-dev

# Apache Portable Runtime（Livox 驱动的网络通信层依赖）
sudo apt install libapr1-dev

# colcon 构建工具（ROS 2 工作空间的编译入口）
sudo apt install python3-colcon-common-extensions
```

### 4.3 系统级依赖的特征

- 安装路径固定：头文件位于 `/usr/include/`，库文件位于 `/usr/lib/`
- 版本由系统软件源决定，通常与 Ubuntu 发行版绑定
- 升级需谨慎，可能影响系统其他组件

---

## 5. 第二层：ROS 2 框架层依赖

### 5.1 ROS 2 简介

ROS 2（Robot Operating System 2）是面向机器人应用的分布式通信框架。其核心功能包括：

- 节点间的发布/订阅（Pub/Sub）与服务/客户端（Service/Client）通信
- 标准化的消息类型定义（`sensor_msgs`、`geometry_msgs` 等）
- 统一的构建系统（`ament` + `colcon`）
- 坐标变换（`tf2`）、参数管理、生命周期管理等基础设施

### 5.2 colcon 构建系统

`colcon` 是 ROS 2 的标准构建工具，负责统一管理工作空间中所有包的编译过程。

```bash
# 在工作空间根目录执行
colcon build
```

其工作流程如下：

1. 扫描 `src/` 目录下所有包含 `package.xml` 的子目录
2. 解析每个包的 `package.xml`，提取依赖声明
3. 基于依赖关系构建有向无环图（DAG），确定编译拓扑顺序
4. 根据 `<build_type>` 标签选择对应的构建后端：
   - `ament_cmake` → 调用 CMake 编译 C++ 代码
   - `ament_python` → 调用 setuptools 安装 Python 包
5. 将编译产物输出至 `build/` 和 `install/` 目录

### 5.3 ROS 2 包的两种构建类型

| 构建类型 | 适用语言 | 核心配置文件 | 本项目实例 |
|---------|---------|------------|-----------|
| `ament_python` | Python | `setup.py` + `setup.cfg` + `package.xml` | `hnurm_radar`、`ekf` |
| `ament_cmake` | C/C++ | `CMakeLists.txt` + `package.xml` | `detect_result`、`registration`、`livox_ros_driver2`、`Quatro`、`hnurm_bringup` |

两种类型的包均需要 `package.xml` 作为元数据描述文件，这是 ROS 2 包的统一标识。

### 5.4 环境变量加载机制

ROS 2 通过 shell 脚本注入环境变量，使系统能够定位已安装的包、消息类型和可执行文件：

```bash
# 加载 ROS 2 基础环境（提供 ros2 命令、标准消息类型等）
source /opt/ros/humble/setup.bash

# 加载本工作空间的编译产物（使自定义包可被发现）
source install/setup.bash
```

这两条 `source` 命令的本质是设置 `AMENT_PREFIX_PATH`、`PYTHONPATH`、`LD_LIBRARY_PATH` 等环境变量，将 ROS 2 的库路径和 Python 模块路径注入当前 shell 会话。

---

## 6. 第三层：Python 依赖与虚拟环境

### 6.1 Python 虚拟环境的原理

Python 虚拟环境（Virtual Environment）是一种**依赖隔离机制**，其核心原理是：

- 在指定目录下创建独立的 `bin/`（含 `python`、`pip` 可执行文件）和 `lib/site-packages/`（第三方包安装目录）
- 激活虚拟环境后，shell 的 `PATH` 被修改，使 `python` 和 `pip` 指向虚拟环境内的副本
- 不同虚拟环境之间的包完全隔离，互不影响

```
系统 Python: /usr/bin/python3
│
├── 虚拟环境: /data/venv/radar-env/     ← 本项目使用
│   ├── bin/python3                     ← 激活后 python 指向此处
│   ├── bin/pip
│   └── lib/python3.10/site-packages/   ← 第三方包安装于此
│       ├── torch==2.5.1
│       ├── numpy==1.23.0
│       └── opencv_python==4.10.0.84
│
└── 其他虚拟环境: /data/venv/other-project/
    └── lib/python3.10/site-packages/
        └── numpy==2.0.0               ← 与上方版本不冲突
```

### 6.2 本项目的虚拟环境配置

本项目的虚拟环境路径为 `/data/venv/radar-env/`，在启动脚本 `bringup.sh` 中通过以下方式激活：

```bash
VENV_PATH="/data/venv/radar-env/bin/activate"
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
fi
```

激活后，`which python` 将返回 `/data/venv/radar-env/bin/python`，所有 `pip install` 操作均作用于该虚拟环境。

### 6.3 requirements.txt 解析

`requirements.txt` 是 Python 生态中标准的依赖声明文件，每行指定一个包及其版本约束：

```
包名==精确版本号
```

#### 6.3.1 本项目 requirements.txt 的生成方式

本项目的 `requirements.txt` 由 `pip freeze` 命令生成，该命令会导出当前环境中**所有已安装包**（包括直接依赖和传递性依赖），共计约 300 个条目。

#### 6.3.2 直接依赖与传递性依赖

| 类别 | 定义 | 示例 |
|------|------|------|
| 直接依赖 | 项目代码中显式 `import` 的包 | `torch`、`opencv-python`、`numpy` |
| 传递性依赖 | 被直接依赖自动引入的包 | `pillow`（被 `torchvision` 依赖）、`sympy`（被 `torch` 依赖） |

`pip freeze` 不区分两者，这是其局限性。更规范的做法是维护一份仅包含直接依赖的精简文件，再通过 `pip-compile`（`pip-tools` 工具链）生成锁定文件。

#### 6.3.3 本项目的核心 Python 依赖

从 `requirements.txt` 中提取的核心直接依赖如下：

| 包名 | 版本 | 用途 |
|------|------|------|
| `torch` | 2.5.1 | PyTorch 深度学习框架 |
| `torchvision` | 0.20.1 | PyTorch 计算机视觉工具库 |
| `opencv-python` | 4.10.0.84 | 图像处理 |
| `numpy` | 1.23.0 | 数值计算基础库 |
| `open3d` | 0.18.0 | 3D 点云处理与可视化 |
| `cupy` | 13.3.0 | 基于 CUDA 的 GPU 加速数组运算 |
| `scipy` | 1.8.0 | 科学计算（滤波、优化等） |
| `scikit-learn` | 1.5.2 | 机器学习工具库 |
| `ultralytics-thop` | 2.0.9 | 模型 FLOPs 计算 |
| `PyYAML` | 5.4.1 | YAML 配置文件解析 |
| `matplotlib` | 3.5.1 | 数据可视化 |
| `lapx` | 0.5.11 | 线性分配问题求解（目标跟踪） |
| `pyserial` | 3.5 | 串口通信 |
| `plotly` | 5.24.1 | 交互式数据可视化 |
| `dash` | 2.18.1 | Web 数据面板 |

#### 6.3.4 NVIDIA CUDA 相关依赖

本项目使用 GPU 加速推理，因此 `requirements.txt` 中包含大量 NVIDIA 相关包：

```
nvidia-cublas-cu12==12.4.5.8
nvidia-cuda-runtime-cu12==12.4.127
nvidia-cudnn-cu12==9.1.0.70
nvidia-tensorrt==8.4.3.1
```

这些包提供 CUDA 运行时库、cuDNN 深度学习加速库和 TensorRT 推理引擎的 Python 绑定。

#### 6.3.5 被注释的 ROS 2 相关条目

`requirements.txt` 中大量条目被 `#` 注释：

```
# colcon-cmake==0.2.28
# ros2topic==0.18.11
# rosidl-generator-py==0.14.4
```

这些是 ROS 2 框架自带的 Python 包，随 `ros-humble-desktop` 一同安装于系统 Python 环境中。若通过 `pip install` 在虚拟环境中重复安装，可能导致版本冲突或路径覆盖问题，因此予以注释排除。

### 6.4 Python 依赖安装步骤

```bash
# 步骤 1：创建虚拟环境
python3 -m venv /data/venv/radar-env

# 步骤 2：激活虚拟环境
source /data/venv/radar-env/bin/activate

# 步骤 3：升级 pip（建议）
pip install --upgrade pip

# 步骤 4：安装依赖
pip install -r requirements.txt
```

---

## 7. 第四层：C++ 依赖与 CMake 构建系统

### 7.1 CMake 概述

CMake 是跨平台的 C/C++ 构建系统生成器。其职责是：

1. 读取 `CMakeLists.txt` 中的构建规则
2. 在系统中查找所需的头文件和库文件
3. 生成平台原生的构建文件（如 Unix Makefile 或 Ninja 文件）
4. 调用底层编译器（GCC/Clang）完成编译和链接

### 7.2 CMake 中的依赖查找机制

#### 7.2.1 `find_package()`

`find_package()` 是 CMake 查找外部依赖的核心命令。其工作流程：

1. 在标准路径（`/usr/lib/cmake/`、`/usr/local/lib/cmake/` 等）中搜索 `<PackageName>Config.cmake` 或 `Find<PackageName>.cmake` 文件
2. 执行该文件，设置头文件路径变量（如 `PCL_INCLUDE_DIRS`）和库文件路径变量（如 `PCL_LIBRARIES`）
3. 后续代码即可通过这些变量引用依赖

示例（`registration/CMakeLists.txt`）：

```cmake
find_package(PCL REQUIRED)        # 查找 PCL 点云库
find_package(Eigen3 REQUIRED)     # 查找 Eigen3 线性代数库
find_package(OpenMP REQUIRED)     # 查找 OpenMP 并行计算支持
find_package(teaserpp REQUIRED)   # 查找 TEASER++ 配准库
```

`REQUIRED` 关键字表示该依赖为必需项，若未找到则终止构建并报错。

#### 7.2.2 `find_library()`

`find_library()` 用于在指定路径中查找特定的库文件，适用于未提供 CMake 配置文件的第三方库。

示例（`livox_ros_driver2/CMakeLists.txt`）：

```cmake
find_library(LIVOX_LIDAR_SDK_LIBRARY liblivox_lidar_sdk_shared.so
    PATHS /usr/local/lib
          ~/lib
          /home/rq/radar/hnurm_radar/src/Livox-SDK2/build/sdk_core
    REQUIRED)
```

此命令在指定的多个路径中搜索 `liblivox_lidar_sdk_shared.so` 文件。

#### 7.2.3 `ament_auto_find_build_dependencies()`

这是 ROS 2 `ament_cmake_auto` 提供的便捷宏，自动从 `package.xml` 中读取 `<depend>` 和 `<build_depend>` 标签，并逐一调用 `find_package()`：

```cmake
ament_auto_find_build_dependencies(REQUIRED
  rclcpp sensor_msgs tf2 tf2_ros pcl_conversions pcl_ros quatro
)
```

等价于手动编写多条 `find_package()` 调用。

### 7.3 编译目标与库链接

找到依赖后，需要将其与编译目标关联：

```cmake
# 声明编译目标（可执行文件）
ament_auto_add_executable(registration_node
  src/main.cpp
  src/registration_node.cpp
)

# 指定头文件搜索路径
target_include_directories(registration_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

# 链接依赖库
target_link_libraries(registration_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  rclcpp::rclcpp
  OpenMP::OpenMP_CXX
  teaserpp::teaser_registration
  teaserpp::teaser_io
)
```

`target_link_libraries()` 的作用是在链接阶段将指定库的目标代码合并到最终的可执行文件中。

### 7.4 需要源码编译的 C++ 依赖

部分 C++ 依赖未提供预编译的系统包，需从源码手动编译安装。

#### 7.4.1 Livox-SDK2

Livox-SDK2 是 Livox 激光雷达的底层通信 SDK，编译后生成动态链接库 `liblivox_lidar_sdk_shared.so`。

```bash
cd src/Livox-SDK2
mkdir build && cd build
cmake ..
make -j$(nproc)
# 产物：build/sdk_core/liblivox_lidar_sdk_shared.so
```

`livox_ros_driver2` 在编译时通过 `find_library()` 定位该 `.so` 文件。

#### 7.4.2 TEASER++

TEASER++ 是鲁棒点云配准算法库，`Quatro` 和 `registration` 均依赖于它。需按照其官方文档从源码编译并安装至系统路径：

```bash
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF
make -j$(nproc)
sudo make install
# 安装后 find_package(teaserpp REQUIRED) 即可找到
```

---

## 8. 第五层：项目内部包间依赖

### 8.1 包间依赖拓扑图

本项目 `src/` 目录下包含 7 个 ROS 2 包，其依赖关系如下：

```
                     hnurm_bringup（启动器）
                    /        |        \
                   /         |         \
     hnurm_radar    livox_ros_driver2   registration
          |               |                  |
     detect_result    Livox-SDK2          Quatro
          |                               /    \
         ekf                       TEASER++    PCL
```

### 8.2 依赖关系详表

| 包名 | 构建类型 | 依赖的 ROS 包 | 依赖的系统库 | 依赖的项目内部包 |
|------|---------|-------------|------------|----------------|
| `hnurm_bringup` | ament_cmake | `launch`, `launch_ros` | — | 通过 launch 文件间接依赖其他包 |
| `hnurm_radar` | ament_python | `rclpy`, `sensor_msgs`, `example_interfaces` | — | `detect_result`, `livox_ros_driver2` |
| `detect_result` | ament_cmake | `rosidl_default_generators/runtime` | — | — |
| `ekf` | ament_python | `rclpy` | — | — |
| `registration` | ament_cmake | `rclcpp`, `sensor_msgs`, `tf2`, `tf2_ros`, `pcl_conversions`, `pcl_ros`, `geometry_msgs` | PCL, Eigen3, OpenMP, TEASER++ | `quatro` |
| `Quatro` | ament_cmake | — | Eigen3, PCL, TEASER++, TBB, OpenMP | — |
| `livox_ros_driver2` | ament_cmake | `rclcpp`, `rclcpp_components`, `std_msgs`, `sensor_msgs`, `pcl_conversions` | PCL, APR | Livox-SDK2（通过 `find_library` 引用） |

### 8.3 colcon 的编译顺序推导

`colcon build` 通过解析所有包的 `package.xml`，构建依赖的有向无环图（DAG），自动推导出合法的编译顺序。例如：

1. `Livox-SDK2`（独立 CMake 项目，需预先手动编译）
2. `detect_result`（无内部依赖，最先编译）
3. `Quatro`（仅依赖系统库）
4. `livox_ros_driver2`（依赖 Livox-SDK2 的 `.so` 文件）
5. `ekf`（仅依赖 `rclpy`）
6. `registration`（依赖 `Quatro`）
7. `hnurm_radar`（依赖 `detect_result`、`livox_ros_driver2`）
8. `hnurm_bringup`（最后编译，仅包含 launch 文件）

---

## 9. 核心配置文件逐一解析

### 9.1 `requirements.txt`

| 属性 | 说明 |
|------|------|
| 文件位置 | 项目根目录 `/requirements.txt` |
| 所属体系 | Python / pip |
| 作用 | 声明 Python 虚拟环境中需要安装的所有 pip 包及其精确版本 |
| 使用命令 | `pip install -r requirements.txt` |
| 生成命令 | `pip freeze > requirements.txt` |

本项目 `requirements.txt` 的特征：

- 共约 300 个条目，由 `pip freeze` 全量导出
- 版本号均使用 `==` 精确锁定，确保环境可复现
- ROS 2 相关的 Python 包（如 `colcon-*`、`ros2*`、`rosidl-*`）已被注释排除
- 包含 NVIDIA CUDA/cuDNN/TensorRT 的 Python 绑定包

### 9.2 `package.xml`

| 属性 | 说明 |
|------|------|
| 文件位置 | 每个 ROS 2 包的根目录 |
| 所属体系 | ROS 2 / ament |
| 作用 | ROS 2 包的元数据描述文件，声明包名、版本、维护者、许可证及依赖关系 |
| 使用者 | `colcon build`、`rosdep`、`bloom` |

#### 关键 XML 标签说明

| 标签 | 含义 | 示例 |
|------|------|------|
| `<buildtool_depend>` | 构建工具依赖 | `ament_cmake`、`ament_python` |
| `<depend>` | 编译期 + 运行期均需要的依赖 | `rclpy`、`sensor_msgs` |
| `<build_depend>` | 仅编译期需要的依赖 | `rosidl_default_generators` |
| `<exec_depend>` | 仅运行期需要的依赖 | `rosidl_default_runtime`、`launch` |
| `<test_depend>` | 仅测试期需要的依赖 | `ament_copyright`、`python3-pytest` |
| `<member_of_group>` | 声明所属的包组 | `rosidl_interface_packages` |
| `<export><build_type>` | 声明构建类型 | `ament_cmake` 或 `ament_python` |

#### 实例：`hnurm_radar/package.xml`

```xml
<package format="3">
  <name>hnurm_radar</name>

  <!-- 编译 + 运行均需要的依赖 -->
  <depend>rclpy</depend>              <!-- ROS 2 Python 客户端库 -->
  <depend>sensor_msgs</depend>        <!-- 传感器消息类型（图像、点云等） -->
  <depend>pcl_conversions</depend>    <!-- PCL 与 ROS 消息的转换工具 -->
  <depend>livox_ros_driver2</depend>  <!-- Livox 雷达驱动（项目内部包） -->
  <depend>example_interfaces</depend> <!-- ROS 2 标准示例接口 -->

  <!-- 构建工具 -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- 测试依赖 -->
  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <!-- 声明构建类型为 ament_python -->
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 实例：`detect_result/package.xml`

```xml
<package format="3">
  <name>detect_result</name>

  <!-- 编译期依赖：消息代码生成器 -->
  <build_depend>rosidl_default_generators</build_depend>

  <!-- 运行期依赖：消息类型的运行时支持 -->
  <exec_depend>rosidl_default_runtime</exec_depend>

  <!-- 构建工具 -->
  <buildtool_depend>ament_cmake</buildtool_depend>

  <!-- 声明本包属于 rosidl 接口包组 -->
  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

`<build_depend>` 与 `<exec_depend>` 的区分在消息定义包中尤为重要：`rosidl_default_generators` 仅在编译期用于从 `.msg` 文件生成代码，运行时不再需要；而 `rosidl_default_runtime` 提供生成代码的运行时支持库。

### 9.3 `setup.py` 与 `setup.cfg`

这两个文件仅存在于 `ament_python` 类型的 ROS 2 包中。

#### 9.3.1 `setup.py`

| 属性 | 说明 |
|------|------|
| 文件位置 | `ament_python` 包的根目录 |
| 所属体系 | Python / setuptools |
| 作用 | 声明 Python 包的元信息、安装依赖、数据文件及可执行入口点 |

以 `hnurm_radar/setup.py` 为例，逐字段解析：

```python
from setuptools import find_packages, setup

package_name = 'hnurm_radar'

setup(
    name=package_name,          # 包名，须与 package.xml 中的 <name> 一致
    version='0.0.0',            # 版本号

    # 自动发现 src/hnurm_radar/ 下所有含 __init__.py 的子目录作为 Python 子包
    packages=find_packages(exclude=['test']),

    # 非代码文件的打包规则（如 YAML 配置文件）
    package_data={
        'ultralytics': ['cfg/**/*.yaml', 'nn/backbone/faster_cfg/*.yaml'],
    },

    # ROS 2 ament 索引所需的数据文件
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],

    # Python 安装依赖（此处仅声明 setuptools，实际依赖通过 requirements.txt 管理）
    install_requires=['setuptools'],

    # 测试依赖
    tests_require=['pytest'],

    # 可执行入口点定义——这是 ROS 2 Python 节点注册的关键
    entry_points={
        'console_scripts': [
            # 格式："ros2_run_命令名 = Python模块路径:入口函数"
            "camera_detector = hnurm_radar.camera_scheme.camera_detector:main",
            "detector_node = hnurm_radar.lidar_scheme.detector_node:main",
            "radar_node = hnurm_radar.lidar_scheme.radar:main",
            "lidar_node = hnurm_radar.lidar_scheme.lidar_node:main",
            "publish_video = hnurm_radar.shared.publish_video:main",
            "display_panel = hnurm_radar.shared.display_panel:main",
            "judge_messager = hnurm_radar.shared.judge_messager:main",
            "make_mask = hnurm_radar.camera_locator.make_mask:main",
            "perspective_calibrator = hnurm_radar.camera_locator.perspective_calibrator:main",
        ],
    },
)
```

`entry_points` 中的 `console_scripts` 是 ROS 2 Python 包的核心配置。`colcon build` 会根据此配置在 `install/lib/<package_name>/` 下生成可执行脚本，使 `ros2 run hnurm_radar camera_detector` 能够正确调用对应模块的 `main()` 函数。

#### 9.3.2 `setup.cfg`

```ini
[develop]
script_dir=$base/lib/hnurm_radar
[install]
install_scripts=$base/lib/hnurm_radar
```

此文件指定 setuptools 生成的可执行脚本的安装路径。`$base/lib/<package_name>/` 是 ROS 2 ament_python 包的标准脚本安装位置，`colcon` 和 `ros2 run` 均依赖此路径约定来定位可执行文件。该文件为 ROS 2 Python 包的固定模板，通常无需修改。

### 9.4 `CMakeLists.txt`

| 属性 | 说明 |
|------|------|
| 文件位置 | `ament_cmake` 包的根目录 |
| 所属体系 | CMake / ament_cmake |
| 作用 | 定义 C++ 项目的编译规则、依赖查找、目标链接及安装规则 |

本项目包含 5 个 `CMakeLists.txt`，按功能复杂度分为三类：

#### 类型一：纯启动器包（`hnurm_bringup`）

```cmake
cmake_minimum_required(VERSION 3.8)
project(hnurm_bringup)

find_package(ament_cmake REQUIRED)

# 仅安装 launch 文件，无需编译任何代码
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
```

此类包不包含源代码，仅将 launch 文件安装至 ROS 2 的共享目录，供 `ros2 launch` 命令调用。

#### 类型二：消息定义包（`detect_result`）

```cmake
cmake_minimum_required(VERSION 3.8)
project(detect_result)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 从 .msg 文件自动生成 C++ 和 Python 的消息类型代码
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectResult.msg"
  "msg/Robots.msg"
  "msg/Location.msg"
  "msg/Locations.msg"
)

ament_package()
```

`rosidl_generate_interfaces()` 是 ROS 2 消息生成系统的核心宏。它读取 `.msg` 文件中的消息定义（如 `DetectResult.msg` 中的 `int32[4] xyxy_box`），自动生成：
- C++ 头文件（供 C++ 节点 `#include`）
- Python 模块（供 Python 节点 `from detect_result.msg import DetectResult`）
- IDL（Interface Definition Language）中间表示

#### 类型三：完整 C++ 节点包（`registration`、`Quatro`、`livox_ros_driver2`）

以 `Quatro/CMakeLists.txt` 为例，展示完整的 C++ 库构建流程：

```cmake
cmake_minimum_required(VERSION 3.10)
project(quatro)

set(CMAKE_CXX_STANDARD 17)

# 查找 ROS 2 构建工具
find_package(ament_cmake REQUIRED)

# 查找系统级 C++ 依赖
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(teaserpp REQUIRED)

# 可选依赖：Intel TBB 并行计算库
option(QUATRO_TBB "Enable TBB support" ON)
if(QUATRO_TBB)
  find_package(TBB REQUIRED)
  add_definitions(-DTBB_EN)
endif()

# 设置头文件搜索路径
include_directories(
  include
  ${Eigen3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# 构建共享库
add_library(${PROJECT_NAME}
  src/fpfh.cc
  src/matcher.cc
  src/quatro_module.cc
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}
  Eigen3::Eigen
  ${PCL_LIBRARIES}
  teaserpp::teaser_registration
  teaserpp::teaser_io
)

# ROS 2 导出配置（使其他 ament 包能通过 find_package(quatro) 找到本库）
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(Eigen3 PCL teaserpp)

# 安装头文件和库文件
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
```

关键流程说明：
1. `find_package()` 查找所有外部依赖
2. `add_library()` 声明编译目标（此处为共享库而非可执行文件）
3. `target_link_libraries()` 将依赖库链接至编译目标
4. `ament_export_*()` 系列命令将本库的头文件路径、库文件和依赖信息导出，使下游包（如 `registration`）能通过 `find_package(quatro)` 找到并使用本库
5. `install()` 将产物安装至 ROS 2 工作空间的标准目录

### 9.5 `bringup.sh`（启动脚本）

| 属性 | 说明 |
|------|------|
| 文件位置 | 项目根目录 `/bringup.sh` |
| 作用 | 一键配置运行环境并启动所有 ROS 2 节点 |

该脚本串联了项目运行所需的全部环境配置：

```bash
#!/bin/bash

# ① 处理动态链接库路径冲突
# 移除 MVS 相机 SDK 的库路径，避免其 libusb 与系统版本冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# ② 激活 Python 虚拟环境
# 使 python/pip 指向虚拟环境，确保 PyTorch 等依赖可用
source "/data/venv/radar-env/bin/activate"

# ③ 加载 ROS 2 基础环境
source /opt/ros/humble/setup.bash

# ④ 加载本工作空间的编译产物
source install/setup.bash

# ⑤ 在独立终端中启动各 ROS 2 launch 文件
gnome-terminal -- bash -c "source /data/venv/radar-env/bin/activate; \
  source /opt/ros/humble/setup.bash; source install/setup.bash; \
  ros2 launch livox_ros_driver2 rviz_HAP_launch.py; exec bash;"

gnome-terminal -- bash -c "source /data/venv/radar-env/bin/activate; \
  source /opt/ros/humble/setup.bash; source install/setup.bash; \
  ros2 launch hnurm_bringup hnurm_radar_launch.py; exec bash;"

gnome-terminal -- bash -c "source /data/venv/radar-env/bin/activate; \
  source /opt/ros/humble/setup.bash; source install/setup.bash; \
  ros2 launch registration registration.launch.py; exec bash;"
```

每个 `gnome-terminal` 启动的子 shell 是独立进程，不继承父 shell 的环境变量，因此需要在每个终端中重新执行 `source` 命令。这是 Linux 进程模型的固有特性。

### 9.6 `.gitignore`

| 属性 | 说明 |
|------|------|
| 文件位置 | 项目根目录 `/.gitignore` |
| 作用 | 声明不纳入 Git 版本控制的文件和目录 |

与依赖管理相关的 `.gitignore` 条目及其原因：

| 条目 | 原因 |
|------|------|
| `build/`、`install/`、`log/` | `colcon build` 的编译产物，可通过重新编译生成 |
| `__pycache__/`、`*.pyc`、`*.pyo` | Python 解释器自动生成的字节码缓存 |
| `*.egg-info/` | setuptools 构建 Python 包时生成的元数据目录 |
| `*.engine` | TensorRT 序列化的推理引擎文件，体积大且与 GPU 硬件绑定 |
| `*.onnx` | ONNX 格式的神经网络模型文件，体积较大 |
| `*.pcd` | 点云数据文件，属于运行时采集的数据 |

这些文件不入库的共同特征：要么可由构建系统重新生成，要么体积过大不适合存储在 Git 仓库中，要么是运行时产生的临时文件。

---

## 10. 各子包依赖清单

### 10.1 `hnurm_radar`（主业务包，Python）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| ROS 2 通信 | `rclpy`、`sensor_msgs`、`example_interfaces` | ROS 2 Humble |
| 消息类型 | `detect_result`（自定义消息） | 项目内部 |
| 雷达驱动 | `livox_ros_driver2`（自定义消息类型） | 项目内部 |
| 深度学习 | `torch`、`torchvision`、`ultralytics` | pip / 虚拟环境 |
| GPU 加速 | `cupy`、`nvidia-tensorrt` | pip / 虚拟环境 |
| 图像处理 | `opencv-python`、`numpy` | pip / 虚拟环境 |
| 点云处理 | `open3d` | pip / 虚拟环境 |
| 目标跟踪 | `lapx`（线性分配） | pip / 虚拟环境 |
| 配置解析 | `PyYAML` | pip / 虚拟环境 |
| 串口通信 | `pyserial` | pip / 虚拟环境 |

### 10.2 `detect_result`（消息定义包，C++/自动生成）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| 构建工具 | `ament_cmake` | ROS 2 Humble |
| 代码生成 | `rosidl_default_generators` | ROS 2 Humble |
| 运行时 | `rosidl_default_runtime` | ROS 2 Humble |

### 10.3 `ekf`（扩展卡尔曼滤波，Python）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| ROS 2 通信 | `rclpy` | ROS 2 Humble |
| 数值计算 | `numpy`、`scipy` | pip / 虚拟环境 |

### 10.4 `registration`（点云配准，C++）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| ROS 2 通信 | `rclcpp`、`sensor_msgs`、`geometry_msgs` | ROS 2 Humble |
| 坐标变换 | `tf2`、`tf2_ros`、`tf2_eigen` | ROS 2 Humble |
| 点云处理 | PCL、`pcl_conversions`、`pcl_ros` | apt (`libpcl-dev`) |
| 线性代数 | Eigen3 | apt (`libeigen3-dev`) |
| 点云配准 | TEASER++ | 源码编译 |
| 配准算法 | `quatro` | 项目内部 |
| 并行计算 | OpenMP | GCC 内置 |

### 10.5 `Quatro`（配准算法库，C++）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| 线性代数 | Eigen3 | apt (`libeigen3-dev`) |
| 点云处理 | PCL (common, io, filters) | apt (`libpcl-dev`) |
| 配准核心 | TEASER++ | 源码编译 |
| 并行计算 | Intel TBB、OpenMP | apt (`libtbb-dev`) / GCC 内置 |

### 10.6 `livox_ros_driver2`（激光雷达驱动，C++）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| ROS 2 通信 | `rclcpp`、`rclcpp_components`、`std_msgs`、`sensor_msgs` | ROS 2 Humble |
| 消息生成 | `rosidl_default_generators` | ROS 2 Humble |
| 点云处理 | PCL、`pcl_conversions` | apt (`libpcl-dev`) |
| 雷达 SDK | Livox-SDK2 (`liblivox_lidar_sdk_shared.so`) | 源码编译 |
| 网络通信 | APR (Apache Portable Runtime) | apt (`libapr1-dev`) |

### 10.7 `hnurm_bringup`（启动器包）

| 依赖类别 | 具体依赖 | 来源 |
|---------|---------|------|
| 构建工具 | `ament_cmake` | ROS 2 Humble |
| 启动框架 | `launch`、`launch_ros` | ROS 2 Humble |

---

## 11. 依赖安装完整流程

以下为在全新 Ubuntu 22.04 系统上从零配置本项目依赖的完整步骤：

### 步骤 1：安装系统级依赖

```bash
# 更新软件源
sudo apt update && sudo apt upgrade -y

# 安装基础编译工具
sudo apt install -y build-essential cmake git wget curl

# 安装 ROS 2 Humble（参照 ROS 2 官方文档添加软件源后执行）
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-colcon-common-extensions python3-rosdep

# 安装 C++ 库依赖
sudo apt install -y libpcl-dev libeigen3-dev libtbb-dev libapr1-dev
```

### 步骤 2：编译源码级 C++ 依赖

```bash
# 编译 Livox-SDK2
cd /data/projects/radar/hnurm_radar/src/Livox-SDK2
mkdir -p build && cd build
cmake ..
make -j$(nproc)

# 编译并安装 TEASER++
cd /tmp
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DBUILD_TESTS=OFF -DBUILD_PYTHON_BINDINGS=OFF
make -j$(nproc)
sudo make install
```

### 步骤 3：配置 Python 虚拟环境

```bash
# 创建虚拟环境
python3 -m venv /data/venv/radar-env

# 激活虚拟环境
source /data/venv/radar-env/bin/activate

# 升级 pip
pip install --upgrade pip

# 安装 Python 依赖
cd /data/projects/radar/hnurm_radar
pip install -r requirements.txt
```

### 步骤 4：编译 ROS 2 工作空间

```bash
# 加载 ROS 2 环境
source /opt/ros/humble/setup.bash

# 激活虚拟环境（部分 Python 包在编译期也需要）
source /data/venv/radar-env/bin/activate

# 在工作空间根目录执行编译
cd /data/projects/radar/hnurm_radar
colcon build --symlink-install

# 加载编译产物
source install/setup.bash
```

`--symlink-install` 选项使 Python 包以符号链接方式安装，修改源码后无需重新编译即可生效，适用于开发阶段。

### 步骤 5：验证安装

```bash
# 验证 ROS 2 环境
ros2 pkg list | grep hnurm

# 验证 Python 依赖
python -c "import torch; print(torch.__version__)"
python -c "import cv2; print(cv2.__version__)"

# 验证节点可执行
ros2 run hnurm_radar camera_detector --help
```

---

## 12. 常见问题与排查

### Q1：`colcon build` 报错 `Could not find a package configuration file provided by "xxx"`

该错误表示 CMake 的 `find_package()` 未能找到指定的依赖包。排查步骤：

1. 确认该依赖是否已安装：`dpkg -l | grep <package_name>` 或 `apt list --installed | grep <package_name>`
2. 若为 ROS 2 包，确认已执行 `source /opt/ros/humble/setup.bash`
3. 若为源码编译的库（如 TEASER++），确认已执行 `sudo make install` 且安装路径在 CMake 搜索路径中

### Q2：Python 节点运行时报 `ModuleNotFoundError: No module named 'torch'`

该错误表示当前 Python 环境中未安装 `torch`。排查步骤：

1. 确认已激活虚拟环境：`which python` 应返回 `/data/venv/radar-env/bin/python`
2. 确认虚拟环境中已安装该包：`pip list | grep torch`
3. 若在 `gnome-terminal` 中启动节点，确认终端内也执行了 `source /data/venv/radar-env/bin/activate`

### Q3：`requirements.txt` 中的包安装失败

常见原因及对策：

| 原因 | 对策 |
|------|------|
| CUDA 版本不匹配 | 确认系统 CUDA 版本与 `nvidia-*` 包版本对应 |
| 编译型包缺少系统依赖 | 安装对应的 `-dev` 包（如 `libffi-dev`、`libssl-dev`） |
| 网络问题 | 使用国内镜像源：`pip install -i https://pypi.tuna.tsinghua.edu.cn/simple` |
| 版本冲突 | 注释掉冲突的包，逐步排查 |

### Q4：`LD_LIBRARY_PATH` 相关的动态链接错误

运行时出现 `error while loading shared libraries: libxxx.so: cannot open shared object file` 时：

1. 确认库文件存在：`find / -name "libxxx.so" 2>/dev/null`
2. 将库文件所在目录添加至 `LD_LIBRARY_PATH`：`export LD_LIBRARY_PATH=/path/to/lib:$LD_LIBRARY_PATH`
3. 或将路径写入 `/etc/ld.so.conf.d/` 并执行 `sudo ldconfig`

### Q5：`package.xml` 中声明了依赖但 `setup.py` 的 `install_requires` 中没有

这是 ROS 2 Python 包的常见模式。`package.xml` 中的依赖声明供 `colcon` 和 `rosdep` 使用，用于确定编译顺序和系统级依赖安装；而 `setup.py` 的 `install_requires` 供 `pip` 使用。在 ROS 2 工程中，Python 第三方依赖通常通过虚拟环境的 `requirements.txt` 统一管理，而非在每个包的 `setup.py` 中重复声明。

---

## 13. 依赖配置全景总结

### 13.1 配置文件与管理体系对照表

| 配置文件 | 管理体系 | 管理对象 | 作用域 |
|---------|---------|---------|--------|
| `requirements.txt` | pip | Python 第三方包 | 虚拟环境 |
| `package.xml` | ROS 2 ament | ROS 包间依赖、系统依赖声明 | ROS 2 工作空间 |
| `setup.py` + `setup.cfg` | setuptools / ament_python | Python 包的入口点、元信息 | 单个 ROS 2 Python 包 |
| `CMakeLists.txt` | CMake / ament_cmake | C++ 编译规则、库查找与链接 | 单个 ROS 2 C++ 包 |
| `bringup.sh` | Shell | 运行时环境变量配置 | 整个项目 |
| `.gitignore` | Git | 版本控制排除规则 | 整个仓库 |

### 13.2 依赖安装方式汇总

| 安装方式 | 适用对象 | 命令示例 |
|---------|---------|---------|
| `apt install` | 系统级 C/C++ 库、ROS 2 框架 | `sudo apt install libpcl-dev` |
| `pip install` | Python 第三方包 | `pip install -r requirements.txt` |
| 源码编译 (`cmake + make`) | 无预编译包的 C++ 库 | Livox-SDK2、TEASER++ |
| `colcon build` | ROS 2 工作空间内的所有包 | `colcon build --symlink-install` |

### 13.3 环境加载顺序

项目运行前需按以下顺序加载环境，顺序不可颠倒：

```
1. source /opt/ros/humble/setup.bash      ← ROS 2 基础环境
2. source /data/venv/radar-env/bin/activate  ← Python 虚拟环境
3. source install/setup.bash               ← 本工作空间编译产物
```

若顺序错误，可能导致 `PYTHONPATH` 被覆盖，使 ROS 2 的 Python 包或虚拟环境中的包无法被正确导入。

### 13.4 依赖管理最佳实践建议

1. 维护一份精简的核心依赖文件（如 `requirements.in`），仅列出项目直接依赖的包，再通过 `pip-compile` 生成完整的锁定文件 `requirements.txt`，以便区分直接依赖与传递性依赖。
2. 在 `package.xml` 中准确区分 `<build_depend>`、`<exec_depend>` 和 `<depend>`，避免引入不必要的编译期或运行期依赖。
3. 对于源码编译的 C++ 库（如 Livox-SDK2、TEASER++），建议在项目文档中明确记录其版本、编译参数和安装路径，以便团队成员复现环境。
4. 虚拟环境路径不应硬编码于脚本中。可通过环境变量（如 `RADAR_VENV_PATH`）进行参数化，提升可移植性。
5. 定期执行 `pip list --outdated` 和 `apt list --upgradable` 检查依赖更新，但升级前应在独立环境中充分测试兼容性。
6. 将 `.gitignore` 中的编译产物和缓存文件排除规则保持完整，避免将可再生文件纳入版本控制，减小仓库体积。

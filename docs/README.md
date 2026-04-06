# 📖 HNURM Radar 文档总览

本目录包含项目的所有文档。以下是按类别整理的索引。

---

## 入门指南

| 文档 | 说明 |
|------|------|
| [快速开始](guides/quickstart.md) | 新手 15 分钟上手，涵盖环境配置、三种方案启动 |
| [学习路线图](guides/LEARNING_ROADMAP.md) | 技术栈学习指南（ROS2、YOLO、点云等） |
| [空中方案指南](guides/AIR_SCHEME_GUIDE.md) | 空中方案详细说明 |
| [配置与路径指南](guides/CONFIG_AND_PATHS_GUIDE.md) | 配置文件与路径管理 |

## 系统架构

| 文档 | 说明 |
|------|------|
| [系统架构总览](architecture/system_overview.md) | 架构图、节点关系、数据流 |
| [空中目标集成](architecture/AIR_TARGET_INTEGRATION.md) | 空中目标检测子系统设计 |

## API 参考

| 文档 | 说明 |
|------|------|
| [ROS2 话题/服务清单](api/ros_topics.md) | 节点、话题、消息类型完整列表 |
| [配置文件参考](../configs/README.md) | 配置项说明 |
| [模型权重说明](../data/weights/README.md) | 模型文件说明与导出方式 |

## 开发文档

| 文档 | 说明 |
|------|------|
| [开发计划](OPTIMIZATION_PLAN.md) | 四阶段优化路线图（本项目核心规划文档） |
| [Ultralytics 修改记录](../src/hnurm_radar/ultralytics/PATCHES.md) | YOLO 框架的定制修改点清单 |

## 归档文档

> 以下文档为历史记录或临时分析，仅供参考。

| 文档 | 说明 |
|------|------|
| [X 轴漂移分析](archive/x_axis_drift_master_solution.md) | X 轴坐标漂移问题根因与解决方案 |
| [根因分析](archive/ROOT_CAUSE_ANALYSIS.md) | 历史问题根因分析 |
| [文档建议](archive/FINAL_DOCUMENT_SUGGESTIONS.md) | 文档改进建议（已部分落地） |
| [实验室测试计划](archive/LAB_TEST_PLAN.md) | 实验室联调测试计划 |
| [HAP 配置说明](archive/HAP_config_说明.txt) | Livox HAP 雷达配置说明 |

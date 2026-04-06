"""
communication — 裁判系统通信模块

从 shared/judge_messager.py (977行) 拆分而来，按职责分为：
  - serial_protocol : CRC 校验、帧头帧尾构造（底层协议）
  - referee_receiver: Receiver 独立进程（串口接收 + 协议解析）
  - judge_messager  : JudgeMessager ROS2 节点（坐标发送 + 双倍易伤）
  - _deprecated     : 归档的废弃代码（暂时保留，不参与运行）
"""

# serial_protocol 无 ROS 依赖，直接导入
from .serial_protocol import (
    CRC8_TABLE,
    CRC16_TABLE,
    get_crc8_check_byte,
    get_crc16_check_byte,
    get_frame_header,
    get_frame_tail,
)

# Receiver 和 JudgeMessager 依赖 ROS2 / detect_result 消息包，
# 仅在 colcon 构建环境中可用，采用延迟导入。
# 使用方式:
#   from hnurm_radar.communication.referee_receiver import Receiver
#   from hnurm_radar.communication.judge_messager import JudgeMessager, main

__all__ = [
    "CRC8_TABLE",
    "CRC16_TABLE",
    "get_crc8_check_byte",
    "get_crc16_check_byte",
    "get_frame_header",
    "get_frame_tail",
    # 以下需从子模块显式导入
    # "Receiver",
    # "JudgeMessager",
    # "main",
]

"""
judge_messager.py — 兼容垫片（shim）

原始 977 行文件已拆分至 communication/ 包：
  - communication/serial_protocol.py  — CRC 校验 + 帧头帧尾
  - communication/referee_receiver.py — Receiver 接收进程
  - communication/judge_messager.py   — JudgeMessager ROS2 节点
  - communication/_deprecated.py      — 废弃代码归档

本文件保留以维持向后兼容（setup.py 入口点、其他可能的 import 路径）。
所有符号均从 communication 包重导出。
"""

# serial_protocol 无 ROS 依赖，直接导入
from ..communication.serial_protocol import (
    CRC8_TABLE,
    CRC16_TABLE,
    get_crc8_check_byte,
    get_crc16_check_byte,
    get_frame_header,
    get_frame_tail,
)

# Receiver 和 JudgeMessager 依赖 ROS2 消息包 (detect_result)，
# 仅在 colcon 构建环境中可用。
# setup.py 入口点 "judge_messager = hnurm_radar.shared.judge_messager:main"
# 在 ROS2 运行时通过此垫片的 main() 函数启动。


def main(args=None):
    """ROS2 入口点垫片 — 委托给 communication.judge_messager.main()"""
    from ..communication.judge_messager import main as _main
    _main(args)


__all__ = [
    "CRC8_TABLE",
    "CRC16_TABLE",
    "get_crc8_check_byte",
    "get_crc16_check_byte",
    "get_frame_header",
    "get_frame_tail",
    "main",
]

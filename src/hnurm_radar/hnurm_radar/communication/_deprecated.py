"""
_deprecated.py — 从 judge_messager.py 拆分出的废弃代码归档

此文件中的代码不参与运行，仅作为历史参考保留。
用户明确要求 "废弃的代码不要删除，暂时留着"。

内容：
  1. Tools 类 — 帧率控制工具（与 Tools/Tools.py 完全重复）
  2. parse_frame() — 模块级函数但使用 self 参数，无法正常调用，
     疑为早期调试遗留的死代码

原始来源: shared/judge_messager.py
"""

import time
import struct


# ══════════════════════════════════════════════════════════════
# DEPRECATED: Tools 类
# 与 hnurm_radar/Tools/Tools.py 完全相同，系复制粘贴产物。
# 现在 Receiver.parse_cmd_id() (DEPRECATED方法) 中的引用已改为
# 从 Tools.Tools 导入。
# 原始位置: judge_messager.py 第 39-73 行
# ══════════════════════════════════════════════════════════════

class Tools:
    @staticmethod
    def get_time_stamp():
        return time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())

    @staticmethod
    # 帧率控制，传入帧率和上一帧的时间戳，自动进行sleep
    def frame_control_sleep(fps, last_time):
        '''

        :param fps: 期望控制帧率
        :param last_time: 上次执行的时间戳
        :return: 本次执行的时间戳，用于下次调用，用例：last_time = frame_control(fps, last_time)
        '''
        current_time = time.time()
        sleep_time = 1 / fps - (current_time - last_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
        return time.time()  # 返回当前时间戳 , last_time = frame_control(fps, last_time)

    @staticmethod
    def frame_control_skip(fps, last_time):
        '''

        :param fps: 期望控制帧率
        :param last_time: 上次执行的时间戳
        :return: 是否跳过本次执行，用例：skip, last_time = frame_control(fps, last_time)
        if skip:
            continue
        '''
        if time.time() - last_time < 1 / fps:
            return True, last_time
        else:
            # 传入的last_time是一个不可变对象（如整数，字符串，元组），所以是副本，不会改变原来的值，需要返回新的时间戳
            return False, time.time()


# ══════════════════════════════════════════════════════════════
# DEPRECATED: parse_frame() 孤立函数
# 使用 self 参数但不是任何类的方法，无法正常调用。
# 疑为早期调试遗留代码，专门解析 0x0305 (cmd_id=773) 回传。
# 原始位置: judge_messager.py 第 789-813 行
# ══════════════════════════════════════════════════════════════

def parse_frame(self, serial_port):
    # 找到SOF
    if not self.find_sof(serial_port):
        return False

    # 读取帧头（SOF之后的4字节）
    header = serial_port.read(4)
    # print(header)
    data_length, seq, crc8 = struct.unpack('<HBB', header)

    # 根据data_length读取data和frame_tail
    data_and_tail = serial_port.read(2 + data_length + 2)  # 包括命令码和CRC16

    # 解析出命令码和数据内容
    cmd_id = struct.unpack('H', data_and_tail[:2])
    if cmd_id[0] == 773:
        data = data_and_tail[2:-2]

        carid = struct.unpack('H', data[:2])
        x = struct.unpack('f', data[2:6])
        y = struct.unpack('f', data[6:])
        print("carId:", carid, "x:", x, "y:", y)

    return True

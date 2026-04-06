"""
referee_receiver.py — 裁判系统串口接收器（独立进程）

以 multiprocessing.Process 方式运行，阻塞式读取串口数据，
通过共享内存（Value/Array）将解析结果传递给主进程。

接收的 cmd_id：
  - 0x0001: 比赛状态（剩余时间）
  - 0x0003: 敌方机器人血量
  - 0x020C: 标记进度
  - 0x020E: 双倍易伤状态
  - 0x0105: 飞镖目标

拆分来源: shared/judge_messager.py 第 197-783 行
"""

import time
import struct
import serial
from multiprocessing import Value, Process

from .serial_protocol import (
    get_crc8_check_byte,
    get_crc16_check_byte,
    check_crc8,
    check_crc16,
)


class Receiver:
    """裁判系统串口接收器，运行在独立子进程中。"""

    def __init__(
        self,
        my_color,
        shared_is_activating_double_effect,
        shared_enemy_health_list,
        shared_enemy_marked_process_list,
        shared_have_double_effect_times,
        shared_time_left,
        shared_dart_target,
        serial_port='/dev/ttyACM0',
        baud_rate=115200,
    ):
        # 共享内存变量
        try:
            self.shared_is_activating_double_effect = shared_is_activating_double_effect
            self.shared_enemy_health_list = shared_enemy_health_list
            self.shared_enemy_marked_process_list = shared_enemy_marked_process_list
            self.shared_have_double_effect_times = shared_have_double_effect_times
            self.shared_time_left = shared_time_left
            self.shared_dart_target = shared_dart_target

            # 全局变量
            self.my_color = my_color

        except Exception as e:
            print(f"shared init fail {e}")

        self.ser = serial.Serial(serial_port, baud_rate, timeout=1)  # 串口初始化
        self.fps = 100  # 控制主线程帧率为100Hz

        # 数据存储
        self.time_left = -1  # 剩余时间
        # 共享内存变量
        self.already_send_double_effect = Value('i', 0)  # 是否已经发送了双倍概率
        self.dart_target = 0  # 飞镖目标

        # 线程
        self.working_flag = False
        self.last_time_main_loop = time.time()  # 保持一秒一帧

        # 接收进程
        self.process = Process(target=self.parse_cmd_id_batch, daemon=True)

    # ── 进程控制 ─────────────────────────────────────────────

    def start(self):
        """启动接收进程"""
        self.working_flag = True
        self.process.start()

    def stop(self):
        """停止接收进程"""
        if self.working_flag:
            self.working_flag = False
            print("receiver stop")
            self.process.terminate()
            self.process.join()
            self.ser.close()
            print(f"working status {self.working_flag}")

    # ── CRC 校验（委托给 serial_protocol） ────────────────────

    def _check_crc8(self, tx_buff, crc8):
        """判断 CRC8 是否正确"""
        return check_crc8(tx_buff, crc8)

    def _check_crc16(self, tx_buff, crc16_bytes):
        """判断 CRC16 是否正确"""
        return check_crc16(tx_buff, crc16_bytes)

    # ── 帧头构造辅助 ────────────────────────────────────────

    def get_tx_buff_to_cal_crc16(self, header, cmd_id, data):
        """将帧头、cmd_id、data 合成用来计算 CRC16 的数据"""
        return header + struct.pack('H', cmd_id) + data

    # ── 帧解析核心 ──────────────────────────────────────────

    def find_sof(self):
        """读取单个字节直至找到 SOF (0xA5)"""
        while True:
            if not self.working_flag:
                return
            try:
                self.ser.timeout = 0.01
                byte = self.ser.read(1)
            except serial.SerialTimeoutException as e:
                print("serial read timeout")
                print(f"time out as {e}")
            if byte == b'\xA5':
                return True
            time.sleep(0.001)
        return False

    def parse_frame_header(self):
        """
        解析帧头，返回 (data_length, crc8_valid, full_header)。
        
        找到 SOF 后读取后续 4 字节，校验 CRC8。
        """
        if not self.find_sof():
            print("not")
            return False

        header = self.ser.read(4)
        data_length, seq, crc8 = struct.unpack('<HBB', header)
        full_header = struct.pack('B', 165) + header

        _header = struct.pack('B', 165) + struct.pack('H', data_length) + struct.pack('B', seq)
        if self._check_crc8(_header, crc8):
            return data_length, True, full_header
        else:
            return -1, False, full_header

    def read_cmd_id(self):
        """读取 cmd_id (2 字节)"""
        cmd_id = self.ser.read(2)
        return struct.unpack('H', cmd_id)[0]

    def read_data(self, data_length):
        """读取 data 段"""
        data = self.ser.read(data_length)
        return data

    def read_frame_tail(self):
        """读取帧尾 CRC16 (2 字节)"""
        frame_tail = self.ser.read(2)
        return frame_tail

    # ── 剩余时间读取 ────────────────────────────────────────

    def read_remaining_time(self):
        """读取剩余比赛时间，返回 (is_valid, remaining_time)"""
        data_length, is_valid, header = self.parse_frame_header()
        if not is_valid:
            print("Frame header CRC8 check failed")
            return False, 0

        cmd_id = self.read_cmd_id()

        if cmd_id == 0x0001:
            remaining_time_data = self.read_data(data_length + 2)
            remaining_time = remaining_time_data[1] + remaining_time_data[2] * 256
            self.read_frame_tail()
            return True, remaining_time
        else:
            return False

    # ── DEPRECATED: 逐帧解析（未使用，保留备查） ──────────────

    def parse_receiver_data(self):  # DEPRECATED: 被 parse_cmd_id_batch 取代
        """
        DEPRECATED — 单帧逐个解析模式。
        已被 parse_cmd_id_batch() 替代，保留仅供参考。
        """
        while True:
            if not self.working_flag:
                print("not")
                return
            time_interval = time.time() - self.last_time_main_loop
            if time_interval < 0.02:
                time.sleep(0.02 - (time_interval))
            self.last_time_main_loop = time.time()

            data_length, is_valid, header = self.parse_frame_header()
            print("receiver one ")
            if not is_valid:
                print("Frame header CRC8 check failed")
                continue

            rest_data = self.ser.read(2 + data_length + 2)

            tx_buff = header + rest_data[:-2]

            frame_tail_ori = rest_data[-2:]

            if not self._check_crc16(tx_buff, frame_tail_ori):
                print("CRC16 check failed")
                continue

            cmd_id = rest_data[:2]
            data = rest_data[2:-2]

            self.switch_method(cmd_id, data)

    def log_buffer_content(self, buffer):
        """存 log（预留接口）"""
        pass

    # ── 批量解析（实际使用的入口） ────────────────────────────

    def parse_cmd_id_batch(self):
        """
        批量解析串口缓冲区中的所有帧。
        
        作为子进程的 target 函数运行（见 __init__ 中 Process 定义）。
        一次性读取缓冲区全部数据，循环解析每一帧。
        """
        buffer = b''

        while True:
            if not self.working_flag:
                print("receiver process exit")
                return

            buffer += self.ser.read(self.ser.in_waiting or 1)
            try:
                self.log_buffer_content(buffer)
            except Exception as e:
                print(f"buffer save Error: {e}")
            while True:
                if not self.working_flag:
                    print("receiver process exit")
                    return
                try:
                    if len(buffer) < 5:
                        break

                    sof_index = buffer.find(b'\xA5')
                    if sof_index == -1:
                        buffer = b''
                        print("no xa5")
                        break

                    if len(buffer) < sof_index + 5:
                        break

                    header = buffer[sof_index:sof_index + 5]
                    data_length, seq, crc8 = struct.unpack('<HBB', header[1:])

                    _header = struct.pack('B', 165) + struct.pack('H', data_length) + struct.pack('B', seq)
                    if not self._check_crc8(_header, crc8):
                        buffer = buffer[sof_index + 1:]
                        print("crc failed")
                        continue

                    frame_length = 5 + 2 + data_length + 2
                    if len(buffer) < sof_index + frame_length:
                        break

                    frame = buffer[sof_index:sof_index + frame_length]
                    buffer = buffer[sof_index + frame_length:]

                    header = frame[:5]
                    rest_data = frame[5:]
                    cmd_id = rest_data[:2]
                    data = rest_data[2:-2]
                    frame_tail = rest_data[-2:]

                    tx_buff = header + rest_data[:-2]
                    if not self._check_crc16(tx_buff, frame_tail):
                        print("crc16 check fail")
                        print("crc16 check fail")

                    self.switch_method(cmd_id, data)
                except Exception as e:
                    print(f"Exception: {e}")

    # ── DEPRECATED: 带帧率控制的逐帧解析（未使用，保留备查） ──

    def parse_cmd_id(self):  # DEPRECATED: 被 parse_cmd_id_batch 取代
        """
        DEPRECATED — 带帧率控制的逐帧解析模式。
        已被 parse_cmd_id_batch() 替代，保留仅供参考。
        """
        from ..Tools.Tools import Tools

        while True:
            if not self.working_flag:
                print("not")
                return

            self.last_time_main_loop = Tools.frame_control_sleep(1000, self.last_time_main_loop)

            data_length, is_valid, header = self.parse_frame_header()
            if not is_valid:
                print("Frame header CRC8 check failed")
                continue

            rest_data = self.ser.read(2 + data_length + 2)

            tx_buff = header + rest_data[:-2]

            frame_tail_ori = rest_data[-2:]

            if not self._check_crc16(tx_buff, frame_tail_ori):
                print("CRC16 check failed")
                continue

            cmd_id = rest_data[:2]
            data = rest_data[2:-2]

            self.switch_method(cmd_id, data)

    # ── 命令分发 ────────────────────────────────────────────

    def switch_method(self, cmd_id, data):
        """根据 cmd_id 分发到对应的解析方法"""
        try:
            cmd_id_value = struct.unpack('<H', cmd_id)[0]
        except Exception as e:
            print(f"Error: {e}")
            print(f"Error in switch method: {e}")
        try:
            if cmd_id_value == 0x0001:  # 比赛进行时间解析
                self.process_game_status(data)
            elif cmd_id_value == 0x0003:
                self.parse_robot_status(data)
            elif cmd_id_value == 0x020C:
                self.parse_mark_process(data)
            elif cmd_id_value == 0x020E:
                self.parse_double_effect(data)
            elif cmd_id_value == 0x0105:  # 飞镖目标
                self.parse_dart_target(data)
        except Exception as e:
            print(f"Error: {e}")
            print(f"Error in switch method: {e}")

    # ── 各 cmd_id 解析方法 ──────────────────────────────────

    def parse_dart_target(self, data):
        """
        解析飞镖目标 (cmd_id=0x0105)。
        
        bit 5-6: 飞镖此时选定的击打目标
            0 = 未选定/前哨站, 1 = 基地固定目标, 2 = 基地随机目标
        """
        dart_info_value = struct.unpack('<H', data[1:3])[0]
        dart_target = (dart_info_value >> 5) & 0x03
        self.shared_dart_target.value = dart_target

    def process_game_status(self, data):
        """解析比赛状态 (cmd_id=0x0001)，提取剩余时间"""
        time_left = data[1] + data[2] * 256
        self.shared_time_left.value = time_left
        print(f"Time left: {time_left}")

    def get_time_left(self):
        """获取比赛剩余时间"""
        return self.time_left

    def parse_robot_status(self, data):
        """
        解析敌方机器人血量 (cmd_id=0x0003)。
        
        数据结构 game_robot_HP_t:
            red_1 ~ red_5, red_7, red_outpost, red_base,
            blue_1 ~ blue_5, blue_7, blue_outpost, blue_base
            各 uint16_t
        
        :return: 6 元素列表 [英雄HP, 工程HP, 步兵3HP, 步兵4HP, 步兵5HP, 哨兵HP]
        """
        if self.my_color == 'Red':
            enemy_hp = [data[2 * i + 8] + data[2 * i + 9] * 256 for i in range(6)]
        else:
            enemy_hp = [data[2 * i] + data[2 * i + 1] * 256 for i in range(6)]

        for i in range(6):
            self.shared_enemy_health_list[i] = enemy_hp[i]

        return enemy_hp

    def parse_mark_process(self, data):
        """
        解析标记进度 (cmd_id=0x020C)。
        
        数据结构 radar_mark_data_t:
            mark_hero_progress, mark_engineer_progress,
            mark_standard_3_progress, mark_standard_4_progress,
            mark_standard_5_progress, mark_sentry_progress
            各 uint8_t (0-120)
        
        :return: 6 元素列表 [英雄标记, 工程标记, 步兵3标记, 步兵4标记, 步兵5标记, 哨兵标记]
        """
        mark_process = [data[i] for i in range(6)]
        for i in range(6):
            self.shared_enemy_marked_process_list[i] = mark_process[i]

        print(f"Mark process: {mark_process}")

        return mark_process

    def parse_double_effect(self, data):
        """
        解析双倍易伤信息 (cmd_id=0x020E)。
        
        data[0]:
            bit 0-1: 双倍易伤机会次数
            bit 2:   双倍易伤是否激活
        
        :return: (double_effect_chance, is_double_effect_active)
        """
        radar_info = data[0]

        double_effect_chance = radar_info & 0x03
        is_double_effect_active = (radar_info >> 2) & 0x01

        self.shared_is_activating_double_effect.value = is_double_effect_active
        self.shared_have_double_effect_times.value = double_effect_chance

        print(f"Double effect chance: {double_effect_chance}, is double effect active: {is_double_effect_active}")

        return double_effect_chance, is_double_effect_active

    def parse_0x0305(self):
        """
        解析 0x0305 回传数据（调试用）。
        
        注意：此方法在正常运行中未被调用，仅用于调试验证。
        """
        data_length, is_valid = self.parse_frame_header()

        if not is_valid:
            return False

        cmd_id = self.read_cmd_id()
        if cmd_id[0] != 773:
            return False

        data = self.read_data(data_length)
        carid = struct.unpack('H', data[:2])
        x = struct.unpack('f', data[2:6])
        y = struct.unpack('f', data[6:])
        print("carId:", carid, "x:", x, "y:", y)

        frame_tail = self.read_frame_tail()

        crc16 = struct.unpack('H', frame_tail)
        tx_buff = struct.pack('H', carid) + struct.pack('ff', x, y)

        return True

    def close(self):
        """关闭串口"""
        self.ser.close()

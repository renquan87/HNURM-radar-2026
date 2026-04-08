"""
judge_messager.py — 裁判系统 ROS2 通信节点

功能：
  - 订阅 /ekf_location_filtered 话题获取滤波后的敌方机器人坐标
  - 以 ~150ms 周期通过串口向裁判系统发送敌方坐标（cmd_id=0x0305）
  - 发送双倍易伤触发指令（cmd_id=0x0301，子命令 0x0121）
  - 启动 Receiver 子进程接收裁判系统下行数据

入口：ros2 run hnurm_radar judge_messager
配置：configs/main_config.yaml → global.my_color

拆分来源: shared/judge_messager.py 第 821-976 行
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from detect_result.msg import Location, Locations
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import threading
import time
import struct
import serial
from ruamel.yaml import YAML

import multiprocessing

from .serial_protocol import get_frame_header, get_frame_tail
from .referee_receiver import Receiver
from ..shared.paths import FIELD_WIDTH, FIELD_HEIGHT


class JudgeMessager(Node):
    def __init__(self):
        super().__init__('judge_messager')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=3
        )
        # 读取配置信息
        from ..shared.paths import MAIN_CONFIG_PATH
        self.cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='Utf-8', mode='r'))
        self.my_color = self.cfg['global']['my_color']
        self.get_logger().info(f"my color is {self.my_color}")
        self.serial_lock = threading.Lock()
        if self.my_color == 'Red':
            self.my_id = 9
        else:
            self.my_id = 109
        # 创建共享内存变量
        self.shared_is_activating_double_effect = multiprocessing.Value('b', False)  # 共享内存，用于多进程
        self.shared_enemy_health_list = multiprocessing.Array('i', [100, 100, 100, 100, 100, 100])  # 对方1-5和7号的血量信息
        self.shared_enemy_marked_process_list = multiprocessing.Array('i', [0, 0, 0, 0, 0, 0, 0])  # 标记进度
        self.shared_have_double_effect_times = multiprocessing.Value('i', 0)  # 拥有的双倍易伤次数
        self.shared_time_left = multiprocessing.Value('i', -1)  # 剩余时间
        self.shared_dart_target = multiprocessing.Value('i', 0)  # 飞镖目标
        self.dart_target_times = [0, 0, 0]

        # 从配置读取串口参数（消除硬编码）
        comm_cfg = self.cfg.get('communication', {})
        serial_port = comm_cfg.get('port', '/dev/ttyACM0')
        baud_rate = int(comm_cfg.get('bps', 115200))
        serial_timeout = float(comm_cfg.get('timex', 0.01))

        self.receiver = Receiver(
            self.my_color,
            self.shared_is_activating_double_effect,
            self.shared_enemy_health_list,
            self.shared_enemy_marked_process_list,
            self.shared_have_double_effect_times,
            self.shared_time_left,
            self.shared_dart_target,
            serial_port=serial_port,
            baud_rate=baud_rate,
        )
        self.receiver.start()

        # 打开串口（发送用，与 Receiver 共享同一物理设备）
        self.get_logger().info(f'Trying to Open Serial port: {serial_port} @ {baud_rate}')
        self.ser = serial.Serial(serial_port, baud_rate, timeout=serial_timeout)
        if not self.ser.is_open:
            self.ser.open()
        else:
            self.get_logger().info('Serial port is already open')
        # 订阅location话题，不断更新检测到机器人位置
        self.locations = Locations()
        self.location_lock = threading.Lock()
        self.sub_location = self.create_subscription(Locations, "ekf_location_filtered", self.location_callback, qos_profile)

        # 双倍易伤计数
        self.used_double_effect = 0

        # 在创建的线程中处理location
        self.judger = threading.Thread(target=self.judge_loop)
        self.judger.start()

    # ── 发送方法 ────────────────────────────────────────────

    def send_map_robot_location(self, robot_location):
        """
        使用 0x0305 向裁判系统发送敌方机器人定位信息。
        
        robot_location: 6×[x,y] 的列表
        顺序：英雄(1号)、工程(2号)、步兵3(3号)、步兵4(4号)、步兵5(5号)、哨兵(7号)
        """
        cmd_id = struct.pack('H', 0x0305)
        data = b''
        for loc in robot_location:
            x = int(loc[0] * 100)
            y = int(loc[1] * 100)
            data += struct.pack('HH', x, y)  # 单位转换为cm
        data_len = len(data)
        frame_head = get_frame_header(data_len)
        tx_buff = frame_head + cmd_id + data
        frame_tail = get_frame_tail(tx_buff)
        tx_buff += frame_tail
        self.ser.write(tx_buff)

    def send_double_effect(self):
        """
        发送双倍易伤触发指令 (cmd_id=0x0301, 子命令 0x0121)。
        """
        cmd_id = struct.pack('H', 0x0301)
        data_cmd_id = struct.pack('H', 0x0121)
        sender_id = struct.pack('H', self.my_id)
        receiver_id = struct.pack('H', 0x8080)
        times_data = struct.pack('H', self.used_double_effect + 1)
        data = data_cmd_id + sender_id + receiver_id + times_data

        data_len = len(data)
        frame_head = get_frame_header(data_len)

        tx_buff = frame_head + cmd_id + data

        frame_tail = get_frame_tail(tx_buff)

        tx_buff += frame_tail
        self.ser.write(tx_buff)

    # ── 主循环 ──────────────────────────────────────────────

    def judge_loop(self):
        """主循环，不断发送敌方机器人位置和双倍易伤信息"""
        # ID 到发送数组下标的映射：英雄(0),工程(1),步兵3(2),步兵4(3),步兵5(4),哨兵(5)
        robot_trans = {101: 0, 102: 1, 103: 2, 104: 3, 105: 4, 107: 5,
                       1: 0, 2: 1, 3: 2, 4: 3, 5: 4, 7: 5}
        while rclpy.ok():
            cur_locations = Locations()
            with self.location_lock:
                cur_locations = self.locations

            robot_loc = [[0, 0], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0]]

            for i in range(len(cur_locations.locs)):
                location = cur_locations.locs[i]
                # 跳过 NULL 等无效 ID，避免 KeyError
                if location.id not in robot_trans:
                    continue
                x = round(location.x, 2)
                y = round(location.y, 2)
                z = round(location.z, 2)
                # 将坐标限制在赛场内
                x = max(0, min(FIELD_WIDTH, x))
                y = max(0, min(FIELD_HEIGHT, y))

                # 配准初始位姿已根据 my_color 正确设置，坐标直接就是裁判系统坐标，
                # 无需根据 label 做坐标翻转。
                robot_loc[robot_trans[location.id]] = [x, y]

            self.send_map_robot_location(robot_loc)

            # 第一次使用后，迭代使用次数
            if self.receiver.shared_is_activating_double_effect.value == 1 and self.used_double_effect == 0:
                self.used_double_effect += 1
            if self.receiver.shared_time_left == 5:
                # 当比赛快要结束时，清空
                self.used_double_effect = 0
            self.send_double_effect()
            time.sleep(0.15)

    def location_callback(self, msg):
        """更新 locations"""
        with self.location_lock:
            self.locations = msg


def main(args=None):
    rclpy.init(args=args)
    node = JudgeMessager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from detect_result.msg import Location, Locations
from detect_result.msg import DetectResult
from detect_result.msg import Robots
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from ekf.RobotEKF import RobotEKF

# 机器人信息类
class RobotInfo(object):

    def __init__(self, time, x, y, z):
        self.x, self.y, self.z = x, y, z
        self.time = time
        self.v_x = 0
        self.v_y = 0
        self.last_v_x = 0 # 上一次的速度
        self.last_v_y = 0 # 上一次的速度
        self.a_x = 0   # V1 = V0 + at   -> a = (v1-v0)/t
        self.a_y = 0

    def __str__(self):
        return '%4d %4d %4d %4d' % (self.time, self.x, self.y, self.z)
    
    # 计算速度和加速度
    def calculateInfo(self, last_info):

        if last_info.x > 0 and last_info.y > 0:
            t = (self.time - last_info.time) # 转s

            self.v_x = (self.x - last_info.x) / t
            self.v_y = (self.y - last_info.y) / t

            self.a_x = (self.v_x - last_info.v_x) / t
            self.a_y = (self.v_y - last_info.v_y) / t





# EKF节点类
class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')
        
        # QoS 设置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 记录敌方颜色
        self.eneny_color = 'NULL'
        # 机器人ID映射到数组下标
        self.transform_to_th = {
            1: 1,
            2: 2,
            3: 3,
            4: 4,
            5: 5,
            7: 6,
            101: 1,
            102: 2,
            103: 3,
            104: 4,
            105: 5,
            107: 6
        }
        # 初始化6个机器人的EKF滤波器，修改噪声参数q，r减小突变对数据的影响
        self.kalfilt = [
            RobotEKF(4, 4, pval=0.001, qval=1e-4, rval=0.0005, interval=0.05) for _ in range(6)
        ]

        # 创建发布者和订阅者
        self.pub_location_filtered = self.create_publisher(Locations, 'ekf_location_filtered', qos_profile)
        self.sub_location = self.create_subscription(Locations, "location", self.location_callback, qos_profile)
        # 初始化变量
        self.recv_location = Locations()
        self.last_location = Locations()
        # 创建定时器
        self.timer = self.create_timer(0.05, self.timer_callback)  # 50 ms
        
        # 初始化位置队列，保存上次和当前的机器人位置信息 [0]是上次的， [1]是当前的
        self.locations_queue = [[RobotInfo(0,0,0, 0), RobotInfo(0,0,0,0), RobotInfo(0,0,0,0), RobotInfo(0,0,0,0),RobotInfo(0,0,0,0),RobotInfo(0,0,0,0)] for i in range(2)]

    # 获取当前时间的毫秒数    
    def get_current_time_ms(self):
        current_time = self.get_clock().now()
        time_msg = current_time.to_msg()
        milliseconds = time_msg.sec * 1000 + time_msg.nanosec / 1e6
        return milliseconds

    # 定时器回调函数    
    def timer_callback(self):
        self.get_logger().info(f"Timestamp: {self.get_clock().now()}, Location: {self.recv_location.__str__()}")
        # self.get_logger().info(f"Timestamp: {self.get_current_time_ms()}, Location: {self.recv_location.__str__()}")
        locations = self.recv_location
        # 更新测量值
        for i in range(len(locations.locs)):
            location = locations.locs[i]
            robot_id = location.id
            x = location.x
            y = location.y
            time = self.get_current_time_ms()
            if self.eneny_color == "NULL":
                self.eneny_color = location.label
            self.locations_queue[1][self.transform_to_th[robot_id] - 1].x = x
            self.locations_queue[1][self.transform_to_th[robot_id] - 1].y = y
            self.locations_queue[1][self.transform_to_th[robot_id] - 1].time = time
            self.locations_queue[1][self.transform_to_th[robot_id] - 1].z = location.z
        estimated_locations = [[0,0,0,0], [0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]
        # 对每个机器人进行卡尔曼滤波
        for i in range(len(self.locations_queue[0])):
            # 如果没有新的数据,则跳过
            if self.locations_queue[1][i].time == 0:
                continue
            # 计算加速度
            self.locations_queue[1][i].calculateInfo(self.locations_queue[0][i])
            self.kalfilt[i].update_acceleration(self.locations_queue[1][i].a_x, self.locations_queue[1][i].a_y)
            estimated_locations[i] = (self.kalfilt[i].step((self.locations_queue[1][i].x, self.locations_queue[1][i].v_x, self.locations_queue[1][i].y, self.locations_queue[1][i].v_y)))
            
        # 保存历史位置
        for i in range(len(self.locations_queue[0])):
            if self.locations_queue[0][i].time == 0:
                self.locations_queue[0][i].time = self.locations_queue[1][i].time
                self.locations_queue[0][i].x = self.locations_queue[1][i].x
                self.locations_queue[0][i].y = self.locations_queue[1][i].y
                self.locations_queue[0][i].z = self.locations_queue[1][i].z
        self.get_logger().info(f"Estimated locations: {estimated_locations}")
        
        # 发布滤波后的位置信息
        locations = Locations()
        for i in range(len(estimated_locations)):
            location = Location()
            if estimated_locations[i][0] == 0 or estimated_locations[i][2] == 0:
                continue
            if self.eneny_color == 'Blue':
                location.id = i + 101
                if location.id == 106:
                    location.id = 107
            elif self.eneny_color == 'Red':
                location.id = i + 1
                if location.id == 6:
                    location.id = 7
            location.x = float(estimated_locations[i][0])
            location.y = float(estimated_locations[i][2])
            location.z = float(self.locations_queue[1][i].z)
            location.label = self.eneny_color
            locations.locs.append(location)
        # 通过话题发布
        self.pub_location_filtered.publish(locations)
            
                
    # 订阅位置信息的回调函数
    def location_callback(self, msg):
        # self.get_logger().info(f"Received location: {msg}")
        self.recv_location = msg
        
    

def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
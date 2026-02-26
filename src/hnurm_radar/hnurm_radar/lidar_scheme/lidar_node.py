"""
lidar_node.py — 激光雷达点云接收与预处理节点（方案二）

功能：
  订阅 Livox HAP 激光雷达驱动发布的 /livox/lidar 话题（PointCloud2），
  对原始点云进行距离滤波（近距离 + 远距离剔除），将多帧点云累积到
  PcdQueue 队列中，并以 ~100Hz 频率重新发布合并后的点云到 /lidar_pcds 话题，
  供下游 registration 节点进行点云配准。

数据流：
  /livox/lidar (PointCloud2) → listener_callback() → PcdQueue(累积10帧)
                                                        ↓
  publish_point_cloud() → /lidar_pcds (PointCloud2) → registration 节点

核心类：
  - Pcd:           单帧点云的 Open3D 封装
  - PcdQueue:      固定长度的点云队列，自动合并所有帧为一个 numpy 数组
  - LidarListener: ROS2 节点，订阅点云 → 滤波 → 入队 → 发布

配置参数（来自 configs/main_config.yaml → lidar 段）：
  - height_threshold: 地面点高度阈值（当前未启用）
  - min_distance:     近距离滤除阈值（m），默认 1
  - max_distance:     远距离滤除阈值（m），默认 40
  - lidar_topic_name: 点云话题名，默认 "/livox/lidar"
"""

import threading
from std_msgs.msg import String
import rclpy
from std_msgs.msg import Header
from rclpy.node import Node
import open3d as o3d
import numpy as np
from collections import deque
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import time
from ruamel.yaml import YAML
from ..shared.paths import MAIN_CONFIG_PATH

# 定义一个Pcd类，用于存储点云数据
class Pcd:
    def __init__(self, pcd_name=""):
        self.pcd = o3d.geometry.PointCloud()
        self.pcd_name = pcd_name

    # 将本Pcd的pcd属性设置为传入的pcd，修改引用
    def set_pcd(self, pcd):
        self.pcd = pcd

    # 更新pcd属性的points
    def update_pcd_points(self, pc):
        self.pcd.points = o3d.utility.Vector3dVector(pc)

    #


class PcdQueue(object):
    def __init__(self, max_size=90, voxel_size=0.05):
        self.max_size = max_size  # 传入最大次数
        self.queue = deque(maxlen=max_size)  # 存放的是pc类型
        self.pc_all = []  # 用于存储所有点云的numpy数组
        # 内部属性
        self.record_times = 0  # 已存点云的次数
        self.point_num = 0  # 已存点的数量

    # 添加一个点云
    def add(self, pc):  # 传入的是pc
        self.queue.append(pc)  # 传入的是点云，一份点云占deque的一个位置
        # print("append")
        if self.record_times < self.max_size:
            self.record_times += 1

        self.update_pc_all()
        self.point_num = self.cal_point_num()

    def get_all_pc(self):
        return self.pc_all

    # 把每个queue里的pc:[[x1,y1,z1],[x2,y2,x2],...]的点云合并到一个numpy数组中
    def update_pc_all(self):

        self.pc_all = np.vstack(self.queue)

    def is_full(self):
        return self.record_times == self.max_size

    # 获得队列中点的数量，而非队列的大小
    def cal_point_num(self):
        return len(self.pc_all)




class LidarListener(Node):
    def __init__(self, cfg):
        super().__init__('lidar_listener')
        qos__lidar_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 参数
        self.height_threshold = cfg["lidar"]["height_threshold"]  # 自身高度，用于去除地面点云
        self.min_distance = cfg["lidar"]["min_distance"]  # 最近距离，距离小于这个范围的不要
        self.max_distance = cfg["lidar"]["max_distance"]  # 最远距离，距离大于这个范围的不要
        self.lidar_topic_name = cfg["lidar"]["lidar_topic_name"] # 激光雷达话题名
        
        # 点云队列
        self.pcdQueue = PcdQueue(max_size=10) # 将激光雷达接收的点云存入点云队列中，读写上锁？
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        # 订阅/livox/lidar topic
        self.sub_livox = self.create_subscription(PointCloud2, self.lidar_topic_name, self.listener_callback, 10)
        # 发布点云话题
        self.pub_pcds = self.create_publisher(PointCloud2, "lidar_pcds", qos__lidar_profile)
        
        # 创建并启动发布线程
        self.publish_thread = threading.Thread(target=self.publish_point_cloud)
        self.publish_thread.start()
        
    def publish_point_cloud(self):
        # 统计发布频率
        last_time = time.time()
        while rclpy.ok():
            cur_time = time.time()
            delta_time = cur_time - last_time
            last_time = cur_time
            time.sleep(0.01)
            # self.get_logger().info("Publishing point cloud, frequency: {:.2f} Hz".format(1 / delta_time))
            points = self.get_all_pc()
            if len(points) > 0:
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "livox"
                # 以fileds x,y,z打包
                fields = [
                    pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                ]
                pc = pc2.create_cloud(header, fields, points)
                self.pub_pcds.publish(pc)
                

    def listener_callback(self,data):

        
        points = pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(
                list(points),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        
        # 将结构化数组转换为普通的float64数组
        points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(
            np.float64
        )
        # 过滤点云
        dist = np.linalg.norm(points, axis=1)  # 计算点云距离

        points = points[dist > self.min_distance]  # 雷达近距离滤除
        # 第二次需要重新计算距离，否则会出现维度不匹配
        dist = np.linalg.norm(points, axis=1)  # 计算点云距离
        points = points[dist < self.max_distance]  # 雷达远距离滤除

        # 如果在地面+5cm以上，才保留，在地面的点为-height_threshold,
        # pc = pc[pc[:, 2] > (-1 * self.height_threshold)]

        self.pcdQueue.add(points)
        

    # 获取所有点云
    def get_all_pc(self):
        return self.pcdQueue.get_all_pc()




def main(args=None):
    main_cfg = YAML().load(open(MAIN_CONFIG_PATH, encoding='Utf-8', mode='r'))
    rclpy.init(args=args)
    lidar = LidarListener(main_cfg)
    rclpy.spin(lidar)
    lidar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

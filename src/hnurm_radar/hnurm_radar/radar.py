import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import time
from sensor_msgs.msg import Image
import open3d as o3d
from .Lidar.Converter import Converter , ROISelector
import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from collections import deque
from ruamel.yaml import YAML
import os
from detect_result.msg import DetectResult
from detect_result.msg import Robots
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
class Radar(Node):

    def __init__(self):
        super().__init__('radar_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.bridge = CvBridge()
        detector_config_path = "./configs/detector_config.yaml"
        binocular_camera_cfg_path = "./configs/bin_cam_config.yaml"
        main_config_path = "./configs/main_config.yaml"
        converter_config_path = "./configs/converter_config.yaml"
        main_cfg = YAML().load(open(main_config_path, encoding='Utf-8', mode='r'))
        
        # 全局变量
        global_my_color = main_cfg['global']['my_color']
        is_debug = main_cfg['global']['is_debug']
        
        today = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) # 今日日期，例如2024年5月6日则为20240506
        self.converter = Converter(global_my_color,converter_config_path)  # 传入的是path
        # 场地解算
        # converter.camera_to_field_init(capture)
        self.converter_inted = False
        
        self.start_time = time.time()
        # fps计算
        N = 10
        self.fps_queue = deque(maxlen=10)
        self.lidar_points = None
        # 订阅Robots话题
        self.sub_detect = self.create_subscription(Robots, "detect_result", self.radar_callback, qos_profile)
        self.get_logger().info('Radar subscriber has been started at {}.'.format(today))
        # 订阅Image话题，用于converter初始化
        self.sub_image = self.create_subscription(Image, "image", self.image_callback, qos_profile)
        # 订阅点云话题 detect_pcds
        self.sub_pcds = self.create_subscription(PointCloud2, "lidar_pcds", self.pcd_callback, qos_profile)
    def pcd_callback(self, msg):
        '''
        子线程函数，对于/livox/lidar topic数据的处理 , data是传入的
        '''
        if not self.converter_inted:
            return
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(list(points))
        self.lidar_points = points
        # self.converter.lidar_to_field(points)

    def image_callback(self, msg):
        # 转换成cv2格式
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        if self.converter_inted:
            return
        self.converter.camera_to_field_init_by_image(frame)
        self.converter_inted = True
    
    def radar_callback(self, msg):
        if not self.converter_inted:
            return
        detect_results = msg.detect_results
        if self.lidar_points is None:
            self.get_logger().info("lidar points is None")
            return
        
        for detect_result in detect_results:
            # self.get_logger().info('I heard: "%s"' % detect_result)
            sg_result = detect_result
            xyxy_box, xywh_box ,  track_id , label = sg_result.xyxy_box, sg_result.xywh_box, sg_result.track_id, sg_result.label
            # self.get_logger().info('I heard: "%s"' % xyxy_box)
            
            
            print(len(self.lidar_points))
                


        # self.get_logger().info('I heard: "%s"' % msg)
    def __del__(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    radar = Radar()
    rclpy.spin(radar)
    radar.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
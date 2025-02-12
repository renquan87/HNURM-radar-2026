import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import time
from .Car.Car import *
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data


class Radar(Node):

    def __init__(self):
        super().__init__('radar_subscriber')
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos__lidar_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.bridge = CvBridge()
        # detector_config_path = "./configs/detector_config.yaml"
        # binocular_camera_cfg_path = "./configs/bin_cam_config.yaml"
        main_config_path = "/home/rm/lsa/radar/hnurm_radar/configs/main_config.yaml"
        converter_config_path = "/home/rm/lsa/radar/hnurm_radar/configs/converter_config.yaml"
        main_cfg = YAML().load(open(main_config_path, encoding='Utf-8', mode='r'))
        
        # 全局变量
        self.global_my_color = main_cfg['global']['my_color']
        is_debug = main_cfg['global']['is_debug']
        self.carList = CarList(main_cfg)
        self.carList_results = []
        self.all_detections = [] # 创建一个空列表来存储所有检测的结果
        # 当前帧ID
        self.frame_id = 1
        self.counter = 0
        
        today = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) # 今日日期，例如2024年5月6日则为20240506
        self.converter = Converter(self.global_my_color,converter_config_path)  # 传入的是path
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
        self.sub_pcds = self.create_subscription(PointCloud2, "lidar_pcds", self.pcd_callback, qos__lidar_profile)
    def pcd_callback(self, msg):
        '''
        子线程函数，对于/livox/lidar topic数据的处理 , data是传入的
        '''
        if not self.converter_inted:
            return
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(
                list(points),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float32)
        
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
        # print(len(self.lidar_points))
        # 创建总体点云pcd
        pcd_all = o3d.geometry.PointCloud()
        pcd_all.points = o3d.utility.Vector3dVector(self.lidar_points)
        # 将总体点云转到相机坐标系下
        self.converter.lidar_to_camera(pcd_all)

        # 检测框对应点云
        box_pcd = o3d.geometry.PointCloud()
        # 对每个结果进行分析 , 进行目标定位
        for detect_result in detect_results:
            # self.get_logger().info('I heard: "%s"' % detect_result)
            sg_result = detect_result
            xyxy_box, xywh_box ,  track_id , label = sg_result.xyxy_box, sg_result.xywh_box, sg_result.track_id, sg_result.label
            # self.get_logger().info('I heard: "%s"' % xyxy_box)
            # 如果没有分类出是什么车，或者是己方车辆，直接跳过
            if label == "NULL":
                continue
            if self.global_my_color == "Red" and self.carList.get_car_id(label) < 100 and self.carList.get_car_id(label) != 7:
                continue
            if self.global_my_color == "Blue" and self.carList.get_car_id(label) > 100 and self.carList.get_car_id(label) != 107:
                continue
            
            # 获取新xyxy_box , 原来是左上角和右下角，现在想要中心点保持不变，宽高设为原来的一半，再计算一个新的xyxy_box,可封装
            div_times = 1.01
            new_w = xywh_box[2] / div_times
            new_h = xywh_box[3] / div_times
            new_xyxy_box = [xywh_box[0] - new_w / 2, xywh_box[1] - new_h / 2, xywh_box[0] + new_w / 2, xywh_box[1] + new_h / 2]
            # 获取检测框内numpy格式pc
            box_pc = self.converter.get_points_in_box(pcd_all.points, new_xyxy_box)
            # print(len(box_pc))
            # 如果没有获取到点，直接continue
            if len(box_pc) == 0:
                self.get_logger().info("box_pc is None")
                continue
            box_pcd.points = o3d.utility.Vector3dVector(box_pc)
            # 点云过滤
            box_pcd = self.converter.filter(box_pcd)
            # 获取box_pcd的中心点
            cluster_result = self.converter.cluster(box_pcd) # 也就6帧变7帧，还是启用
            _, center = cluster_result
            # # 如果聚类结果为空，则用中值取点
            if center[0] == 0 and center[1] == 0 and center[2] == 0:
                center = self.converter.get_center_mid_distance(box_pcd)
            # 计算距离
            distance = self.converter.get_distance(center)
            if distance == 0:
                continue
            # 将点转到赛场坐标系下
            field_xyz = self.converter.camera_to_field(center)
            # 计算赛场坐标系下的距离
            field_distance = self.converter.get_distance(field_xyz)
            # 在图像上写距离,位置为xyxy_box的左上角,可以去掉
            # if is_debug:
            #     cv2.putText(result_img, "distance: {:.2f}".format(field_distance), (int(xyxy_box[0]), int(xyxy_box[1]),), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 122), 2)
            #     cv2.putText(result_img, "x: {:.2f}".format(field_xyz[0])+"y:{:.2f}".format(field_xyz[1])+"z:{:.2f}".format(field_xyz[2]), (int(xyxy_box[2]), int(xyxy_box[3]+10),),
            #         cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 122), 2)
            # 将结果打包
            self.carList_results.append([track_id , self.carList.get_car_id(label) , xywh_box , 1 , center , field_xyz])
        self.carList.update_car_info(self.carList_results)
        all_infos = self.carList.get_all_info() # 此步不做trust的筛选，留给messager做
        my_car_infos = []
        enemy_car_infos = []
        # result in results:[car_id , center_xy , camera_xyz , field_xyz]
        # 如果是我方车辆，找到所有敌方车辆，计算与每一台敌方车辆距离，并在图像两车辆中心点之间画线，线上写距离
        for all_info in all_infos:
            track_id , car_id , center_xy , camera_xyz , field_xyz , color , is_valid = all_info
            
            # 将信息分两个列表存储
            if color == self.global_my_color:
                if track_id == -1:
                    continue
                my_car_infos.append(all_info)
            else:
                enemy_car_infos.append(all_info)
                # print(car_id,field_xyz,color)
                
                if track_id != -1:
                    print("car",car_id,field_xyz,color)
                    # 将每个检测结果添加到列表中，增加frame_id作为每一帧的ID
                    self.all_detections.append([self.frame_id] + list(all_info))
        # 在此发布对方车辆检测结果
        # print(enemy_car_infos)




            
            
            
                


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
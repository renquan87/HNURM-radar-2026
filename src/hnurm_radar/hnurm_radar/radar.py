import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Bool
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
from detect_result.msg import Location, Locations

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import yaml
from std_msgs.msg import Header
class Radar(Node):

    def __init__(self):
        super().__init__('Radar')
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
        main_config_path = "/home/rq/radar/hnurm_radar/configs/main_config.yaml"
        converter_config_path = "/home/rq/radar/hnurm_radar/configs/converter_config.yaml"
        main_cfg = YAML().load(open(main_config_path, encoding='Utf-8', mode='r'))
        
        # 全局变量
        self.global_my_color = main_cfg['global']['my_color']
        is_debug = main_cfg['global']['is_debug']
        self.carList = CarList(main_cfg)
        self.carList_results = []
        self.all_detections = [] # 创建一个空列表来存储所有检测的结果
        self.last_all_detections = [] # 创建一个空列表来存储上一帧的检测结果
        # 当前帧ID
        self.frame_id = 1
        # 计数器，用于计算fps
        self.counter = 0
        
        today = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) # 今日日期，例如2024年5月6日则为20240506
        self.converter = Converter(self.global_my_color,converter_config_path)  # 传入的是path
        # 场地解算
        # converter.camera_to_field_init(capture)
        # self.converter_inted = False
        
        # 加载转换器配置文件
        with open(converter_config_path, 'r', encoding='utf-8') as file:
            data_loader = yaml.safe_load(file)
            
        # 读取激光雷达到相机外参
        # 构建旋转和平移矩阵，4*3的矩阵，前三列为旋转矩阵，第四列为平移矩阵
        self.R = np.array(data_loader['calib']['extrinsic']['R']['data']).reshape(
            (data_loader['calib']['extrinsic']['R']['rows'], data_loader['calib']['extrinsic']['R']['cols']))
        self.T = np.array(data_loader['calib']['extrinsic']['T']['data']).reshape(
            (data_loader['calib']['extrinsic']['T']['rows'], data_loader['calib']['extrinsic']['T']['cols']))
        # 构建外参矩阵，4*4的矩阵，激光雷达到相机的外参矩阵
        self.extrinsic_matrix = np.hstack((self.R, self.T))
        self.extrinsic_matrix = np.vstack((self.extrinsic_matrix, [0, 0, 0, 1]))
        # 相机到激光雷达的变换矩阵（通过求逆得到）
        self.extrinsic_matrix_inv = np.linalg.inv(self.extrinsic_matrix)
        
        
        self.start_time = time.time()
        # fps计算
        N = 10
        self.fps_queue = deque(maxlen=10)
        self.lidar_points = None
        # 订阅Robots话题
        self.sub_detect = self.create_subscription(Robots, "detect_result", self.radar_callback, qos_profile)
        self.get_logger().info('Radar subscriber has been started at {}.'.format(today))
        # 订阅点云话题 detect_pcds
        self.sub_pcds = self.create_subscription(PointCloud2, "lidar_pcds", self.pcd_callback, qos__lidar_profile)
        # 发布车辆位置信息
        self.pub_location = self.create_publisher(Locations, "location", qos_profile)
        self.last_frameid = -1
        # 可视化去除地面后的点云
        self.pub_nognd = self.create_publisher(PointCloud2, "pcd_removed", qos__lidar_profile)
        
        # 订阅实时icp传来的tf消息
        self.tf_buffer = Buffer()  # 创建 TF 缓冲区
        self.tf_listener = TransformListener(self.tf_buffer, self)  # 创建监听器
        self.timer = self.create_timer(1.0, self.on_timer)  # 定时查询 TF
        self.radar_to_field = np.ones((4, 4)) # 激光雷达到赛场的tf矩阵
        self.radar_to_field_inv = np.ones((4, 4)) # 激光雷达到赛场的tf矩阵的逆（用于将点云转换回雷达坐标系）
    
    # 定时查询 TF
    def on_timer(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                target_frame='map',
                source_frame='livox',
                time=rclpy.time.Time()  # 获取最新可用变换
            )
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            # self.log_transform(transform)
            # 转换为 4x4 变换矩阵
            transform_matrix = self.tf_to_matrix(translation, rotation)
            self.radar_to_field = transform_matrix
            self.radar_to_field_inv = np.linalg.inv(self.radar_to_field)
            # self.get_logger().info(f"获取 TF 成功: {transform}")
        except TransformException as ex:
            self.radar_to_field = np.ones((4, 4))
            self.get_logger().error(f"获取 TF 失败: {ex}")    
    
    # 将 TF 转换为 4x4 齐次变换矩阵
    def tf_to_matrix(self, translation, rotation):
        # 四元数转旋转矩阵
        q = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
        R = self.quaternion_to_rotation_matrix(q)

        # 构建 4x4 齐次变换矩阵
        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R
        transform_matrix[:3, 3] = [translation.x, translation.y, translation.z]
        return transform_matrix
    def quaternion_to_rotation_matrix(self, q):
        # 四元数转旋转矩阵
        x, y, z, w = q
        return np.array([
            [1 - 2*(y**2 + z**2), 2*(x*y - z*w),     2*(x*z + y*w)],
            [2*(x*y + z*w),     1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
            [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x**2 + y**2)]
        ])
    
    # 将相机坐标系下的点云转换到激光雷达坐标系
    def camera_to_lidar(self, pc):
        pc = np.hstack((pc, np.ones((pc.shape[0], 1)))) # 齐次化
        ret = np.dot(pc, self.extrinsic_matrix) # 矩阵变换
        ret = ret[:, :3] # 去齐次化
        return ret
    # 点云回调函数
    def pcd_callback(self, msg):
        '''
        子线程函数，对于/livox/lidar topic数据的处理 , data是传入的
        '''
        points = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        points = np.array(
                list(points),
                dtype=[("x", np.float32), ("y", np.float32), ("z", np.float32)],
        )
        points = np.stack([points["x"], points["y"], points["z"]], axis=-1).astype(np.float32)
        
        points = np.array(list(points))
        self.lidar_points = points.copy()
        # self.converter.lidar_to_field(points)

    # 去除地面点云，将点云转换到赛场坐标系，过滤 Z 值，发布去除地面后的点云
    def remove_ground_points(self, pcd):
        if self.radar_to_field is None:
            self.get_logger().info("radar_to_field is None, skip removing process")
            return pcd

        points = np.asarray(pcd.points)
        # self.get_logger().info("points: {}".format(points))
        # 添加一列全为 1 的列，扩展为齐次坐标
        points = np.hstack((points, np.ones((points.shape[0], 1))))

        # 转换到赛场坐标系
        field_pts = np.dot(points,self.radar_to_field)  # 转置以保持形状一致
        field_pts = field_pts[:, :3]  # 去掉齐次坐标的最后一列

        # 筛选 z 值不在范围内的点
        mask = ((field_pts[:, 2] > -11111))
        filtered_points = field_pts[mask]

        # 检查过滤后的点云
        if filtered_points.size == 0:
            self.get_logger().info("No points left after filtering.")
            return o3d.geometry.PointCloud()

        # 再转换回到雷达坐标系
        filtered_points = np.hstack((filtered_points, np.ones((filtered_points.shape[0], 1))))
        filtered_points = np.dot(filtered_points, self.radar_to_field_inv)  # 转置以保持形状一致
        filtered_points = filtered_points[:, :3]  # 去掉齐次坐标的最后一列

        # 创建新的点云
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(filtered_points)

        # 发布点云数据
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "livox"
        # 以fileds x,y,z打包
        fields = [
                    pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
                    pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
                ]
        pc = pc2.create_cloud(header, fields, field_pts)
        # pc = pc2.create_cloud(header, fields, filtered_points) -> 实际效果不好，所以实际输入的是field_pts
        self.pub_nognd.publish(pc)
        
        # 可视化过滤后点云
        # o3d.visualization.draw_geometries([filtered_pcd])
        return filtered_pcd
    
    # 雷达回调函数
    def radar_callback(self, msg):
        detect_results = msg.detect_results
        if self.lidar_points is None:
            self.get_logger().info("lidar points is None")
            return
        # print(len(self.lidar_points))
        # 创建总体点云pcd
        # lidar_points_fixed = self.remove_ground_points(self.lidar_points)
        pcd_all = o3d.geometry.PointCloud()
        pcd_all.points = o3d.utility.Vector3dVector(self.lidar_points)
        pcd_fixed = self.remove_ground_points(pcd_all)
        # pcd_fixed = pcd_all
        # pcd_print = np.hstack((self.lidar_points, np.ones((self.lidar_points.shape[0], 1))))
        # self.get_logger().info("pcd_print: {}".format(pcd_print.T))
        # 将总体点云转到相机坐标系下
        self.converter.lidar_to_camera(pcd_fixed)
        if self.radar_to_field is None:
            self.get_logger().info("radar_to_field is None")
            return
        # 检测框对应点云
        box_pcd = o3d.geometry.PointCloud()
        # 遍历所有检测结果
        for detect_result in detect_results:
            # self.get_logger().info('I heard: "%s"' % detect_result)
            sg_result = detect_result
            # 获取检测框信息
            xyxy_box, xywh_box ,  track_id , label = sg_result.xyxy_box, sg_result.xywh_box, sg_result.track_id, sg_result.label
            # self.get_logger().info('I heard: "%s"' % xyxy_box)
            # 过滤无效和己方车辆
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
            box_pc = self.converter.get_points_in_box(pcd_fixed.points, new_xyxy_box)
            
            # 如果没有获取到点，直接continue
            if len(box_pc) == 0:
                self.get_logger().info("box_pc is None")
                continue
            box_pcd.points = o3d.utility.Vector3dVector(box_pc)
            box_pcd = self.converter.filter(box_pcd) # 过滤点云，去除离检测框太远的点
            # 聚类，获取box_pcd的中心点
            cluster_result = self.converter.cluster(box_pcd) # 也就6帧变7帧，还是启用
            _, center = cluster_result
            # 如果聚类结果为空，则用中值取点
            if center[0] == 0 and center[1] == 0 and center[2] == 0:
                center = self.converter.get_center_mid_distance(box_pcd)
            # 计算距离
            distance = self.converter.get_distance(center)
            if distance == 0:
                continue

            # 多坐标系转换
            # print((self.camera_to_lidar((center))))
            # 相机坐标系 → 雷达坐标系
            center = np.hstack((center, np.array((1)))) # 齐次化
            center = center[:3]
            center = np.hstack((center, np.array((1)))) # ❓为什么要齐次化两次？
            lidar_center = np.dot(center, self.extrinsic_matrix) # 使用外参矩阵进行转换
            lidar_center = lidar_center[:3]
            self.get_logger().info("lidar_center: {}".format(lidar_center))
            # self.get_logger().info("center: {}".format(center))

            # 雷达坐标系 → 赛场坐标系  
            lidar_center = np.hstack((lidar_center, np.array((1))))
            field_xyz = np.dot(self.radar_to_field, lidar_center) # 使用重定位TF进行转换
            field_xyz = field_xyz[:3]
            self.get_logger().info("field_xyz: {}".format(field_xyz))
            # 将点转到赛场坐标系下
            # field_xyz = self.converter.camera_to_field(center)
            # 计算赛场坐标系下的距离
            # field_distance = self.converter.get_distance(field_xyz)
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
        
        allLocation = Locations()
        
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
                    # print("car",car_id,field_xyz,color)
                    # 将每个检测结果添加到列表中，增加frame_id作为每一帧的ID
                    self.all_detections.append([self.frame_id] + list(all_info))


                    loc = Location()
                    loc.x = field_xyz[0]
                    loc.y = field_xyz[1]
                    loc.z = field_xyz[2]
                    loc.id = car_id
                    loc.label = color
                    allLocation.locs.append(loc)
        self.pub_location.publish(allLocation)



            
            
            
                


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
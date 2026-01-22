#!/usr/bin/env python3
"""
将2D地图图片转换为PCD点云文件
使用方法: python3 image_to_pcd.py
"""

import cv2
import numpy as np
import open3d as o3d
import os

def image_to_point_cloud(image_path, output_path, scale=0.1, height_threshold=200):
    """
    将2D地图图片转换为3D点云
    
    参数:
        image_path: 输入图片路径
        output_path: 输出PCD文件路径
        scale: 比例尺（像素到米的转换）
        height_threshold: 高度阈值，用于区分障碍物和自由空间
    """
    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图片: {image_path}")
        return False
    
    # 转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 二值化处理（白色为自由空间，黑色为障碍物）
    _, binary = cv2.threshold(gray, height_threshold, 255, cv2.THRESH_BINARY_INV)
    
    # 获取图片尺寸
    height, width = binary.shape
    
    # 创建点云列表
    points = []
    
    # 遍历图片，提取障碍物点
    for y in range(height):
        for x in range(width):
            if binary[y, x] > 0:  # 如果是障碍物
                # 转换为实际坐标（图片中心为原点）
                real_x = (x - width / 2) * scale
                real_y = (y - height / 2) * scale
                real_z = 0.0  # 假设所有障碍物在同一高度
                
                points.append([real_x, real_y, real_z])
    
    # 转换为numpy数组
    points = np.array(points, dtype=np.float32)
    
    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 保存为PCD文件
    success = o3d.io.write_point_cloud(output_path, pcd)
    
    if success:
        print(f"成功保存点云到: {output_path}")
        print(f"点云包含 {len(points)} 个点")
        return True
    else:
        print(f"保存点云失败: {output_path}")
        return False

def create_field_map(image_path, output_path, scale=0.1):
    """
    创建场地地图点云（包含边界和关键特征）
    
    参数:
        image_path: 输入图片路径
        output_path: 输出PCD文件路径
        scale: 比例尺（像素到米的转换）
    """
    # 读取图片
    img = cv2.imread(image_path)
    if img is None:
        print(f"无法读取图片: {image_path}")
        return False
    
    # 转换为灰度图
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # 边缘检测
    edges = cv2.Canny(gray, 50, 150)
    
    # 获取图片尺寸
    height, width = edges.shape
    
    # 创建点云列表
    points = []
    
    # 遍历边缘点
    for y in range(height):
        for x in range(width):
            if edges[y, x] > 0:  # 如果是边缘
                # 转换为实际坐标
                real_x = (x - width / 2) * scale
                real_y = (y - height / 2) * scale
                real_z = 0.0
                
                points.append([real_x, real_y, real_z])
    
    # 如果没有足够的点，添加一些边界点
    if len(points) < 100:
        print("检测到的边缘点较少，添加边界点...")
        # 添加外边界
        for i in range(width):
            real_x = (i - width / 2) * scale
            real_y = -(height / 2) * scale
            points.append([real_x, real_y, 0.0])
            real_y = (height / 2) * scale
            points.append([real_x, real_y, 0.0])
        
        for i in range(height):
            real_x = -(width / 2) * scale
            real_y = (i - height / 2) * scale
            points.append([real_x, real_y, 0.0])
            real_x = (width / 2) * scale
            points.append([real_x, real_y, 0.0])
    
    # 转换为numpy数组
    points = np.array(points, dtype=np.float32)
    
    # 创建Open3D点云对象
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # 保存为PCD文件
    success = o3d.io.write_point_cloud(output_path, pcd)
    
    if success:
        print(f"成功保存场地地图到: {output_path}")
        print(f"点云包含 {len(points)} 个点")
        return True
    else:
        print(f"保存场地地图失败: {output_path}")
        return False

if __name__ == "__main__":
    # 设置路径
    image_path = "/home/rq/radar/hnurm_radar/map2026.jpg"
    output_path = "/home/rq/radar/hnurm_radar/normalized_map.pcd"
    
    print("开始转换地图图片为PCD格式...")
    
    # 创建场地地图（推荐使用这个）
    success = create_field_map(image_path, output_path, scale=0.05)
    
    if not success:
        print("尝试使用障碍物提取方法...")
        # 备选方法：提取障碍物
        success = image_to_point_cloud(image_path, output_path, scale=0.05)
    
    if success:
        print("地图转换完成！")
        print(f"输出文件: {output_path}")
        print("现在可以更新registration配置文件使用这个地图文件。")
    else:
        print("地图转换失败！")
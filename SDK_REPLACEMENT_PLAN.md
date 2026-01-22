# HAP SDK替换计划

## 概述
将 `/home/rq/radar/hnurm_radar` 中的旧Livox SDK替换为 `/home/rq/radar/livox_ws` 中的HAP SDK。

## 关键配置
- HAP雷达IP: 192.168.1.100
- 保留并适配现有bringup.sh脚本
- 仅支持HAP，完全移除旧SDK

## 实施步骤

### 1. 备份项目
```bash
cp -r /home/rq/radar/hnurm_radar /home/rq/radar/hnurm_radar_backup
```

### 2. 移除旧SDK依赖
- 删除 `/home/rq/radar/hnurm_radar/Livox-SDK/` 目录
- 删除 `/home/rq/radar/hnurm_radar/src/livox_ros2_driver/` 目录

### 3. 集成新SDK
- 复制 `/home/rq/radar/livox_ws/src/livox_ros_driver2/` 到 `/home/rq/radar/hnurm_radar/src/`
- 复制 `/home/rq/radar/livox_ws/src/Livox-SDK2/` 到 `/home/rq/radar/hnurm_radar/src/`

### 4. 修改配置文件
- 创建 `/home/rq/radar/hnurm_radar/configs/HAP_config.json`
- 更新 `/home/rq/radar/hnurm_radar/configs/main_config.yaml`

### 5. 更新依赖配置
修改以下文件：
- `/home/rq/radar/hnurm_radar/src/hnurm_radar/package.xml`
- `/home/rq/radar/hnurm_radar/src/livox_ros_driver2/package.xml`
- 相关CMakeLists.txt文件

### 6. 适配启动脚本
修改 `/home/rq/radar/hnurm_radar/bringup.sh` 以支持新SDK

### 7. 构建和测试
```bash
cd /home/rq/radar/hnurm_radar
colcon build
source install/setup.zsh
```

## 关键文件内容

### HAP_config.json
```json
{
  "lidar_summary_info": {"lidar_type": 8},
  "HAP": {
    "lidar_net_info": {
      "cmd_data_port": 56000,
      "point_data_port": 57000,
      "imu_data_port": 58000
    },
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",
      "point_data_ip": "192.168.1.50",
      "imu_data_ip": "192.168.1.50"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.100",
      "pcl_data_type": 1,
      "pattern_mode": 0,
      "blind_spot_set": 50,
      "extrinsic_parameter": {
        "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
        "x": 0, "y": 0, "z": 0
      }
    }
  ]
}
```

### bringup.sh修改
```bash
#!/bin/bash
source install/setup.zsh
# 使用新的HAP驱动
ros2 launch livox_ros_driver2 livox_lidar_rviz_launch.py &
# 启动主程序
ros2 launch hnurm_bringup hnurm_radar_launch.py
```

## 验证清单
- [ ] 编译成功无错误
- [ ] HAP雷达连接成功
- [ ] 点云数据正常发布
- [ ] Python节点正确接收数据
- [ ] 目标检测功能正常

## 风险缓解
- 保留完整备份以便回滚
- 逐步替换，每步验证
- 保持话题名称兼容性
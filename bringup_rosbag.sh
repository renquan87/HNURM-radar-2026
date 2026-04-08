#!/bin/bash
# bringup_rosbag.sh — Rosbag 回放模式一键启动脚本
#
# 分两个终端启动，与实时模式的 bringup.sh 对应：
#   终端1: ros2 launch hnurm_bringup hnurm_rosbag_launch.py
#          （rosbag play + 主节点组 + rviz + foxglove）
#   终端2: ros2 launch registration registration.launch.py
#          （ICP 配准，rosbag 模式自动使用 rosbag_pcd_file）
#
# 前置条件：
#   1. configs/main_config.yaml 中 camera.mode 设为 "rosbag"
#   2. configs/main_config.yaml 中 camera.rosbag_path 填写 rosbag 目录路径
#   3. colcon build 已完成

# 移除MVS库路径以避免libusb冲突
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | sed 's|/opt/MVS/lib/64:||g')

# 激活虚拟环境（PyTorch/TensorRT/Ultralytics 等依赖）
VENV_PATH="/data/venv/radar-env/bin/activate"
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
    echo "✅ 虚拟环境已激活: $(which python)"
else
    echo "⚠️  虚拟环境未找到: $VENV_PATH"
fi

# 自动定位到脚本所在目录（即项目根目录）
cd "$(dirname "$(readlink -f "$0")")"
source /opt/ros/humble/setup.bash
source install/setup.bash

cmds=(
       "ros2 launch hnurm_bringup hnurm_rosbag_launch.py"
       "ros2 launch registration registration.launch.py"
      )

for cmd in "${cmds[@]}"
do
    echo "Current CMD : $cmd"
    gnome-terminal -- bash -c "source /data/venv/radar-env/bin/activate 2>/dev/null;cd $(pwd);source /opt/ros/humble/setup.bash;source install/setup.bash;$cmd;exec bash;"
    sleep 0.1
done

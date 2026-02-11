#!/bin/bash
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

echo "Press any key to continue after camera and lidar are ready!"
read

cmds=(
       "ros2 launch livox_ros_driver2 rviz_HAP_launch.py"
       "ros2 launch hnurm_bringup hnurm_radar_launch.py"
       "ros2 launch registration registration.launch.py"
    #    "ros2 bag record /livox/lidar /detect_result /location  /ekf_location_filtered"
       )
       
cd /data/projects/radar/hnurm_radar
source /opt/ros/humble/setup.bash
source install/setup.bash

for cmd in "${cmds[@]}"
do
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "source /data/venv/radar-env/bin/activate 2>/dev/null;cd $(pwd);source /opt/ros/humble/setup.bash;source install/setup.bash;$cmd;exec bash;"
    sleep 0.1
done
#!/bin/bash
echo "Press any to continue after camera and lidar are ready!"
read

cmds=(
       "ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py"
       "ros2 launch hnurm_bringup hnurm_radar_launch.py"
       "ros2 launch registration registration.launch.py"
    #    "ros2 bag record /livox/lidar /detect_result /location  /ekf_location_filtered"
    )
for cmd in "${cmds[@]}"
do
    echo Current CMD : "$cmd"
    gnome-terminal -- bash -c "cd $(pwd);source /home/rq/radar/hnurm_radar/install/setup.bash;$cmd;exec bash;"
    sleep 0.1
done

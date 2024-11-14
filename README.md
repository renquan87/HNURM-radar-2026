# 启动 Lidar

在启动launch前记得修改config里面的BDCODE

```shell
source install/setup.zsh
ros2 launch livox_ros2_driver livox_lidar_rviz_launch.py
# 或者
ros2 launch livox_ros2_driver livox_lidar_launch.py
```

# 调试

启动foxglove

```shell
source install/setup.zsh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
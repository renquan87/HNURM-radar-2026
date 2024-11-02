import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 0    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar,1-hub
publish_freq  = 10.0 # freqency of publish,1.0,2.0,5.0,10.0,etc
output_type   = 0
frame_id      = 'livox_frame'
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_config_path = os.path.join(
        get_package_share_directory('livox_ros2_driver'),
        'config'
        )

# If you want to be able to quickly change configuration and have them impact this launch, 
# use `colcon build --symlink-install` and change directly in the source folder.
rviz_config_path = os.path.join(cur_config_path, 'livox_lidar.rviz')
user_config_path = os.path.join(cur_config_path, 'livox_lidar_config.json')
################### user configure parameters for ros2 end #####################

livox_ros2_params = [
    {"xfer_format": xfer_format},
    {"multi_topic": multi_topic},
    {"data_src": data_src},
    {"publish_freq": publish_freq},
    {"output_data_type": output_type},
    {"frame_id": frame_id},
    {"lvx_file_path": lvx_file_path},
    {"user_config_path": user_config_path},
    {"cmdline_input_bd_code": cmdline_bd_code}
]

def generate_launch_description():

    # Composition Manager
    composition_manager = ComposableNodeContainer (
        name="ComponentManager",
        namespace="livox_ros",
        package="rclcpp_components",
        executable="component_container"
    )

    livox_driver = LoadComposableNodes(
        target_container="livox_ros/ComponentManager",
        composable_node_descriptions=[
            ComposableNode(
                package="livox_ros2_driver",
                namespace="livox_ros",
                plugin="livox_ros::LivoxDriver",
                name="livox_lidar_publisher",
                parameters=livox_ros2_params,
                extra_arguments=[{"use_intra_process_comms": True}]
            )
        ]
    )

    return LaunchDescription([
        composition_manager,
        livox_driver
    ])
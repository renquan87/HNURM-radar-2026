import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
# execute
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

calib_param = {
    "fExposureTime": 20000.0,
    'fGain': 15.0
}


def generate_launch_description():
    camera_dir = get_package_share_directory('hnurm_camera')
    params_file = LaunchConfiguration('params_file')

    cam_node = LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(camera_dir, 'params', 'default.yaml'),
            description='Full path to the ROS2 parameters file to use for the camera'
        ),
        Node(
            package='hnurm_camera',
            executable='hnurm_camera_node',
            parameters=[params_file, calib_param]
        )
    ])

    # execute `ros2 run camera_calibration cameracalibrator --size --square
    camera_calibration_node = ExecuteProcess(

        cmd=['ros2', 'run', 'camera_calibration', 'cameracalibrator', '--size', '11x8', '--square', '0.02',
             '--no-service-check'
             ],
        output='screen'
    )

    return LaunchDescription([camera_calibration_node, cam_node])

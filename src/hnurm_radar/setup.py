from setuptools import find_packages, setup

package_name = 'hnurm_radar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        'ultralytics': ['cfg/**/*.yaml', 'nn/backbone/faster_cfg/*.yaml'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lisian',
    maintainer_email='lisian_magic@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 方案一：纯相机透视变换
            "camera_detector = hnurm_radar.camera_scheme.camera_detector:main",
            # 方案二：相机 + 激光雷达
            "detector_node = hnurm_radar.lidar_scheme.detector_node:main",
            "radar_node = hnurm_radar.lidar_scheme.radar:main",
            "lidar_node = hnurm_radar.lidar_scheme.lidar_node:main",
            # 共享节点
            "publish_video = hnurm_radar.shared.publish_video:main",
            "display_panel = hnurm_radar.shared.display_panel:main",
            "judge_messager = hnurm_radar.shared.judge_messager:main",
            # 工具
            "make_mask = hnurm_radar.camera_locator.make_mask:main",
            "perspective_calibrator = hnurm_radar.camera_locator.perspective_calibrator:main"
        ],
    },
)

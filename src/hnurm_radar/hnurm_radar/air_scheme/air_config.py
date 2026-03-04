"""
air_config.py — 空中机器人检测配置管理

适配自 air_target_radar/src/config.py，集成到 hnurm_radar 项目。
从 main_config.yaml 的 air_target 段读取配置。
"""

from dataclasses import dataclass, field as dc_field
from typing import Tuple


@dataclass
class AirROIConfig:
    """空中目标 ROI 裁剪范围（雷达坐标系）"""
    x_min: float = -1.0
    x_max: float = 30.0
    y_min: float = -9.0
    y_max: float = 9.0
    z_min: float = -3.5
    z_max: float = 0.5


@dataclass
class AirHeightFilterConfig:
    """空中目标高度过滤参数
    雷达坐标系中 Z 轴向下为负，空中目标 Z 值在 z_min ~ z_max 之间
    """
    z_min: float = -1.5   # 离雷达更远（更高）
    z_max: float = -0.5   # 离雷达更近（较低）


@dataclass
class AirPreprocessConfig:
    """空中目标点云预处理参数"""
    roi: AirROIConfig = dc_field(default_factory=AirROIConfig)
    voxel_size: float = 0.05
    height_filter: AirHeightFilterConfig = dc_field(default_factory=AirHeightFilterConfig)


@dataclass
class AirClusterConfig:
    """DBSCAN 聚类参数"""
    eps: float = 0.3
    min_samples: int = 5
    z_zip: float = 0.5     # Z 轴压缩系数（HITS 特性）


@dataclass
class AirTargetFilterConfig:
    """空中目标筛选参数"""
    min_points: int = 5
    max_points: int = 200
    min_size: float = 0.1
    max_size: float = 1.5


@dataclass
class AirKalmanConfig:
    """卡尔曼滤波器参数"""
    q_pos: float = 0.1
    q_vel: float = 0.5
    q_pv: float = 0.05
    r_pos: float = 0.2
    r_vel: float = 1.0
    decay_rate: float = 0.1
    max_velocity: float = 5.0
    cov_factor: float = 0.1
    stop_p_time: int = 30


@dataclass
class AirTrackingConfig:
    """目标跟踪参数"""
    enabled: bool = True
    max_lost_frames: int = 10
    match_distance: float = 50.0
    force_combine_dist: float = 0.5
    cc_thres: float = 0.3
    kalman: AirKalmanConfig = dc_field(default_factory=AirKalmanConfig)


@dataclass
class AirBackgroundConfig:
    """背景减除参数"""
    enabled: bool = True
    voxel_size: float = 0.15
    occupy_threshold: int = 5
    learning_frames: int = 10


@dataclass
class AirBufferConfig:
    """多帧缓冲参数"""
    enabled: bool = False
    frame_count: int = 3


@dataclass
class AirTargetConfig:
    """空中机器人检测完整配置"""
    enabled: bool = True
    preprocessing: AirPreprocessConfig = dc_field(default_factory=AirPreprocessConfig)
    clustering: AirClusterConfig = dc_field(default_factory=AirClusterConfig)
    target_filter: AirTargetFilterConfig = dc_field(default_factory=AirTargetFilterConfig)
    tracking: AirTrackingConfig = dc_field(default_factory=AirTrackingConfig)
    background: AirBackgroundConfig = dc_field(default_factory=AirBackgroundConfig)
    buffer: AirBufferConfig = dc_field(default_factory=AirBufferConfig)
    publish_rate: float = 10.0  # 发布频率 Hz


def load_air_target_config(cfg: dict) -> AirTargetConfig:
    """从 main_config.yaml 的 air_target 段加载空中目标配置

    Args:
        cfg: main_config.yaml 解析后的完整字典

    Returns:
        AirTargetConfig 实例
    """
    at = cfg.get("air_target", {})
    if not at:
        return AirTargetConfig(enabled=False)

    result = AirTargetConfig()
    result.enabled = at.get("enabled", True)
    result.publish_rate = at.get("publish_rate", 10.0)

    # 预处理
    prep = at.get("preprocessing", {})
    roi = prep.get("roi", {})
    hf = prep.get("height_filter", {})
    result.preprocessing = AirPreprocessConfig(
        roi=AirROIConfig(
            x_min=roi.get("x_min", -1.0),
            x_max=roi.get("x_max", 30.0),
            y_min=roi.get("y_min", -9.0),
            y_max=roi.get("y_max", 9.0),
            z_min=roi.get("z_min", -3.5),
            z_max=roi.get("z_max", 0.5),
        ),
        voxel_size=prep.get("voxel_size", 0.05),
        height_filter=AirHeightFilterConfig(
            z_min=hf.get("z_min", -1.5),
            z_max=hf.get("z_max", -0.5),
        ),
    )

    # 聚类
    cl = at.get("clustering", {})
    result.clustering = AirClusterConfig(
        eps=cl.get("eps", 0.3),
        min_samples=cl.get("min_samples", 5),
        z_zip=cl.get("z_zip", 0.5),
    )

    # 目标筛选
    tf = at.get("target_filter", {})
    result.target_filter = AirTargetFilterConfig(
        min_points=tf.get("min_points", 5),
        max_points=tf.get("max_points", 200),
        min_size=tf.get("min_size", 0.1),
        max_size=tf.get("max_size", 1.5),
    )

    # 跟踪
    tr = at.get("tracking", {})
    kp = tr.get("kalman", {})
    result.tracking = AirTrackingConfig(
        enabled=tr.get("enabled", True),
        max_lost_frames=tr.get("max_lost_frames", 10),
        match_distance=tr.get("match_distance", 50.0),
        force_combine_dist=tr.get("force_combine_dist", 0.5),
        cc_thres=tr.get("cc_thres", 0.3),
        kalman=AirKalmanConfig(
            q_pos=kp.get("q_pos", 0.1),
            q_vel=kp.get("q_vel", 0.5),
            q_pv=kp.get("q_pv", 0.05),
            r_pos=kp.get("r_pos", 0.2),
            r_vel=kp.get("r_vel", 1.0),
            decay_rate=kp.get("decay_rate", 0.1),
            max_velocity=kp.get("max_velocity", 5.0),
            cov_factor=kp.get("cov_factor", 0.1),
            stop_p_time=kp.get("stop_p_time", 30),
        ),
    )

    # 背景减除
    bg = at.get("background", {})
    result.background = AirBackgroundConfig(
        enabled=bg.get("enabled", True),
        voxel_size=bg.get("voxel_size", 0.15),
        occupy_threshold=bg.get("occupy_threshold", 5),
        learning_frames=bg.get("learning_frames", 10),
    )

    # 多帧缓冲
    bu = at.get("buffer", {})
    result.buffer = AirBufferConfig(
        enabled=bu.get("enabled", False),
        frame_count=bu.get("frame_count", 3),
    )

    return result

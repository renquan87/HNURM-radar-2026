"""
air_config.py — 空中机器人检测配置管理

适配自 air_target_radar/src/config.py，集成到 hnurm_radar 项目。
从 main_config.yaml 的 air_target 段读取配置。
"""

from dataclasses import dataclass, field as dc_field


@dataclass
class AirROIConfig:
    """空中目标 ROI 裁剪范围（雷达坐标系：X=前方, Y=左方, Z=上方, 右手系）"""
    x_min: float = -1.0    # 前后方向（X）下限；负值=雷达后方
    x_max: float = 30.0    # 前后方向（X）上限
    y_min: float = -9.0    # 左右方向（Y）下限；负值=右方
    y_max: float = 9.0     # 左右方向（Y）上限
    z_min: float = -3.5   # 赛场默认；实验室用 -1.3（需低于地面供 BG 学习）
    z_max: float = 0.5     # 赛场默认；实验室用 0.3


@dataclass
class AirHeightFilterConfig:
    """空中目标高度过滤参数
    雷达坐标系中 LiDAR 在 Z=0，Z 轴向上为正。
    更负 = 更低（接近地面），更接近 0 = 更高（接近 LiDAR）。
    空中目标 Z 值在 z_min ~ z_max 之间。
    """
    z_min: float = -1.5   # 飞行区域 Z 下界（更接近地面）；赛场默认，实验室用 -1.0
    z_max: float = -0.5   # 飞行区域 Z 上界（更接近 LiDAR）；赛场默认，实验室用 -0.35


@dataclass
class AirAdaptiveHeightConfig:
    """高度自适应过滤参数"""
    enabled: bool = False
    margin: float = 0.25
    min_span: float = 0.5
    max_span: float = 2.0


@dataclass
class AirPreprocessConfig:
    """空中目标点云预处理参数"""
    roi: AirROIConfig = dc_field(default_factory=AirROIConfig)
    voxel_size: float = 0.05
    height_filter: AirHeightFilterConfig = dc_field(default_factory=AirHeightFilterConfig)
    adaptive_height: AirAdaptiveHeightConfig = dc_field(default_factory=AirAdaptiveHeightConfig)


@dataclass
class AirClusterConfig:
    """DBSCAN 聚类参数"""
    eps: float = 0.3
    min_samples: int = 5
    z_zip: float = 0.5     # Z 轴压缩系数（HITS 特性）
    backend: str = "open3d"  # open3d | sklearn
    split_merged: bool = True
    split_size_xy: float = 1.0
    split_min_gap: float = 0.25
    split_min_points: int = 12


@dataclass
class AirTargetFilterConfig:
    """空中目标筛选参数"""
    min_points: int = 5
    max_points: int = 200
    min_size: float = 0.1
    max_size: float = 1.5
    confidence_threshold: float = 0.0


@dataclass
class AirKalmanConfig:
    """卡尔曼滤波器参数（对齐 HITS KalmanFilter.cpp 默认值）"""
    q_pos: float = 1e-7
    q_vel: float = 5e-6
    q_pv: float = 5e-6
    r_pos: float = 5e-2
    r_vel: float = 5e-2
    decay_rate: float = 1e-4
    max_velocity: float = 10.0
    cov_factor: float = 2.5
    stop_p_time: int = 40
    init_p_times: int = 20


@dataclass
class AirTrackingConfig:
    """目标跟踪参数"""
    enabled: bool = True
    max_lost_frames: int = 120
    match_distance: float = 6.0
    force_combine_dist: float = 0.5
    cc_thres: float = 0.05
    combine_limit: int = 15
    separate_limit: int = 8
    confirm_frames: int = 2
    kalman: AirKalmanConfig = dc_field(default_factory=AirKalmanConfig)


@dataclass
class AirLooseQueryConfig:
    """丢失目标松弛查询参数（HITS loose query）"""
    enabled: bool = True
    max_lost_frames: int = 5
    expand_xy: float = 1.0
    expand_z: float = 0.5
    eps_scale: float = 1.4
    min_samples_scale: float = 0.6


@dataclass
class AirBackgroundConfig:
    """背景减除参数

    新方案（map_kdtree 模式）：
      使用场景配置中的点云地图（scenes.xxx.pcd_file）作为背景，
      在 map 坐标系下做 KDTree 最近邻匹配。
      不需要赛前录制背景 PCD，TF 漂移不影响背景模型。

    旧方案（voxel_legacy 模式）：
      使用 bg_pcd_file 指定的预录 PCD 或在线学习。
      需要将 PCD 逆变换到 livox 帧，TF 漂移时需重载。

    source 选项：
      "map"    → 新方案：使用场景点云地图（推荐）
      "pcd"    → 旧方案：使用 bg_pcd_file 指定的预录背景 PCD
      "online" → 旧方案：纯在线学习（不推荐）
    """
    enabled: bool = True
    source: str = "map"         # 背景来源: "map" | "pcd" | "online"
    bg_threshold: float = 0.15  # map 模式：KDTree 最近邻距离阈值 (m)
    voxel_size: float = 0.10    # 地图降采样体素大小 (m)
    occupy_threshold: int = 5   # 旧模式：体素占据阈值
    learning_frames: int = 10   # 旧模式：在线学习帧数
    bg_pcd_file: str = ""       # 旧模式：预建背景PCD路径（source="pcd" 时使用）


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
    loose_query: AirLooseQueryConfig = dc_field(default_factory=AirLooseQueryConfig)
    background: AirBackgroundConfig = dc_field(default_factory=AirBackgroundConfig)
    buffer: AirBufferConfig = dc_field(default_factory=AirBufferConfig)
    publish_rate: float = 10.0  # 发布频率 Hz
    strict_dual_uav: bool = False
    max_targets: int = 0
    field_split_x: float = 14.0


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
    result.strict_dual_uav = at.get("strict_dual_uav", False)
    result.max_targets = at.get("max_targets", 0)
    result.field_split_x = at.get("field_split_x", 14.0)

    # 预处理
    prep = at.get("preprocessing", {})
    roi = prep.get("roi", {})
    hf = prep.get("height_filter", {})
    ah = prep.get("adaptive_height", {})
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
        adaptive_height=AirAdaptiveHeightConfig(
            enabled=ah.get("enabled", False),
            margin=ah.get("margin", 0.25),
            min_span=ah.get("min_span", 0.5),
            max_span=ah.get("max_span", 2.0),
        ),
    )

    # 聚类
    cl = at.get("clustering", {})
    result.clustering = AirClusterConfig(
        eps=cl.get("eps", 0.3),
        min_samples=cl.get("min_samples", 5),
        z_zip=cl.get("z_zip", 0.5),
        backend=cl.get("backend", "open3d"),
        split_merged=cl.get("split_merged", True),
        split_size_xy=cl.get("split_size_xy", 1.0),
        split_min_gap=cl.get("split_min_gap", 0.25),
        split_min_points=cl.get("split_min_points", 12),
    )

    # 目标筛选
    tf = at.get("target_filter", {})
    result.target_filter = AirTargetFilterConfig(
        min_points=tf.get("min_points", 5),
        max_points=tf.get("max_points", 200),
        min_size=tf.get("min_size", 0.1),
        max_size=tf.get("max_size", 1.5),
        confidence_threshold=tf.get("confidence_threshold", 0.0),
    )

    # 跟踪
    tr = at.get("tracking", {})
    kp = tr.get("kalman", {})
    result.tracking = AirTrackingConfig(
        enabled=tr.get("enabled", True),
        max_lost_frames=tr.get("max_lost_frames", 120),
        match_distance=tr.get("match_distance", 6.0),
        force_combine_dist=tr.get("force_combine_dist", 0.5),
        cc_thres=tr.get("cc_thres", 0.05),
        combine_limit=tr.get("combine_limit", 15),
        separate_limit=tr.get("separate_limit", 8),
        confirm_frames=tr.get("confirm_frames", 2),
        kalman=AirKalmanConfig(
            q_pos=kp.get("q_pos", 1e-7),
            q_vel=kp.get("q_vel", 5e-6),
            q_pv=kp.get("q_pv", 5e-6),
            r_pos=kp.get("r_pos", 5e-2),
            r_vel=kp.get("r_vel", 5e-2),
            decay_rate=kp.get("decay_rate", 1e-4),
            max_velocity=kp.get("max_velocity", 10.0),
            cov_factor=kp.get("cov_factor", 2.5),
            stop_p_time=kp.get("stop_p_time", 40),
            init_p_times=kp.get("init_p_times", 20),
        ),
    )

    # 松弛查询
    lq = at.get("loose_query", {})
    result.loose_query = AirLooseQueryConfig(
        enabled=lq.get("enabled", True),
        max_lost_frames=lq.get("max_lost_frames", 5),
        expand_xy=lq.get("expand_xy", 1.0),
        expand_z=lq.get("expand_z", 0.5),
        eps_scale=lq.get("eps_scale", 1.4),
        min_samples_scale=lq.get("min_samples_scale", 0.6),
    )

    # 背景减除
    bg = at.get("background", {})
    result.background = AirBackgroundConfig(
        enabled=bg.get("enabled", True),
        source=bg.get("source", "map"),
        bg_threshold=bg.get("bg_threshold", 0.15),
        voxel_size=bg.get("voxel_size", 0.10),
        occupy_threshold=bg.get("occupy_threshold", 5),
        learning_frames=bg.get("learning_frames", 10),
        bg_pcd_file=bg.get("bg_pcd_file", ""),
    )

    # 多帧缓冲
    bu = at.get("buffer", {})
    result.buffer = AirBufferConfig(
        enabled=bu.get("enabled", False),
        frame_count=bu.get("frame_count", 3),
    )

    return result

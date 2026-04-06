"""
test_paths.py — 路径管理模块测试

验证 paths.py 中导出的路径常量是否正确、目录是否实际存在。
"""

import os

import pytest

from hnurm_radar.shared.paths import (
    BYTETRACK_CONFIG_PATH,
    CONFIGS_DIR,
    CONVERTER_CONFIG_PATH,
    DATA_DIR,
    DETECTOR_CONFIG_PATH,
    MAIN_CONFIG_PATH,
    MAP_DIR,
    PROJECT_ROOT,
    WEIGHTS_DIR,
)


class TestProjectRoot:
    """项目根目录测试"""

    @pytest.mark.unit
    def test_project_root_exists(self):
        """PROJECT_ROOT 应指向一个存在的目录"""
        assert os.path.isdir(PROJECT_ROOT), f"PROJECT_ROOT 不存在: {PROJECT_ROOT}"

    @pytest.mark.unit
    def test_project_root_contains_configs(self):
        """项目根目录下应有 configs/ 子目录"""
        assert os.path.isdir(os.path.join(PROJECT_ROOT, "configs"))

    @pytest.mark.unit
    def test_project_root_contains_src(self):
        """项目根目录下应有 src/ 子目录"""
        assert os.path.isdir(os.path.join(PROJECT_ROOT, "src"))

    @pytest.mark.unit
    def test_project_root_is_absolute(self):
        """PROJECT_ROOT 应为绝对路径"""
        assert os.path.isabs(PROJECT_ROOT)


class TestDirectoryPaths:
    """目录路径测试"""

    @pytest.mark.unit
    def test_configs_dir_exists(self):
        """CONFIGS_DIR 应存在"""
        assert os.path.isdir(CONFIGS_DIR), f"CONFIGS_DIR 不存在: {CONFIGS_DIR}"

    @pytest.mark.unit
    def test_data_dir_exists(self):
        """DATA_DIR 应存在"""
        assert os.path.isdir(DATA_DIR), f"DATA_DIR 不存在: {DATA_DIR}"

    @pytest.mark.unit
    def test_map_dir_under_data(self):
        """MAP_DIR 应在 DATA_DIR 之下"""
        assert MAP_DIR.startswith(DATA_DIR)

    @pytest.mark.unit
    def test_weights_dir_under_data(self):
        """WEIGHTS_DIR 应在 DATA_DIR 之下"""
        assert WEIGHTS_DIR.startswith(DATA_DIR)


class TestConfigPaths:
    """配置文件路径测试"""

    @pytest.mark.unit
    def test_main_config_exists(self):
        """main_config.yaml 应存在"""
        assert os.path.isfile(MAIN_CONFIG_PATH), (
            f"main_config.yaml 不存在: {MAIN_CONFIG_PATH}"
        )

    @pytest.mark.unit
    def test_detector_config_exists(self):
        """detector_config.yaml 应存在"""
        assert os.path.isfile(DETECTOR_CONFIG_PATH), (
            f"detector_config.yaml 不存在: {DETECTOR_CONFIG_PATH}"
        )

    @pytest.mark.unit
    def test_converter_config_exists(self):
        """converter_config.yaml 应存在"""
        assert os.path.isfile(CONVERTER_CONFIG_PATH), (
            f"converter_config.yaml 不存在: {CONVERTER_CONFIG_PATH}"
        )

    @pytest.mark.unit
    def test_bytetrack_config_exists(self):
        """bytetrack.yaml 应存在"""
        assert os.path.isfile(BYTETRACK_CONFIG_PATH), (
            f"bytetrack.yaml 不存在: {BYTETRACK_CONFIG_PATH}"
        )

    @pytest.mark.unit
    def test_all_configs_under_configs_dir(self):
        """所有配置文件路径应在 CONFIGS_DIR 之下"""
        for path in [MAIN_CONFIG_PATH, DETECTOR_CONFIG_PATH,
                     CONVERTER_CONFIG_PATH, BYTETRACK_CONFIG_PATH]:
            assert path.startswith(CONFIGS_DIR), (
                f"{path} 不在 CONFIGS_DIR ({CONFIGS_DIR}) 下"
            )


class TestPathConsistency:
    """路径一致性测试"""

    @pytest.mark.unit
    def test_no_double_separator(self):
        """路径中不应出现连续的路径分隔符"""
        all_paths = [
            PROJECT_ROOT, CONFIGS_DIR, DATA_DIR, MAP_DIR, WEIGHTS_DIR,
            MAIN_CONFIG_PATH, DETECTOR_CONFIG_PATH, CONVERTER_CONFIG_PATH,
        ]
        double_sep = os.sep + os.sep
        for p in all_paths:
            assert double_sep not in p, f"路径含连续分隔符: {p}"

    @pytest.mark.unit
    def test_no_dotdot_in_normalized(self):
        """规范化后的路径不应包含 '..'"""
        all_paths = [
            PROJECT_ROOT, CONFIGS_DIR, DATA_DIR, MAP_DIR, WEIGHTS_DIR,
        ]
        for p in all_paths:
            assert ".." not in p, f"路径含 '..': {p}"

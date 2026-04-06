"""
test_label_mappings.py — 装甲板标签映射测试

验证 Gray2Blue, Gray2Red, gray2gray 等映射字典的正确性和一致性。
纯数据测试，不依赖任何硬件或 ROS。
"""

import pytest

from hnurm_radar.detection.label_mappings import (
    Blue2Gray,
    Gray2Blue,
    Gray2Red,
    Red2Gray,
    gray2gray,
)


class TestGrayMappings:
    """灰色装甲板映射测试"""

    @pytest.mark.unit
    def test_gray2blue_keys(self):
        """Gray2Blue 的 key 应为灰色索引 12-17"""
        assert set(Gray2Blue.keys()) == {12, 13, 14, 15, 16, 17}

    @pytest.mark.unit
    def test_gray2red_keys(self):
        """Gray2Red 的 key 应为灰色索引 12-17"""
        assert set(Gray2Red.keys()) == {12, 13, 14, 15, 16, 17}

    @pytest.mark.unit
    def test_gray2blue_values_are_blue(self):
        """Gray2Blue 的 value 应为蓝方索引 0-5"""
        for v in Gray2Blue.values():
            assert 0 <= v <= 5, f"蓝方索引 {v} 超出范围 0-5"

    @pytest.mark.unit
    def test_gray2red_values_are_red(self):
        """Gray2Red 的 value 应为红方索引 6-11"""
        for v in Gray2Red.values():
            assert 6 <= v <= 11, f"红方索引 {v} 超出范围 6-11"

    @pytest.mark.unit
    def test_gray2blue_values_unique(self):
        """Gray2Blue 的 value 应唯一（每个灰色只映射到一个蓝方编号）"""
        values = list(Gray2Blue.values())
        assert len(values) == len(set(values)), "Gray2Blue 存在重复映射"

    @pytest.mark.unit
    def test_gray2red_values_unique(self):
        """Gray2Red 的 value 应唯一"""
        values = list(Gray2Red.values())
        assert len(values) == len(set(values)), "Gray2Red 存在重复映射"

    @pytest.mark.unit
    def test_blue_red_correspondence(self):
        """Gray2Blue 和 Gray2Red 应覆盖相同的灰色索引，且蓝红编号对应"""
        # 同一个灰色索引，蓝方编号+6 = 红方编号
        for gray_idx in Gray2Blue:
            blue_idx = Gray2Blue[gray_idx]
            red_idx = Gray2Red[gray_idx]
            assert red_idx == blue_idx + 6, (
                f"灰色索引 {gray_idx}: 蓝方 {blue_idx} + 6 ≠ 红方 {red_idx}"
            )


class TestReverseMapping:
    """反向映射测试"""

    @pytest.mark.unit
    def test_blue2gray_is_inverse(self):
        """Blue2Gray 应为 Gray2Blue 的反向映射"""
        for gray, blue in Gray2Blue.items():
            assert Blue2Gray[blue] == gray

    @pytest.mark.unit
    def test_red2gray_is_inverse(self):
        """Red2Gray 应为 Gray2Red 的反向映射"""
        for gray, red in Gray2Red.items():
            assert Red2Gray[red] == gray

    @pytest.mark.unit
    def test_blue2gray_size(self):
        """Blue2Gray 大小应与 Gray2Blue 一致"""
        assert len(Blue2Gray) == len(Gray2Blue)

    @pytest.mark.unit
    def test_red2gray_size(self):
        """Red2Gray 大小应与 Gray2Red 一致"""
        assert len(Red2Gray) == len(Gray2Red)


class TestGray2Gray:
    """Stage 3 内部索引映射测试"""

    @pytest.mark.unit
    def test_keys_range(self):
        """gray2gray 的 key 应为 Stage 3 输出的 0-5"""
        assert set(gray2gray.keys()) == {0, 1, 2, 3, 4, 5}

    @pytest.mark.unit
    def test_values_range(self):
        """gray2gray 的 value 应为全局灰色索引 12-17"""
        assert set(gray2gray.values()) == {12, 13, 14, 15, 16, 17}

    @pytest.mark.unit
    def test_bijective(self):
        """gray2gray 应为双射（一一对应）"""
        values = list(gray2gray.values())
        assert len(values) == len(set(values)), "gray2gray 不是双射"

    @pytest.mark.unit
    def test_composed_with_gray2blue(self):
        """gray2gray → Gray2Blue 组合后应覆盖所有蓝方编号"""
        composed = {k: Gray2Blue[v] for k, v in gray2gray.items()}
        assert set(composed.values()) == {0, 1, 2, 3, 4, 5}

"""
test_serial_protocol.py — 裁判系统串口帧协议测试

测试 CRC8/CRC16 校验、帧头/帧尾构造的正确性。
这些函数是纯计算函数，不依赖硬件或 ROS。
"""

import struct

import pytest

from hnurm_radar.communication.serial_protocol import (
    check_crc8,
    check_crc16,
    get_crc8_check_byte,
    get_crc16_check_byte,
    get_frame_header,
    get_frame_tail,
)

# ================================================================
# CRC8 测试
# ================================================================

class TestCRC8:
    """CRC8 校验函数测试"""

    @pytest.mark.unit
    def test_empty_data(self):
        """空数据的 CRC8 应该是确定值"""
        result = get_crc8_check_byte(b"")
        assert isinstance(result, int)
        assert 0 <= result <= 0xFF

    @pytest.mark.unit
    def test_single_byte(self):
        """单字节输入应返回有效 CRC8"""
        result = get_crc8_check_byte(b"\x00")
        assert isinstance(result, int)
        assert 0 <= result <= 0xFF

    @pytest.mark.unit
    def test_deterministic(self):
        """相同输入应产生相同 CRC8"""
        data = b"\xA5\x0E\x00\x00"
        assert get_crc8_check_byte(data) == get_crc8_check_byte(data)

    @pytest.mark.unit
    def test_different_input_different_crc(self):
        """不同输入应（大概率）产生不同 CRC8"""
        crc1 = get_crc8_check_byte(b"\x00\x01\x02")
        crc2 = get_crc8_check_byte(b"\x03\x04\x05")
        # 不同数据通常有不同 CRC，但理论上可能碰撞
        # 这里用两组差异明显的数据，碰撞概率 ~1/256
        assert crc1 != crc2

    @pytest.mark.unit
    def test_check_crc8_match(self):
        """check_crc8 对正确的 CRC 应返回 True"""
        data = b"\xA5\x0E\x00\x00"
        crc = get_crc8_check_byte(data)
        assert check_crc8(data, crc) is True

    @pytest.mark.unit
    def test_check_crc8_mismatch(self):
        """check_crc8 对错误的 CRC 应返回 False"""
        data = b"\xA5\x0E\x00\x00"
        crc = get_crc8_check_byte(data)
        wrong_crc = (crc + 1) & 0xFF
        assert check_crc8(data, wrong_crc) is False


# ================================================================
# CRC16 测试
# ================================================================

class TestCRC16:
    """CRC16 校验函数测试"""

    @pytest.mark.unit
    def test_empty_data(self):
        """空数据的 CRC16 应该是确定值"""
        result = get_crc16_check_byte(b"")
        assert isinstance(result, int)
        assert 0 <= result <= 0xFFFF

    @pytest.mark.unit
    def test_deterministic(self):
        """相同输入应产生相同 CRC16"""
        data = b"\xA5\x0E\x00\x00\xAB"
        assert get_crc16_check_byte(data) == get_crc16_check_byte(data)

    @pytest.mark.unit
    def test_check_crc16_match(self):
        """check_crc16 对正确的 CRC 应返回 True"""
        data = b"\xA5\x0E\x00\x00\xAB\x01\x02"
        crc16 = get_crc16_check_byte(data)
        crc16_bytes = struct.pack("<H", crc16)
        assert check_crc16(data, crc16_bytes) is True

    @pytest.mark.unit
    def test_check_crc16_mismatch(self):
        """check_crc16 对错误的 CRC 应返回 False"""
        data = b"\xA5\x0E\x00\x00\xAB\x01\x02"
        wrong_bytes = b"\x00\x00"
        assert check_crc16(data, wrong_bytes) is False


# ================================================================
# 帧头 / 帧尾测试
# ================================================================

class TestFrameConstruction:
    """帧头帧尾构造测试"""

    @pytest.mark.unit
    def test_frame_header_length(self):
        """帧头应为 5 字节"""
        header = get_frame_header(data_length=14)
        assert len(header) == 5

    @pytest.mark.unit
    def test_frame_header_sof(self):
        """帧头第 1 字节应为 SOF = 0xA5"""
        header = get_frame_header(data_length=14)
        assert header[0] == 0xA5

    @pytest.mark.unit
    def test_frame_header_data_length(self):
        """帧头第 2-3 字节应包含 data_length（小端）"""
        header = get_frame_header(data_length=14)
        data_len = struct.unpack("<H", header[1:3])[0]
        assert data_len == 14

    @pytest.mark.unit
    def test_frame_header_seq(self):
        """帧头第 4 字节应为 seq = 0"""
        header = get_frame_header(data_length=14)
        assert header[3] == 0x00

    @pytest.mark.unit
    def test_frame_header_crc8_valid(self):
        """帧头第 5 字节应为前 4 字节的 CRC8"""
        header = get_frame_header(data_length=14)
        expected_crc = get_crc8_check_byte(header[:4])
        assert header[4] == expected_crc

    @pytest.mark.unit
    def test_frame_header_different_length(self):
        """不同 data_length 应产生不同帧头"""
        h1 = get_frame_header(data_length=10)
        h2 = get_frame_header(data_length=20)
        assert h1 != h2

    @pytest.mark.unit
    def test_frame_tail_length(self):
        """帧尾应为 2 字节"""
        header = get_frame_header(data_length=2)
        cmd_id = struct.pack("<H", 0x0305)
        data = b"\x01\x02"
        tx_buff = header + cmd_id + data
        tail = get_frame_tail(tx_buff)
        assert len(tail) == 2

    @pytest.mark.unit
    def test_frame_round_trip(self):
        """构造帧头+数据+帧尾后，CRC16 校验应通过"""
        header = get_frame_header(data_length=4)
        cmd_id = struct.pack("<H", 0x0305)
        data = b"\x01\x02\x03\x04"
        tx_buff = header + cmd_id + data
        tail = get_frame_tail(tx_buff)
        # 验证帧尾 CRC16 与完整 tx_buff 匹配
        assert check_crc16(tx_buff, tail) is True

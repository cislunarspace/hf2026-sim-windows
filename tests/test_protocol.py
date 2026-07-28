"""tests/test_protocol.py — 通信协议编解码测试。

纯 ASCII 格式 "T:lat,lon"，50 bytes 总限制，每条目标 ~18 chars。
与 baseline coop_distributed 格式兼容。
"""
import pytest

from algorithms.coordination.protocol import (
    TargetInfo, decode_targets, encode_targets,
    encode_announce, decode_announce, parse_payload,
    _MAX_PAYLOAD_BYTES,
)


class TestTargetInfo:
    def test_create_valid(self):
        t = TargetInfo(type="T", target_id=1, lat=27.0, lon=125.0, status=1)
        assert t.type == "T"

    def test_invalid_type(self):
        with pytest.raises(ValueError):
            TargetInfo(type="X", target_id=0, lat=0.0, lon=0.0, status=0)

    def test_invalid_id(self):
        with pytest.raises(ValueError):
            TargetInfo(type="T", target_id=256, lat=0.0, lon=0.0, status=0)

    def test_invalid_status(self):
        with pytest.raises(ValueError):
            TargetInfo(type="T", target_id=0, lat=0.0, lon=0.0, status=3)


class TestEncodeDecode:
    def test_roundtrip_single(self):
        t = TargetInfo(type="T", target_id=42, lat=27.005, lon=125.005)
        payload = encode_targets([t])
        result = decode_targets(payload)
        assert len(result) == 1
        assert result[0].type == "T"
        assert abs(result[0].lat - 27.005) < 0.001
        assert abs(result[0].lon - 125.005) < 0.001

    def test_roundtrip_two_targets(self):
        targets = [
            TargetInfo(type="T", target_id=1, lat=27.0, lon=125.0),
            TargetInfo(type="D", target_id=2, lat=27.01, lon=125.01),
        ]
        payload = encode_targets(targets)
        assert len(payload.encode("utf-8")) <= _MAX_PAYLOAD_BYTES
        result = decode_targets(payload)
        assert len(result) == 2
        assert result[0].type == "T"
        assert result[1].type == "D"

    def test_roundtrip_negative_coords(self):
        t = TargetInfo(type="T", target_id=0, lat=-33.86, lon=-151.21)
        result = decode_targets(encode_targets([t]))
        assert abs(result[0].lat - (-33.86)) < 0.001
        assert abs(result[0].lon - (-151.21)) < 0.001

    def test_roundtrip_all_types(self):
        for t_type in "TDAJ":
            t = TargetInfo(type=t_type, target_id=0, lat=27.0, lon=125.0)
            result = decode_targets(encode_targets([t]))
            assert result[0].type == t_type

    def test_roundtrip_empty(self):
        assert decode_targets("") == []
        assert decode_targets(encode_targets([])) == []

    def test_baseline_compatible(self):
        """baseline 的 "T:lat,lon" 格式应能解码。"""
        result = decode_targets("T:27.00123,125.00456")
        assert len(result) == 1
        assert result[0].type == "T"
        assert abs(result[0].lat - 27.00123) < 0.001


class TestPayloadSize:
    def test_single_target_size(self):
        t = TargetInfo(type="T", target_id=0, lat=27.0, lon=125.0)
        payload = encode_targets([t])
        assert len(payload) < 20  # "T:27.000,125.000" = 18

    def test_two_targets_within_limit(self):
        targets = [
            TargetInfo(type="T", target_id=0, lat=27.0, lon=125.0),
            TargetInfo(type="D", target_id=1, lat=27.01, lon=125.01),
        ]
        payload = encode_targets(targets)
        assert len(payload.encode("utf-8")) <= _MAX_PAYLOAD_BYTES

    def test_pure_ascii(self):
        targets = [
            TargetInfo(type="T", target_id=0, lat=27.0, lon=125.0),
            TargetInfo(type="D", target_id=1, lat=27.01, lon=125.01),
        ]
        payload = encode_targets(targets)
        assert len(payload.encode("utf-8")) == len(payload)

    def test_too_many_targets_raises(self):
        """4 条目标超 50 bytes。"""
        targets = [
            TargetInfo(type="T", target_id=i, lat=27.0, lon=125.0)
            for i in range(4)
        ]
        with pytest.raises(ValueError, match="超限"):
            encode_targets(targets)

    def test_three_targets_exactly_fits(self):
        """3 条目标恰好 50 bytes（边界）。"""
        targets = [
            TargetInfo(type="T", target_id=i, lat=27.0, lon=125.0)
            for i in range(3)
        ]
        payload = encode_targets(targets)
        assert len(payload.encode("utf-8")) <= _MAX_PAYLOAD_BYTES


class TestForwardCompatibility:
    def test_unknown_type_skipped(self):
        result = decode_targets("T:27.000,125.000;X:27.000,125.000")
        assert len(result) == 1

    def test_malformed_entry_skipped(self):
        result = decode_targets("T:27.000,125.000;garbage;D:27.000,125.000")
        assert len(result) == 2

    def test_trailing_semicolon_ok(self):
        result = decode_targets("T:27.000,125.000;")
        assert len(result) == 1


class TestAnnounce:
    """announce 消息测试。"""

    def test_encode_announce(self):
        payload = encode_announce(27.005, 125.005)
        assert payload == "A:27.005,125.005"
        assert len(payload) < 20

    def test_decode_announce(self):
        result = decode_announce("A:27.005,125.005")
        assert result is not None
        lat, lon = result
        assert abs(lat - 27.005) < 1e-5
        assert abs(lon - 125.005) < 1e-5

    def test_decode_non_announce(self):
        assert decode_announce("T:27.000,125.000") is None
        assert decode_announce("") is None
        assert decode_announce("garbage") is None

    def test_roundtrip(self):
        payload = encode_announce(-33.86, 151.21)
        lat, lon = decode_announce(payload)
        assert abs(lat - (-33.86)) < 0.001
        assert abs(lon - 151.21) < 0.001


class TestParsePayload:
    """通用 payload 解析测试。"""

    def test_parse_single_target(self):
        msgs = parse_payload("T:27.000,125.000")
        assert len(msgs) == 1
        assert msgs[0] == ("T", 27.0, 125.0)

    def test_parse_announce(self):
        msgs = parse_payload("A:27.005,125.005")
        assert len(msgs) == 1
        assert msgs[0] == ("A", 27.005, 125.005)

    def test_parse_mixed(self):
        msgs = parse_payload("A:27.005,125.005;T:27.010,125.010")
        assert len(msgs) == 2
        assert msgs[0][0] == "A"
        assert msgs[1][0] == "T"

    def test_parse_empty(self):
        assert parse_payload("") == []

    def test_parse_malformed_ignored(self):
        msgs = parse_payload("T:27.000,125.000;garbage;D:27.000,125.000")
        assert len(msgs) == 2

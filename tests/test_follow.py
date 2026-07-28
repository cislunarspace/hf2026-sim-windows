"""tests/test_follow.py — 飞行跟随策略测试。"""

import pytest

try:
    from algorithms.tracking.follow import compute_lead_point
except ImportError:
    pytest.skip(
        "algorithms.tracking.follow 尚未实现，跳过测试", allow_module_level=True
    )


class TestComputeLeadPoint:
    """前馈飞行点计算测试。"""

    def test_static_target(self):
        """静止目标 → lead point = 目标位置。"""
        lead_lat, lead_lon = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=0.0,
            target_v_north=0.0,
            lead_time_s=2.0,
        )
        assert abs(lead_lat - 27.005) < 1e-6
        assert abs(lead_lon - 125.005) < 1e-6

    def test_eastbound_target(self):
        """向东运动目标 → lead point 在目标东侧。"""
        lead_lat, lead_lon = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=8.0,
            target_v_north=0.0,
            lead_time_s=2.0,
        )
        assert lead_lon > 125.005, f"lead_lon={lead_lon:.6f}，应 > 125.005（向东）"
        assert abs(lead_lat - 27.005) < 1e-4, "纬度应基本不变"

    def test_northbound_target(self):
        """向北运动目标 → lead point 在目标北侧。"""
        lead_lat, _lead_lon = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=0.0,
            target_v_north=8.0,
            lead_time_s=2.0,
        )
        assert lead_lat > 27.005, f"lead_lat={lead_lat:.6f}，应 > 27.005（向北）"

    def test_longer_lead_time(self):
        """更长的 lead time → 更大的偏移。"""
        _lead_lat_1, lead_lon_1 = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=8.0,
            target_v_north=0.0,
            lead_time_s=1.0,
        )
        _lead_lat_2, lead_lon_2 = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=8.0,
            target_v_north=0.0,
            lead_time_s=3.0,
        )
        assert lead_lon_2 > lead_lon_1, "更长 lead time 应产生更大偏移"

    def test_zero_lead_time(self):
        """lead_time=0 → lead point = 目标位置。"""
        lead_lat, lead_lon = compute_lead_point(
            target_lat=27.005,
            target_lon=125.005,
            target_v_east=8.0,
            target_v_north=8.0,
            lead_time_s=0.0,
        )
        assert abs(lead_lat - 27.005) < 1e-6
        assert abs(lead_lon - 125.005) < 1e-6

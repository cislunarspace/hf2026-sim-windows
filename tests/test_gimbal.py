"""tests/test_gimbal.py — 云台控制测试。"""

import pytest

try:
    from algorithms.tracking.gimbal import choose_fov, compute_gimbal_angles
except ImportError:
    pytest.skip(
        "algorithms.tracking.gimbal 尚未实现，跳过测试", allow_module_level=True
    )


class TestComputeGimbalAngles:
    """LOS 角度计算测试。"""

    def test_due_north(self):
        """目标在正北 → pan ≈ 0°，tilt < 0（向下看）。"""
        pan, tilt = compute_gimbal_angles(
            uav_lat=27.0,
            uav_lon=125.0,
            uav_alt=300.0,
            target_lat=27.005,
            target_lon=125.0,
            target_alt=0.0,
        )
        assert abs(pan) < 5.0, f"pan={pan:.1f}°，应接近 0°"
        assert tilt < 0, f"tilt={tilt:.1f}°，应 < 0°（向下看）"

    def test_due_east(self):
        """目标在正东 → pan ≈ 90°。"""
        pan, _tilt = compute_gimbal_angles(
            uav_lat=27.0,
            uav_lon=125.0,
            uav_alt=300.0,
            target_lat=27.0,
            target_lon=125.005,
            target_alt=0.0,
        )
        assert abs(pan - 90.0) < 5.0, f"pan={pan:.1f}°，应接近 90°"

    def test_due_south(self):
        """目标在正南 → pan ≈ ±180°。"""
        pan, _tilt = compute_gimbal_angles(
            uav_lat=27.005,
            uav_lon=125.0,
            uav_alt=300.0,
            target_lat=27.0,
            target_lon=125.0,
            target_alt=0.0,
        )
        assert abs(abs(pan) - 180.0) < 5.0, f"pan={pan:.1f}°，应接近 ±180°"

    def test_below_uav(self):
        """目标在 UAV 正下方 → tilt ≈ -90°。"""
        _pan, tilt = compute_gimbal_angles(
            uav_lat=27.0,
            uav_lon=125.0,
            uav_alt=300.0,
            target_lat=27.0,
            target_lon=125.0,
            target_alt=0.0,
        )
        assert abs(tilt - (-90.0)) < 5.0, f"tilt={tilt:.1f}°，应接近 -90°"

    def test_far_target_low_tilt(self):
        """远处目标 → tilt 接近 0°（平视）。"""
        _pan, tilt = compute_gimbal_angles(
            uav_lat=27.0,
            uav_lon=125.0,
            uav_alt=300.0,
            target_lat=27.1,
            target_lon=125.1,
            target_alt=0.0,
        )
        assert abs(tilt) < 20.0, f"远处目标 tilt={tilt:.1f}°，应接近 0°"

    def test_pan_range(self):
        """pan 在 [-180, 180] 范围内。"""
        for dlat, dlon in [(0.001, 0), (0, 0.001), (-0.001, 0), (0, -0.001)]:
            pan, _tilt = compute_gimbal_angles(
                uav_lat=27.0,
                uav_lon=125.0,
                uav_alt=300.0,
                target_lat=27.0 + dlat,
                target_lon=125.0 + dlon,
                target_alt=0.0,
            )
            assert -180.0 <= pan <= 180.0, f"pan={pan:.1f}° 超出范围"


class TestChooseFov:
    """FOV 选择测试。"""

    def test_search_mode_wide(self):
        """搜索模式应使用宽 FOV。"""
        fov = choose_fov(mode="search")
        assert fov >= 60.0, f"搜索 FOV={fov:.0f}°，应 >= 60°"

    def test_track_mode_narrow(self):
        """跟踪模式应使用窄 FOV。"""
        fov = choose_fov(mode="track")
        assert fov <= 30.0, f"跟踪 FOV={fov:.0f}°，应 <= 30°"

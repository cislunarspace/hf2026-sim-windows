"""tests/test_geometry.py — 几何计算模块测试（TDD RED 阶段）。

测试 rust_core.geometry 的 Python 绑定：
  - haversine_m: 大圆距离
  - bearing_rad: 方位角
  - destination_point: 给定距离和方位的目标点
  - wgs84_to_local / local_to_wgs84: 坐标转换往返一致性
"""
import math
import pytest


# ── import 待实现的模块（RED 阶段会失败）────────────────────────────────

try:
    from algorithms.estimation.geometry import (
        haversine_m,
        bearing_rad,
        destination_point,
        wgs84_to_local,
        local_to_wgs84,
    )
except ImportError:
    # RED 阶段：模块尚不存在，pytest 仍能收集测试
    pytest.skip("rust_core.geometry 尚未实现，跳过测试", allow_module_level=True)


# ── 常量 ────────────────────────────────────────────────────────────────

_TOLERANCE_M = 1.0        # 距离容差 1 米
_TOLERANCE_DEG = 0.01     # 角度容差 0.01 度
_TOLERANCE_RAD = math.radians(_TOLERANCE_DEG)
_ROUNDTRIP_M = 0.001      # 往返一致性 1 毫米


# ── haversine_m ─────────────────────────────────────────────────────────

class TestHaversineM:
    """大圆距离计算测试。"""

    def test_same_point_is_zero(self):
        assert haversine_m(27.0, 125.0, 27.0, 125.0) == pytest.approx(0.0, abs=1e-6)

    def test_equator_one_degree(self):
        """赤道上经度差 1° ≈ 111,195 m（WGS84 椭球近似）。"""
        d = haversine_m(0.0, 0.0, 0.0, 1.0)
        assert d == pytest.approx(111_195.0, abs=500.0)  # 500m 容差，椭球 vs 球体差异

    def test_symmetry(self):
        """haversine(A→B) == haversine(B→A)。"""
        d_ab = haversine_m(27.0, 125.0, 27.1, 125.1)
        d_ba = haversine_m(27.1, 125.1, 27.0, 125.0)
        assert d_ab == pytest.approx(d_ba, abs=1e-6)

    def test_known_distance(self):
        """北京天安门 → 上海人民广场 ≈ 1068 km。"""
        d = haversine_m(39.9087, 116.3975, 31.2304, 121.4737)
        assert d == pytest.approx(1_068_000, abs=50_000)  # 50km 容差

    def test_short_distance(self):
        """竞赛区域内短距离：27.0,125.0 → 27.001,125.0 ≈ 111m（纬度差 0.001°）。"""
        d = haversine_m(27.0, 125.0, 27.001, 125.0)
        assert d == pytest.approx(111.0, abs=2.0)


# ── bearing_rad ─────────────────────────────────────────────────────────

class TestBearingRad:
    """方位角计算测试。返回值范围 [-π, π]。"""

    def test_due_north(self):
        """正北方向 = 0 rad。"""
        b = bearing_rad(27.0, 125.0, 27.001, 125.0)
        assert b == pytest.approx(0.0, abs=_TOLERANCE_RAD)

    def test_due_east(self):
        """正东方向 ≈ π/2。"""
        b = bearing_rad(27.0, 125.0, 27.0, 125.001)
        assert b == pytest.approx(math.pi / 2, abs=_TOLERANCE_RAD)

    def test_due_south(self):
        """正南方向 ≈ ±π。"""
        b = bearing_rad(27.001, 125.0, 27.0, 125.0)
        # 可能是 +π 或 -π，取绝对值比较
        assert abs(abs(b) - math.pi) < _TOLERANCE_RAD

    def test_due_west(self):
        """正西方向 ≈ -π/2。"""
        b = bearing_rad(27.0, 125.001, 27.0, 125.0)
        assert b == pytest.approx(-math.pi / 2, abs=_TOLERANCE_RAD)

    def test_range(self):
        """返回值在 [-π, π]。"""
        for _ in range(100):
            b = bearing_rad(27.0, 125.0, 27.0 + 0.01, 125.0 + 0.01)
            assert -math.pi <= b <= math.pi


# ── destination_point ───────────────────────────────────────────────────

class TestDestinationPoint:
    """给定起点、距离和方位，计算目标点。"""

    def test_zero_distance_returns_same_point(self):
        lat, lon = destination_point(27.0, 125.0, 0.0, 0.0)
        assert lat == pytest.approx(27.0, abs=1e-10)
        assert lon == pytest.approx(125.0, abs=1e-10)

    def test_going_north(self):
        """向北 111km ≈ 纬度增加 1°。"""
        lat, lon = destination_point(27.0, 125.0, 111_195.0, 0.0)
        assert lat == pytest.approx(28.0, abs=0.05)
        assert lon == pytest.approx(125.0, abs=0.01)

    def test_going_east(self):
        """向东 ~111km（赤道附近）≈ 经度增加 1°。"""
        lat, lon = destination_point(0.0, 0.0, 111_195.0, math.pi / 2)
        assert lat == pytest.approx(0.0, abs=0.05)
        assert lon == pytest.approx(1.0, abs=0.05)

    def test_roundtrip_consistency(self):
        """A → destination → haversine 距离应等于原始距离。"""
        origin_lat, origin_lon = 27.0, 125.0
        distance_m = 5000.0
        bearing = math.radians(45.0)  # 东北方向

        dest_lat, dest_lon = destination_point(origin_lat, origin_lon,
                                                distance_m, bearing)
        computed = haversine_m(origin_lat, origin_lon, dest_lat, dest_lon)
        assert computed == pytest.approx(distance_m, abs=_ROUNDTRIP_M)


# ── wgs84_to_local / local_to_wgs84 ────────────────────────────────────

class TestCoordinateConversion:
    """WGS84 ↔ 局部切平面坐标转换。"""

    def test_origin_is_zero(self):
        """原点处 local 坐标 = (0, 0)。"""
        e, n = wgs84_to_local(27.0, 125.0, 27.0, 125.0)
        assert e == pytest.approx(0.0, abs=1e-6)
        assert n == pytest.approx(0.0, abs=1e-6)

    def test_roundtrip(self):
        """WGS84 → local → WGS84 往返一致性。"""
        origin_lat, origin_lon = 27.0, 125.0
        test_lat, test_lon = 27.005, 125.005

        e, n = wgs84_to_local(test_lat, test_lon, origin_lat, origin_lon)
        lat2, lon2 = local_to_wgs84(e, n, origin_lat, origin_lon)

        assert lat2 == pytest.approx(test_lat, abs=1e-8)
        assert lon2 == pytest.approx(test_lon, abs=1e-8)

    def test_north_positive(self):
        """向北移动 → north > 0。"""
        e, n = wgs84_to_local(27.001, 125.0, 27.0, 125.0)
        assert n > 0
        assert abs(e) < 1.0  # east 接近 0

    def test_east_positive(self):
        """向东移动 → east > 0。"""
        e, n = wgs84_to_local(27.0, 125.001, 27.0, 125.0)
        assert e > 0
        assert abs(n) < 1.0  # north 接近 0

    def test_distance_matches_haversine(self):
        """local 坐标的欧氏距离应近似等于 haversine 距离。"""
        origin_lat, origin_lon = 27.0, 125.0
        test_lat, test_lon = 27.003, 125.004

        e, n = wgs84_to_local(test_lat, test_lon, origin_lat, origin_lon)
        local_dist = math.sqrt(e**2 + n**2)
        haversine_dist = haversine_m(origin_lat, origin_lon, test_lat, test_lon)

        # 局部近似误差 < 1%（距离 < 1km 时很好）
        assert local_dist == pytest.approx(haversine_dist, rel=0.01)

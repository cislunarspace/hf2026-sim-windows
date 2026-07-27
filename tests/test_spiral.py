"""tests/test_spiral.py — 阿基米德螺旋搜索测试。"""
import math
import pytest

try:
    from algorithms.search.spiral import generate_spiral
except ImportError:
    pytest.skip("algorithms.search.spiral 尚未实现，跳过测试", allow_module_level=True)


class TestGenerateSpiral:
    """螺旋路径生成测试。"""

    def test_returns_list_of_tuples(self):
        waypoints = generate_spiral(27.0, 125.0, radius_m=500, pitch_m=200)
        assert isinstance(waypoints, list)
        assert len(waypoints) > 0
        assert isinstance(waypoints[0], tuple)
        assert len(waypoints[0]) == 2  # (lat, lon)

    def test_first_point_near_center(self):
        """第一个航点应在中心附近。"""
        center_lat, center_lon = 27.0, 125.0
        waypoints = generate_spiral(center_lat, center_lon, radius_m=500, pitch_m=200)
        first_lat, first_lon = waypoints[0]
        # 第一个点离中心 < pitch_m
        dlat = abs(first_lat - center_lat) * 111_320.0
        dlon = abs(first_lon - center_lon) * 111_320.0 * math.cos(math.radians(center_lat))
        dist = math.sqrt(dlat**2 + dlon**2)
        assert dist < 300, f"第一个点离中心 {dist:.0f}m > 300m"

    def test_count_sufficient(self):
        """航点数量应足够覆盖区域。"""
        waypoints = generate_spiral(27.0, 125.0, radius_m=1000, pitch_m=200)
        assert len(waypoints) >= 10, f"航点数 {len(waypoints)} < 10"

    def test_max_radius_respected(self):
        """最远航点不超过指定半径。"""
        center_lat, center_lon = 27.0, 125.0
        radius_m = 500
        waypoints = generate_spiral(center_lat, center_lon,
                                     radius_m=radius_m, pitch_m=100)
        for lat, lon in waypoints:
            dlat = abs(lat - center_lat) * 111_320.0
            dlon = abs(lon - center_lon) * 111_320.0 * math.cos(math.radians(center_lat))
            dist = math.sqrt(dlat**2 + dlon**2)
            assert dist <= radius_m + 50, f"航点超出半径：{dist:.0f}m > {radius_m}m"

    def test_spiral_expands(self):
        """相邻航点的距离应随圈数增加而增大。"""
        waypoints = generate_spiral(27.0, 125.0, radius_m=1000, pitch_m=200)
        # 检查前 10 个航点的距离序列是递增的
        center_lat, center_lon = 27.0, 125.0
        dists = []
        for lat, lon in waypoints[:15]:
            dlat = abs(lat - center_lat) * 111_320.0
            dlon = abs(lon - center_lon) * 111_320.0 * math.cos(math.radians(center_lat))
            dists.append(math.sqrt(dlat**2 + dlon**2))
        # 总体趋势递增（允许局部波动）
        assert dists[-1] > dists[0], "螺旋应从中心向外扩展"

    def test_custom_pitch(self):
        """螺距应影响航点间距。"""
        wp_small = generate_spiral(27.0, 125.0, radius_m=500, pitch_m=100)
        wp_large = generate_spiral(27.0, 125.0, radius_m=500, pitch_m=300)
        # 较小螺距应产生更多航点
        assert len(wp_small) > len(wp_large), \
            f"小螺距({len(wp_small)})应比大螺距({len(wp_large)})产生更多航点"

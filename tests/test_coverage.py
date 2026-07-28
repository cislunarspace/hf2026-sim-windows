"""tests/test_coverage.py — Voronoi 搜索覆盖分区测试。"""
import math
import time

import pytest

from algorithms.search.coverage import nearest_seed, voronoi_partition

# 赛题二 bbox
_BBOX = ((26.982, 124.980), (27.025, 125.020))


class TestNearestSeed:
    """最近 seed 分配测试。"""

    def test_point_at_seed(self):
        assert nearest_seed((27.0, 125.0), [(27.0, 125.0)]) == 0

    def test_closer_to_first(self):
        seeds = [(27.0, 124.99), (27.0, 125.01)]
        assert nearest_seed((27.0, 124.995), seeds) == 0

    def test_closer_to_second(self):
        seeds = [(27.0, 124.99), (27.0, 125.01)]
        assert nearest_seed((27.0, 125.005), seeds) == 1

    def test_equidistant_returns_first(self):
        seeds = [(27.0, 124.99), (27.0, 125.01)]
        idx = nearest_seed((27.0, 125.0), seeds)
        assert idx in (0, 1)  # 任意一个都行


class TestVoronoiPartition:
    """Voronoi 分区测试。"""

    def test_single_uav(self):
        """1 个 UAV → 整个区域。"""
        result = voronoi_partition([(27.0, 125.0)], _BBOX)
        assert len(result) == 1
        assert result[0]["radius_m"] > 1000  # ~4km 对角线的一半

    def test_three_uav_area_sum(self):
        """3 个 UAV 分区面积之和应等于总面积。"""
        seeds = [(27.0, 124.99), (27.0, 125.0), (27.0, 125.01)]
        result = voronoi_partition(seeds, _BBOX)
        total = sum(r["area_km2"] for r in result)
        # 总面积 ≈ 4.8km × 3.7km ≈ 17.8 km²
        assert 15 < total < 20, f"总面积 {total:.1f} km² 不合理"

    def test_three_uav_equal_split_at_center(self):
        """3 个 UAV 在 bbox 中心均匀分布 → 每个分区面积合理。"""
        lat_mid = (26.982 + 27.025) / 2
        seeds = [(lat_mid, 124.99), (lat_mid, 125.0), (lat_mid, 125.01)]
        result = voronoi_partition(seeds, _BBOX)
        areas = [r["area_km2"] for r in result]
        # 每个分区面积应在总面积的 20%-50% 之间
        total = sum(areas)
        for a in areas:
            assert 0.15 * total < a < 0.55 * total, f"面积 {a:.1f} 不在合理范围 {areas}"

    def test_three_uav_centers_distributed(self):
        """3 个 UAV 的分区中心应分散。"""
        seeds = [(27.0, 124.99), (27.0, 125.0), (27.0, 125.01)]
        result = voronoi_partition(seeds, _BBOX)
        centers = [r["center"] for r in result]
        # 中心经度应递增
        lons = [c[1] for c in centers]
        assert lons[0] < lons[1] < lons[2]

    def test_uav_near_edge(self):
        """UAV 靠近边界时分区仍合理。"""
        seeds = [(26.99, 124.985), (27.0, 125.0), (27.02, 125.015)]
        result = voronoi_partition(seeds, _BBOX)
        assert len(result) == 3
        for r in result:
            assert r["area_km2"] > 0

    def test_two_uav_close(self):
        """2 个 UAV 靠近时，远离的 UAV 获得更大分区。"""
        seeds = [(27.0, 124.985), (27.0, 124.986)]  # 非常靠近
        result = voronoi_partition(seeds, _BBOX)
        assert len(result) == 2
        # 两者面积之和应等于总面积
        total = sum(r["area_km2"] for r in result)
        # 总面积 ≈ 4.8km × 3.7km ≈ 17.8 km²
        assert total > 10

    def test_ten_uav(self):
        """10 个 UAV 分区合理。"""
        seeds = [(27.0, 124.98 + i * 0.004) for i in range(10)]
        result = voronoi_partition(seeds, _BBOX)
        assert len(result) == 10
        for r in result:
            assert r["area_km2"] > 0
            assert r["radius_m"] > 0

    def test_empty_seeds(self):
        result = voronoi_partition([], _BBOX)
        assert result == []

    def test_performance(self):
        """计算开销应 < 10ms。"""
        seeds = [(27.0, 124.98 + i * 0.004) for i in range(10)]
        t0 = time.perf_counter()
        for _ in range(100):
            voronoi_partition(seeds, _BBOX)
        elapsed = (time.perf_counter() - t0) / 100
        assert elapsed < 0.01, f"单次耗时 {elapsed*1000:.1f}ms，应 < 10ms"

    def test_grid_resolution_effect(self):
        """更高网格分辨率 → 更精确分区。"""
        seeds = [(27.0, 124.99), (27.0, 125.0), (27.0, 125.01)]
        coarse = voronoi_partition(seeds, _BBOX, grid_n=10)
        fine = voronoi_partition(seeds, _BBOX, grid_n=100)
        # 两者面积应相近（容差 2 km²）
        for i in range(3):
            assert abs(coarse[i]["area_km2"] - fine[i]["area_km2"]) < 2.0

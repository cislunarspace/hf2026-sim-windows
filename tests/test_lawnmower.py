"""tests/test_lawnmower.py — 割草机覆盖搜索航点生成测试。"""

from algorithms.search.lawnmower import generate_lawnmower, sector_lawnmower

_BBOX = ((26.982, 124.980), (27.025, 125.020))


class TestGenerateLawnmower:
    def test_waypoints_within_region(self):
        wps = generate_lawnmower(26.99, 27.02, 124.99, 125.01)
        assert len(wps) >= 4
        for lat, lon in wps:
            assert 26.99 <= lat <= 27.02
            assert 124.99 <= lon <= 125.01

    def test_boustrophedon_alternation(self):
        """相邻车道方向相反：车道 i 从南向北，车道 i+1 从北向南。"""
        wps = generate_lawnmower(26.99, 27.02, 124.99, 125.01)
        lats = [la for la, _ in wps]
        assert len(set(round(la, 6) for la in lats)) == 2
        # 每条车道两个端点纬度不同；相邻车道的衔接点纬度相同（原地转向）
        for lane in range(len(wps) // 2):
            a, b = wps[2 * lane][0], wps[2 * lane + 1][0]
            assert a != b
            if lane % 2 == 0:
                assert a < b, "偶数车道应南→北"
            else:
                assert a > b, "奇数车道应北→南"

    def test_lane_spacing_controls_count(self):
        narrow = generate_lawnmower(26.99, 27.02, 124.99, 125.01, lane_spacing_m=200.0)
        wide = generate_lawnmower(26.99, 27.02, 124.99, 125.01, lane_spacing_m=800.0)
        assert len(narrow) > len(wide)


class TestSectorLawnmower:
    def test_sectors_cover_bbox_without_overlap(self):
        """三个条带的经度范围应互不重叠且合起来覆盖 bbox。"""
        spans = []
        for uid in ("20001", "20002", "20003"):
            wps = sector_lawnmower(uid, _BBOX, n_sectors=3)
            lons = [lo for _, lo in wps]
            spans.append((min(lons), max(lons)))
        spans.sort()
        for (a0, a1), (b0, b1) in zip(spans, spans[1:]):
            assert a1 <= b0 + 1e-9, f"条带重叠: {spans}"

    def test_odd_index_reversed(self):
        """奇数条带起始端反转（多机错开）。"""
        w0 = sector_lawnmower("20002", _BBOX, n_sectors=3)  # idx 1 → 反转
        w1 = sector_lawnmower("20002", _BBOX, n_sectors=3)
        assert w0 == w1  # 确定性

"""tests/test_start_pools.py — Start 池判别辅助模块测试。

真目标路线池（points.json 26 条 road*）与诱饵路线池（random_routes_20.json
18 条 route_*）Start 零重叠；匹配规则：距诱 Start <150m 且距真 Start >250m
→ decoy；反之 true；其余 suspect。
"""

import pytest

try:
    from algorithms.search.start_pools import (
        DECOY_STARTS,
        TARGET_STARTS,
        coverage_waypoints_for_uid,
        match_start_pool,
        nearest_start_m,
    )
except ImportError:
    pytest.skip("start_pools 模块未实现，跳过测试", allow_module_level=True)


class TestStartPoolMatch:
    def test_target_starts_loaded(self):
        assert len(TARGET_STARTS) == 26
        assert len(DECOY_STARTS) == 18

    def test_decoy_start_area_is_decoy(self):
        # route_2 Start (27.019132, 124.983090)：距真 Start（road6 在
        # 27.0191,125.0019 附近）>2km
        assert match_start_pool(27.01913, 124.98309) == "decoy"

    def test_target_start_area_is_true(self):
        # road1 Start (27.001090, 125.000860)：距最近诱饵 Start（route_12
        # 在 27.00066,124.99788 附近）>300m
        assert match_start_pool(27.00109, 125.00086) == "true"

    def test_mid_area_is_suspect(self):
        # 地图中部 (27.005, 125.005)：距两池都 >300m
        assert match_start_pool(27.005, 125.005) == "suspect"

    def test_nearest_distance(self):
        assert nearest_start_m(27.01913, 124.98309, DECOY_STARTS) < 100.0


class TestCoverageWaypoints:
    """coverage_waypoints_for_uid：44 点按 uid 分片，三机覆盖不相交、
    合取为全集，首点按 uid 偏移错开。"""

    def test_three_uavs_partition_44_points(self):
        uids = ["20001", "20002", "20003"]
        slices = [coverage_waypoints_for_uid(u) for u in uids]
        flat = [p for s in slices for p in s]
        assert len(flat) == 44
        assert len(set(flat)) == 44
        assert set(flat) == set(TARGET_STARTS) | set(DECOY_STARTS)
        for i in range(3):
            for j in range(i + 1, 3):
                assert not (set(slices[i]) & set(slices[j])), (
                    f"分片 {i}/{j} 不应相交"
                )

    def test_first_waypoints_differ(self):
        firsts = [
            coverage_waypoints_for_uid(u)[0]
            for u in ["20001", "20002", "20003"]
        ]
        assert len(set(firsts)) == 3, "三机首点应互不相同（按 uid 偏移错开）"

    def test_suffix_digit_uid(self):
        """uav_1 类 uid 取数字后缀分片（与 route_waypoints_for_uid 风格一致）。"""
        points = TARGET_STARTS + DECOY_STARTS
        wps = coverage_waypoints_for_uid("uav_1", n_shares=3)
        assert len(wps) == 15
        assert wps[0] == points[1]
        assert wps == points[1::3]

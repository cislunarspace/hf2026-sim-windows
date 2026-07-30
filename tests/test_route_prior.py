"""tests/test_route_prior.py — 路线先验匹配与位置预测测试。"""

import math

import pytest

from algorithms.search.route_prior import ROUTES, _dist_m, match_route, predict_position


class TestMatchRoute:
    def test_every_start_matches_own_route(self):
        """26 条路线的 Start 都应能在 100m 容差内匹配上（返回某条路线）。"""
        for i, route in enumerate(ROUTES):
            idx = match_route(route[0][0], route[0][1])
            assert idx is not None, f"route {i} 的 Start 未匹配"
            # 匹配到的 Start 应贴近（同名 0m；个别路线 Start 相距 15m 也接受）
            d = _dist_m((route[0][0], route[0][1]),
                        (ROUTES[idx][0][0], ROUTES[idx][0][1]))
            assert d < 100.0

    def test_far_point_returns_none(self):
        """远离所有路线 Start 的点应返回 None（验证集新路线回退）。"""
        assert match_route(28.5, 126.5) is None


class TestPredictPosition:
    def test_t_zero_at_start(self):
        """t=0 应在路线 Start。"""
        for i in (0, 5, 25):
            lat, lon = predict_position(i, 0.0, 8.0)
            assert _dist_m((lat, lon), (ROUTES[i][0][0], ROUTES[i][0][1])) < 1.0

    def test_wait_time_holds_at_start(self):
        """Start.WaitTime=30s 内目标不动（出生点停驶是上报满分窗口）。"""
        for i in (0, 5, 25):
            lat, lon = predict_position(i, 29.9, 8.0)
            assert _dist_m((lat, lon), (ROUTES[i][0][0], ROUTES[i][0][1])) < 1.0

    def test_advances_along_first_segment(self):
        """等待结束后按速度沿路线前进：30s 等待 + 80m 行程应离开 Start 约 80m。"""
        i = 0
        # 第一段长度需大于 80m，否则断言无意义
        seg = _dist_m((ROUTES[i][0][0], ROUTES[i][0][1]),
                      (ROUTES[i][1][0], ROUTES[i][1][1]))
        assert seg > 80.0, "测试假设第一段超过 80m"
        lat, lon = predict_position(i, 30.0 + 10.0, 8.0)
        d = _dist_m((lat, lon), (ROUTES[i][0][0], ROUTES[i][0][1]))
        assert math.isclose(d, 80.0, abs_tol=1.0)

    def test_clamps_at_end(self):
        """时间足够长后停在路线 End。"""
        for i in (0, 25):
            lat, lon = predict_position(i, 1e6, 8.0)
            assert _dist_m((lat, lon), (ROUTES[i][-1][0], ROUTES[i][-1][1])) < 1.0

    def test_route_has_wait_fields(self):
        """烘焙数据必须含 WaitTime（所有路线 Start 停 30s，已核实 points.json）。"""
        for route in ROUTES:
            assert all(len(p) == 3 for p in route)
            assert route[0][2] == pytest.approx(30.0)


class TestPredictVelocity:
    def test_zero_during_start_wait(self):
        """Start 停驶期间先验速度应为 0。"""
        from algorithms.search.route_prior import predict_velocity

        ve, vn = predict_velocity(0, 10.0, 8.0)
        assert ve == 0.0 and vn == 0.0

    def test_matches_speed_along_route(self):
        """行驶中先验速度模长应接近设定速度。"""
        from algorithms.search.route_prior import predict_velocity

        ve, vn = predict_velocity(0, 60.0, 8.0)
        assert math.isclose(math.hypot(ve, vn), 8.0, abs_tol=0.1)

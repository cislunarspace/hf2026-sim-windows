"""tests/test_ekf.py — Bearing-Only EKF 测试。

测试 rust_core.ekf.BearingOnlyEKF：
  - 初始化与状态查询
  - 静止目标收敛（UAV 盘旋）
  - 匀速运动目标跟踪（UAV 也在运动）
  - 缺失观测容错
  - 速度估计方向
"""
import math
import pytest

try:
    from algorithms.estimation.ekf import BearingOnlyEKF
    from algorithms.estimation.geometry import (
        haversine_m, bearing_rad, wgs84_to_local, local_to_wgs84,
    )
except ImportError:
    pytest.skip("rust_core.ekf 尚未实现，跳过测试", allow_module_level=True)


# ── 辅助函数 ────────────────────────────────────────────────────────────

def _simulate_detection(uav_lat, uav_lon, target_lat, target_lon):
    """模拟一帧检测：返回 (bearing_rad, range_m)。"""
    b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
    r = haversine_m(uav_lat, uav_lon, target_lat, target_lon)
    return b, r


# ── 初始化 ──────────────────────────────────────────────────────────────

class TestEKFInit:
    """EKF 初始化测试。"""

    def test_not_initialized_before_init(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        assert not ekf.is_initialized()

    def test_initialized_after_init(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        assert ekf.is_initialized()

    def test_position_after_init(self):
        """初始化后位置应在 UAV 前方 assumed_range 处。"""
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        lat, lon = ekf.position_wgs84()
        assert lat > 27.0
        assert abs(lat - 27.0045) < 0.001
        assert abs(lon - 125.0) < 0.001

    def test_velocity_zero_after_init(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        ve, vn = ekf.velocity_mps()
        assert abs(ve) < 1e-6
        assert abs(vn) < 1e-6

    def test_uncertainty_large_after_init(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        assert ekf.position_uncertainty_m() > 50.0


# ── 静止目标收敛 ────────────────────────────────────────────────────────

class TestEKFStationaryConvergence:
    """给静止目标多帧观测，UAV 做螺旋/盘旋运动，位置应收敛、不确定性递减。

    关键：bearing-only 距离可观性要求 UAV 持续转弯（提供观测几何变化）。
    """

    def test_position_converges(self):
        origin_lat, origin_lon = 27.0, 125.0
        target_lat, target_lon = 27.005, 125.005  # ~550m 东北
        ekf = BearingOnlyEKF(origin_lat, origin_lon)

        b, r = _simulate_detection(origin_lat, origin_lon, target_lat, target_lon)
        ekf.initialize(origin_lat, origin_lon, b, r)

        initial_uncertainty = ekf.position_uncertainty_m()

        # 模拟 UAV 做盘旋（半径 ~150m），提供观测几何变化
        dt = 0.1
        orbit_radius_deg = 0.001  # ~111m
        for i in range(100):
            angle = i * 0.05  # 约 5 rad/s 盘旋
            uav_lat = origin_lat + orbit_radius_deg * math.sin(angle)
            uav_lon = origin_lon + orbit_radius_deg * math.cos(angle)
            b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)

        est_lat, est_lon = ekf.position_wgs84()
        error_m = haversine_m(est_lat, est_lon, target_lat, target_lon)

        assert error_m < 50.0, f"误差 {error_m:.1f}m > 50m"
        assert ekf.position_uncertainty_m() < initial_uncertainty

    def test_uncertainty_decreases(self):
        origin_lat, origin_lon = 27.0, 125.0
        target_lat, target_lon = 27.003, 125.0
        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        b, r = _simulate_detection(origin_lat, origin_lon, target_lat, target_lon)
        ekf.initialize(origin_lat, origin_lon, b, r)

        uncertainties = [ekf.position_uncertainty_m()]
        dt = 0.1
        orbit_radius_deg = 0.001
        for i in range(60):
            angle = i * 0.1
            uav_lat = origin_lat + orbit_radius_deg * math.sin(angle)
            uav_lon = origin_lon + orbit_radius_deg * math.cos(angle)
            b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)
            uncertainties.append(ekf.position_uncertainty_m())

        # 后半段不确定性应比前半段小
        assert sum(uncertainties[-20:]) < sum(uncertainties[:20])


# ── 匀速运动跟踪 ────────────────────────────────────────────────────────

class TestEKFMovingTarget:
    """匀速运动目标跟踪测试。UAV 也在运动（模拟真实跟踪场景）。"""

    def test_tracks_eastbound_target(self):
        origin_lat, origin_lon = 27.0, 125.0
        # 目标初始位置：正北 500m，以 8 m/s 向东运动
        target_lat = 27.0045  # ~500m 北
        target_lon0 = 125.0
        target_speed_east = 8.0  # m/s

        # UAV 以 30 m/s 向北运动（模拟跟踪飞行）
        uav_speed_north = 30.0  # m/s

        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        b = bearing_rad(origin_lat, origin_lon, target_lat, target_lon0)
        ekf.initialize(origin_lat, origin_lon, b, 500.0)

        dt = 0.1
        for i in range(100):  # 10 秒
            t = (i + 1) * dt
            # UAV 向北移动
            uav_lat = origin_lat + (uav_speed_north * t) / 111_320.0
            uav_lon = origin_lon

            # 目标向东移动
            cur_lon = target_lon0 + (target_speed_east * t) / (
                111_320.0 * math.cos(math.radians(target_lat))
            )

            b = bearing_rad(uav_lat, uav_lon, target_lat, cur_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)

        est_lat, est_lon = ekf.position_wgs84()
        true_lon = target_lon0 + 80.0 / (111_320.0 * math.cos(math.radians(target_lat)))
        error_m = haversine_m(est_lat, est_lon, target_lat, true_lon)
        assert error_m < 80.0, f"运动目标跟踪误差 {error_m:.1f}m > 80m"

        # 速度估计应指向东（v_east > 0）
        ve, vn = ekf.velocity_mps()
        assert ve > 0.5, f"v_east={ve:.1f}，应 > 0.5（向东）"

    def test_speed_estimate_reasonable(self):
        origin_lat, origin_lon = 27.0, 125.0
        target_lat = 27.0045
        target_lon0 = 125.0
        true_speed = 8.0

        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        b = bearing_rad(origin_lat, origin_lon, target_lat, target_lon0)
        ekf.initialize(origin_lat, origin_lon, b, 500.0)

        dt = 0.1
        uav_speed_north = 30.0
        for i in range(200):  # 20 秒
            t = (i + 1) * dt
            uav_lat = origin_lat + (uav_speed_north * t) / 111_320.0
            cur_lon = target_lon0 + (true_speed * t) / (
                111_320.0 * math.cos(math.radians(target_lat))
            )
            b = bearing_rad(uav_lat, origin_lon, target_lat, cur_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, origin_lon, b)

        speed = ekf.speed_mps()
        assert 0.3 * true_speed < speed < 3.0 * true_speed, \
            f"速度估计 {speed:.1f} m/s 偏离真实 {true_speed} m/s 过多"


# ── 缺失观测 ────────────────────────────────────────────────────────────

class TestEKFMissingObservation:
    """缺失观测时 predict-only 不崩溃，协方差增长。"""

    def test_survives_predict_only(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        for _ in range(20):
            ekf.predict(0.1)
        assert ekf.is_initialized()

    def test_covariance_grows_without_update(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        u0 = ekf.position_uncertainty_m()
        for _ in range(20):
            ekf.predict(0.1)
        u1 = ekf.position_uncertainty_m()
        assert u1 > u0, "缺失 update 时不确定性应增长"

    def test_recovery_after_gap(self):
        """缺失 10 帧后恢复观测，应能继续跟踪。"""
        origin_lat, origin_lon = 27.0, 125.0
        target_lat, target_lon = 27.003, 125.0
        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        b, r = _simulate_detection(origin_lat, origin_lon, target_lat, target_lon)
        ekf.initialize(origin_lat, origin_lon, b, r)

        orbit_radius_deg = 0.001
        dt = 0.1

        # 前 20 帧正常观测
        for i in range(20):
            angle = i * 0.1
            uav_lat = origin_lat + orbit_radius_deg * math.sin(angle)
            uav_lon = origin_lon + orbit_radius_deg * math.cos(angle)
            b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)

        # 中间 10 帧缺失
        for _ in range(10):
            ekf.predict(dt)

        # 后 20 帧恢复
        for i in range(20):
            angle = (30 + i) * 0.1
            uav_lat = origin_lat + orbit_radius_deg * math.sin(angle)
            uav_lon = origin_lon + orbit_radius_deg * math.cos(angle)
            b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)

        est_lat, est_lon = ekf.position_wgs84()
        error_m = haversine_m(est_lat, est_lon, target_lat, target_lon)
        assert error_m < 100.0, f"恢复后误差 {error_m:.1f}m > 100m"


# ── is_converged ────────────────────────────────────────────────────────

class TestEKFConverged:
    def test_not_converged_initially(self):
        ekf = BearingOnlyEKF(27.0, 125.0)
        ekf.initialize(27.0, 125.0, 0.0, 500.0)
        assert not ekf.is_converged(10.0)

    def test_converged_after_many_updates(self):
        origin_lat, origin_lon = 27.0, 125.0
        target_lat, target_lon = 27.003, 125.003  # ~330m 东北
        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        b, r = _simulate_detection(origin_lat, origin_lon, target_lat, target_lon)
        ekf.initialize(origin_lat, origin_lon, b, r)

        # UAV 做大幅度盘旋（半径 ~200m），充分激发可观性
        orbit_radius_deg = 0.002  # ~222m
        dt = 0.1
        for i in range(150):
            angle = i * 0.06
            uav_lat = origin_lat + orbit_radius_deg * math.sin(angle)
            uav_lon = origin_lon + orbit_radius_deg * math.cos(angle)
            b = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
            ekf.predict(dt)
            ekf.update_bearing(uav_lat, uav_lon, b)

        assert ekf.is_converged(30.0), \
            f"150 帧后应收敛到 30m 以内，当前不确定性 {ekf.position_uncertainty_m():.1f}m"

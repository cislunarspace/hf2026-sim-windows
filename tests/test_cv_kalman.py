"""tests/test_cv_kalman.py — CV 卡尔曼滤波（位置量测）数值测试。"""

import math
import random

from algorithms.estimation.cv_kalman import CvFilter, _wgs84_to_local

OLAT, OLON = 27.0, 125.0


def _run_track(flt: CvFilter, n_frames: int, dt: float, speed_e: float,
               noise_m: float, seed: int = 42, start_e: float = 1000.0,
               start_n: float = -500.0, init_with_velocity: bool = False):
    """目标沿东向匀速运动，喂带噪位置量测，返回末端 (位置误差, 速度误差)。"""
    rng = random.Random(seed)
    e, n = start_e, start_n
    if init_with_velocity:
        flt.initialize(*_to_wgs(e, n), ve=speed_e, vn=0.0)
    else:
        flt.initialize(*_to_wgs(e, n))
    for _ in range(n_frames):
        e += speed_e * dt
        flt.predict(dt)
        flt.update_position(*_to_wgs(e + rng.gauss(0, noise_m),
                                     n + rng.gauss(0, noise_m)))
    est_e, est_n = _wgs84_to_local(*flt.position_wgs84(), OLAT, OLON)
    ve, vn = flt.velocity_mps()
    return math.hypot(est_e - e, est_n - n), math.hypot(ve - speed_e, vn)


def _to_wgs(e: float, n: float):
    from algorithms.estimation.cv_kalman import _local_to_wgs84
    return _local_to_wgs84(e, n, OLAT, OLON)


class TestConvergence:
    def test_zero_noise_tracks_exactly(self):
        """无噪声 + 先验速度初值（比赛用法）时滤波应立即贴合真实轨迹。"""
        flt = CvFilter(OLAT, OLON)
        pos_err, vel_err = _run_track(flt, 10, 0.1, 8.0, noise_m=0.0,
                                      init_with_velocity=True)
        assert pos_err < 1.0, f"位置误差 {pos_err:.2f}m"
        assert vel_err < 0.5, f"速度误差 {vel_err:.2f}m/s"

    def test_noisy_converges_under_30m(self):
        """±50m 量测噪声（比赛设定）下 3s 内位置误差应 <30m（得分线）。"""
        flt = CvFilter(OLAT, OLON)
        pos_err, _ = _run_track(flt, 30, 0.1, 8.0, noise_m=50.0)
        assert pos_err < 30.0, f"位置误差 {pos_err:.2f}m"

    def test_noisy_velocity_converges_by_10s(self):
        """速度估计靠量测从 0 拉起（无过冲设计），10s 内应收敛到 <3 m/s。"""
        flt = CvFilter(OLAT, OLON)
        _, vel_err = _run_track(flt, 100, 0.1, 8.0, noise_m=50.0)
        assert vel_err < 3.0, f"速度误差 {vel_err:.2f}m/s"

    def test_std_shrinks(self):
        """协方差应随量测单调收敛（is_converged 的依据）。"""
        flt = CvFilter(OLAT, OLON)
        flt.initialize(27.001, 125.001)
        std0 = flt.position_std_m()
        for _ in range(20):
            flt.predict(0.1)
            flt.update_position(27.001, 125.001)
        assert flt.position_std_m() < std0
        assert flt.is_converged(15.0), "20 帧后位置 std 应 <15m"


class TestCoast:
    def test_coast_extrapolates(self):
        """丢失量测后 predict 应按速度外推（LOST 重捕获/上报依据）。"""
        flt = CvFilter(OLAT, OLON)
        _run_track(flt, 10, 0.1, 8.0, noise_m=0.0, init_with_velocity=True)
        e0, n0 = _wgs84_to_local(*flt.position_wgs84(), OLAT, OLON)
        for _ in range(10):  # 1s 纯外推
            flt.predict(0.1)
        e1, _ = _wgs84_to_local(*flt.position_wgs84(), OLAT, OLON)
        assert math.isclose(e1 - e0, 8.0, abs_tol=0.5), "1s 外推应约等于速度×时间"


class TestInterface:
    def test_uninitialized(self):
        flt = CvFilter(OLAT, OLON)
        assert not flt.is_initialized()
        assert not flt.is_converged(100.0)

    def test_roundtrip_coordinates(self):
        """WGS84→local→WGS84 应闭合（原点选取不影响结果）。"""
        flt = CvFilter(OLAT, OLON)
        flt.initialize(27.005, 124.995)
        lat, lon = flt.position_wgs84()
        assert abs(lat - 27.005) < 1e-6
        assert abs(lon - 124.995) < 1e-6

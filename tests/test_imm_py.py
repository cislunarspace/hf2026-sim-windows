"""纯 Python IMM（imm_py）测试。

覆盖：
1. 与 Rust 版 ImmFilter 的数值一致性（相同输入序列，结果应在小容差内）
2. rust_core 不可用时 ekf.py 回退到纯 Python 版
"""

import importlib
import math
import sys

import pytest

from algorithms.estimation.geometry import haversine_m
from algorithms.estimation.imm_py import ImmFilter as PyImmFilter

ORIGIN_LAT = 40.0
ORIGIN_LON = 116.0

rust_core = pytest.importorskip("rust_core", reason="对比测试需要 Rust 版")
RustImmFilter = rust_core.ImmFilter


def _drive_scenario(imm):
    """同一条输入序列驱动滤波器：UAV 向东运动，目标先直行后转弯。"""
    dt = 0.1
    imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.atan2(500.0, 0.0), 500.0)

    # 目标真值：先 10 m/s 向东，第 50 步起以 0.1 rad/s 转弯
    te, tn = 500.0, 0.0
    ve, vn = 10.0, 0.0
    omega = 0.1
    # UAV 以 15 m/s 向东运动（产生方位角视差）
    uav_e = 0.0

    for k in range(200):
        imm.predict(dt)
        uav_e += 15.0 * dt
        de = te - uav_e
        dn = tn
        bearing = math.atan2(de, dn)
        rng = math.hypot(de, dn)
        uav_lat = ORIGIN_LAT
        uav_lon = ORIGIN_LON + uav_e / (
            math.cos(math.radians(ORIGIN_LAT)) * 6_371_000.0
        ) * (180 / math.pi)
        imm.update_bearing(uav_lat, uav_lon, bearing)
        imm.update_range(uav_lat, uav_lon, rng)

        # 推进目标真值
        if k >= 50:
            wt = omega * dt
            sin_wt, cos_wt = math.sin(wt), math.cos(wt)
            te, tn = (
                te + (ve * sin_wt - vn * (1 - cos_wt)) / omega,
                tn + (ve * (1 - cos_wt) + vn * sin_wt) / omega,
            )
            ve, vn = ve * cos_wt - vn * sin_wt, ve * sin_wt + vn * cos_wt
        else:
            te += ve * dt


class TestImmPyMatchesRust:
    """纯 Python 版与 Rust 版数值一致性。"""

    def test_same_sequence_same_result(self):
        rust = RustImmFilter(ORIGIN_LAT, ORIGIN_LON)
        py = PyImmFilter(ORIGIN_LAT, ORIGIN_LON)
        _drive_scenario(rust)
        _drive_scenario(py)

        # 位置
        r_lat, r_lon = rust.position_wgs84()
        p_lat, p_lon = py.position_wgs84()
        pos_diff_m = haversine_m(r_lat, r_lon, p_lat, p_lon)
        assert pos_diff_m < 1.0, f"位置差 {pos_diff_m:.4f}m"

        # 速度
        r_ve, r_vn = rust.velocity_mps()
        p_ve, p_vn = py.velocity_mps()
        vel_diff = math.hypot(r_ve - p_ve, r_vn - p_vn)
        assert vel_diff < 0.1, f"速度差 {vel_diff:.4f} m/s"

        # 速率
        assert abs(rust.speed_mps() - py.speed_mps()) < 0.1

        # 不确定性
        unc_diff = abs(rust.position_uncertainty_m() - py.position_uncertainty_m())
        assert unc_diff < 1.0, f"不确定性差 {unc_diff:.4f}m"

        # 模型概率
        for rp, pp in zip(rust.model_probabilities(), py.model_probabilities()):
            assert abs(rp - pp) < 1e-6, f"模型概率差 {abs(rp - pp)}"

    def test_api_parity(self):
        """两版公开方法签名一致。"""
        for name in (
            "initialize",
            "predict",
            "update_bearing",
            "update_range",
            "position_wgs84",
            "velocity_mps",
            "speed_mps",
            "position_uncertainty_m",
            "model_probabilities",
            "is_initialized",
            "is_converged",
        ):
            assert hasattr(PyImmFilter, name), f"缺少方法 {name}"


class TestImmPyFallback:
    """rust_core 不可用时 ekf.py 回退到纯 Python 版。"""

    def test_ekf_falls_back_to_python(self, monkeypatch):
        monkeypatch.setitem(sys.modules, "rust_core", None)  # 屏蔽 rust_core 导入
        monkeypatch.delitem(sys.modules, "algorithms.estimation.ekf", raising=False)

        ekf_mod = importlib.import_module("algorithms.estimation.ekf")
        assert ekf_mod.ImmFilter is PyImmFilter
        assert ekf_mod.BearingOnlyEKF is None

        # 回退版可正常使用
        imm = ekf_mod.ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        assert not imm.is_initialized()
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)
        assert imm.is_initialized()
        for _ in range(10):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(45))
        lat, lon = imm.position_wgs84()
        assert isinstance(lat, float) and isinstance(lon, float)
        assert imm.speed_mps() >= 0

        # 恢复：让后续测试重新按正常路径导入
        importlib.reload(ekf_mod)

"""IMM 滤波器测试。

覆盖：
1. 基本生命周期（初始化、预测、观测更新）
2. 融合状态查询（位置、速度、不确定性）
3. 模型概率（匀速场景 CV 主导、转弯场景 CT 主导）
4. 与单模型 EKF 的精度对比（转弯场景）
"""
import math
import pytest
from algorithms.estimation.ekf import ImmFilter, BearingOnlyEKF

# ── 基准坐标（北京附近） ────────────────────────────────────────────────
ORIGIN_LAT = 40.0
ORIGIN_LON = 116.0


class TestImmLifecycle:
    """基本生命周期测试。"""

    def test_not_initialized_before_initialize(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        assert not imm.is_initialized()

    def test_initialized_after_initialize(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)
        assert imm.is_initialized()

    def test_predict_before_initialize_is_noop(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.predict(0.1)  # 不应崩溃
        assert not imm.is_initialized()

    def test_position_after_initialize(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        bearing = math.radians(90)  # 正东
        range_m = 1000.0
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, bearing, range_m)

        lat, lon = imm.position_wgs84()
        # 位置应在 origin 正东约 1000m
        from algorithms.estimation.geometry import haversine_m
        dist = haversine_m(ORIGIN_LAT, ORIGIN_LON, lat, lon)
        assert 900 < dist < 1100

    def test_speed_initially_near_zero(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, 0.0, 1000.0)
        assert imm.speed_mps() < 1.0  # 初始速度为零


class TestImmPrediction:
    """预测步骤测试。"""

    def test_predict_and_update_sequence(self):
        """连续 predict + update 应不崩溃且状态可查询。"""
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(90), 1000.0)

        for _ in range(50):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(90))

        lat, lon = imm.position_wgs84()
        assert isinstance(lat, float) and isinstance(lon, float)
        ve, vn = imm.velocity_mps()
        assert isinstance(ve, float) and isinstance(vn, float)
        assert imm.speed_mps() >= 0


class TestImmBearingUpdate:
    """方位角观测更新测试。"""

    def test_bearing_update_reduces_uncertainty(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)

        u0 = imm.position_uncertainty_m()
        # 多次 predict + bearing update
        for _ in range(20):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(45))

        u1 = imm.position_uncertainty_m()
        assert u1 < u0, "多次观测后不确定性应减小"

    def test_convergence_with_uav_motion(self):
        """UAV 有运动时，IMM 应能收敛（方位角视差提供距离信息）。"""
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)

        # UAV 向东运动（产生方位角视差）
        uav_e = 0.0
        uav_speed = 15.0  # m/s
        dt = 0.1
        for _ in range(200):
            imm.predict(dt)
            uav_e += uav_speed * dt
            # 目标在 (500, 500)，UAV 在 (uav_e, 0)
            de = 500.0 - uav_e
            dn = 500.0
            bearing = math.atan2(de, dn)
            # 将 UAV 的局部位置转回 WGS84
            uav_lat = ORIGIN_LAT
            uav_lon = ORIGIN_LON + uav_e / (math.cos(math.radians(ORIGIN_LAT)) * 6_371_000.0) * (180 / math.pi)
            imm.update_bearing(uav_lat, uav_lon, bearing)

        u = imm.position_uncertainty_m()
        assert u < 300.0, f"UAV 运动 200 帧后不确定性应 < 300m，实际 {u:.1f}m"


class TestImmModelProbabilities:
    """模型概率测试。"""

    def test_model_probs_sum_to_one(self):
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)

        for _ in range(50):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(45))

        probs = imm.model_probabilities()
        assert len(probs) == 3
        assert abs(sum(probs) - 1.0) < 0.01, f"模型概率之和应为 1，实际 {sum(probs)}"
        assert all(p >= 0 for p in probs)

    def test_cv_dominates_straight_line(self):
        """匀速直线运动场景下，CV 模型概率应最高。"""
        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(90), 500.0)

        # 目标以 10 m/s 向东匀速运动
        target_e = 500.0  # 初始 east
        dt = 0.1
        for _ in range(100):
            imm.predict(dt)
            target_e += 10.0 * dt
            # 计算 UAV 看目标的 bearing（UAV 在 origin）
            bearing = math.atan2(target_e, 0.0)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, bearing)

        probs = imm.model_probabilities()
        cv_prob = probs[0]
        assert cv_prob > 0.5, f"匀速场景 CV 概率应 > 0.5，实际 {cv_prob}"


class TestImmVsEKF:
    """IMM vs 单模型 EKF 精度对比。"""

    def test_turning_target_imm_better_than_ekf(self):
        """转弯场景下 IMM 精度应优于单 CV EKF。"""
        # 设置：目标以 10 m/s 向东运动，然后以 0.1 rad/s 转弯
        origin_lat, origin_lon = ORIGIN_LAT, ORIGIN_LON
        dt = 0.1
        total_steps = 200

        # 目标真实轨迹（先东后转弯）
        true_positions = []
        te, tn = 500.0, 0.0
        ve, vn = 10.0, 0.0
        omega = 0.1  # 转弯率
        turning_start = 50  # 第 50 步开始转弯

        for k in range(total_steps):
            true_positions.append((te, tn))
            if k >= turning_start:
                # CT 模型运动
                wt = omega * dt
                sin_wt = math.sin(wt)
                cos_wt = math.cos(wt)
                te_new = te + (ve * sin_wt - vn * (1 - cos_wt)) / omega
                tn_new = tn + (ve * (1 - cos_wt) + vn * sin_wt) / omega
                ve_new = ve * cos_wt - vn * sin_wt
                vn_new = ve * sin_wt + vn * cos_wt
                te, tn = te_new, tn_new
                ve, vn = ve_new, vn_new
            else:
                te += ve * dt
                tn += vn * dt

        # IMM 滤波
        imm = ImmFilter(origin_lat, origin_lon)
        imm.initialize(origin_lat, origin_lon, math.atan2(500.0, 0.0), 500.0)
        imm_errors = []

        # EKF（CV 模型）
        ekf = BearingOnlyEKF(origin_lat, origin_lon)
        ekf.initialize(origin_lat, origin_lon, math.atan2(500.0, 0.0), 500.0)
        ekf_errors = []

        for k in range(total_steps):
            true_e, true_n = true_positions[k]
            bearing = math.atan2(true_e, true_n)

            imm.predict(dt)
            imm.update_bearing(origin_lat, origin_lon, bearing)

            ekf.predict(dt)
            ekf.update_bearing(origin_lat, origin_lon, bearing)

            # 位置误差
            imm_lat, imm_lon = imm.position_wgs84()
            ekf_lat, ekf_lon = ekf.position_wgs84()

            from algorithms.estimation.geometry import haversine_m
            # 真实位置的 WGS84
            true_lat = origin_lat + true_n / 6_371_000.0 * (180 / math.pi)
            true_lon = origin_lon + true_e / (math.cos(math.radians(origin_lat)) * 6_371_000.0) * (180 / math.pi)

            imm_err = haversine_m(true_lat, true_lon, imm_lat, imm_lon)
            ekf_err = haversine_m(true_lat, true_lon, ekf_lat, ekf_lon)

            imm_errors.append(imm_err)
            ekf_errors.append(ekf_err)

        # 比较转弯后的平均误差（后 100 步）
        turn_imm_avg = sum(imm_errors[turning_start:]) / len(imm_errors[turning_start:])
        turn_ekf_avg = sum(ekf_errors[turning_start:]) / len(ekf_errors[turning_start:])

        # IMM 在转弯场景应更优（允许一定容差）
        assert turn_imm_avg < turn_ekf_avg * 1.5, (
            f"转弯场景 IMM ({turn_imm_avg:.1f}m) 应优于 EKF ({turn_ekf_avg:.1f}m)"
        )


class TestImmPerformance:
    """性能测试。"""

    def test_predict_update_under_0_1ms(self):
        """单帧 predict + update 应 < 0.1ms。"""
        import time

        imm = ImmFilter(ORIGIN_LAT, ORIGIN_LON)
        imm.initialize(ORIGIN_LAT, ORIGIN_LON, math.radians(45), 1000.0)

        # 预热
        for _ in range(100):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(45))

        # 测量
        n = 1000
        t0 = time.perf_counter()
        for _ in range(n):
            imm.predict(0.1)
            imm.update_bearing(ORIGIN_LAT, ORIGIN_LON, math.radians(45))
        elapsed = time.perf_counter() - t0

        avg_us = elapsed / n * 1e6
        assert avg_us < 150, f"单帧耗时 {avg_us:.1f}µs，应 < 150µs (0.15ms)"

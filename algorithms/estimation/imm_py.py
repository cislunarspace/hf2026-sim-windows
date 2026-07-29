"""IMM 滤波器纯 Python 实现（rust_core 不可用时的后备）。

移植自 rust_core/src/imm.rs，API 与 Rust 版 ImmFilter 完全一致。
三个运动模型交互多模型（IMM）：
- CV (Constant Velocity): 4D [east, north, v_east, v_north]
- CA (Constant Acceleration): 6D [..., a_east, a_north]
- CT (Coordinated Turn): 5D [..., omega]

观测模型：方位角 bearing（可选距离），只依赖位置分量。
内部状态为局部 ENU 坐标（米），原点在构造时给定；
bearing 定义为 atan2(de, dn)，即相对正北顺时针的方位角。

环境无 numpy，矩阵用 list[list[float]] 手写小尺寸运算（最大 6×6）。
"""

import math

EARTH_RADIUS_M = 6_371_000.0

# ── 坐标转换 ────────────────────────────────────────────────────────────


def _wgs84_to_local(lat, lon, olat, olon):
    east = math.radians(lon - olon) * math.cos(math.radians(olat)) * EARTH_RADIUS_M
    north = math.radians(lat - olat) * EARTH_RADIUS_M
    return east, north


def _local_to_wgs84(east, north, olat, olon):
    rlat_o = math.radians(olat)
    rlon_o = math.radians(olon)
    dlat = north / EARTH_RADIUS_M
    dlon = east / (math.cos(rlat_o) * EARTH_RADIUS_M)
    return math.degrees(rlat_o + dlat), math.degrees(rlon_o + dlon)


# ── 小矩阵工具（list[list[float]]，行优先） ─────────────────────────────


def _zeros(n):
    return [[0.0] * n for _ in range(n)]


def _identity(n):
    m = _zeros(n)
    for i in range(n):
        m[i][i] = 1.0
    return m


def _matmul(a, b):
    n = len(a)
    k_dim = len(b)
    m_dim = len(b[0])
    out = [[0.0] * m_dim for _ in range(n)]
    for i in range(n):
        ai = a[i]
        oi = out[i]
        for k in range(k_dim):
            aik = ai[k]
            if aik == 0.0:
                continue
            bk = b[k]
            for j in range(m_dim):
                oi[j] += aik * bk[j]
    return out


def _transpose(a):
    return [list(row) for row in zip(*a)]


def _matvec(a, v):
    return [sum(ai * vi for ai, vi in zip(row, v)) for row in a]


def _dot(u, v):
    return sum(ui * vi for ui, vi in zip(u, v))


def _wrap_angle(a):
    """把角度折回 (-π, π]。"""
    if a > math.pi:
        a -= 2.0 * math.pi
    elif a < -math.pi:
        a += 2.0 * math.pi
    return a


# ── 观测模型公共函数 ────────────────────────────────────────────────────

_BEARING_NOISE_VAR = 0.02**2
_RANGE_NOISE_VAR = 50.0**2


def _bearing_innovation_and_jacobian(x, p, uav_e, uav_n, measured_bearing):
    """方位角观测：h(x) = atan2(east - uav_e, north - uav_n)。

    返回 (innovation, S, H_row)。
    """
    n = len(x)
    de = x[0] - uav_e
    dn = x[1] - uav_n
    r2 = de * de + dn * dn

    h = [0.0] * n
    if r2 < 1.0:
        return 0.0, 1e30, h

    innov = _wrap_angle(measured_bearing - math.atan2(de, dn))

    # 雅可比（只在位置分量非零）
    h[0] = dn / r2
    h[1] = -de / r2

    # S = H * P * H^T + R
    s = _dot(h, _matvec(p, h)) + _BEARING_NOISE_VAR
    return innov, s, h


def _range_innovation_and_jacobian(x, p, uav_e, uav_n, measured_range):
    """距离观测：h(x) = sqrt((x[0]-uav_e)² + (x[1]-uav_n)²)。"""
    n = len(x)
    de = x[0] - uav_e
    dn = x[1] - uav_n
    r = math.sqrt(de * de + dn * dn)

    h = [0.0] * n
    if r < 1e-3:
        return 0.0, 1e30, h

    innov = measured_range - r

    h[0] = de / r
    h[1] = dn / r

    s = _dot(h, _matvec(p, h)) + _RANGE_NOISE_VAR
    return innov, s, h


def _joseph_update(x, p, h, innov, r_noise_var, s):
    """Joseph 形式协方差更新：P = (I-KH)P(I-KH)^T + KRK^T。"""
    n = len(x)
    # K = P * H^T / S
    ph = _matvec(p, h)
    k = [v / s for v in ph]

    # 状态更新
    for i in range(n):
        x[i] += k[i] * innov

    # I - K*H
    i_kh = _identity(n)
    for i in range(n):
        for j in range(n):
            i_kh[i][j] -= k[i] * h[j]

    new_p = _matmul(_matmul(i_kh, p), _transpose(i_kh))
    for i in range(n):
        for j in range(n):
            new_p[i][j] += k[i] * r_noise_var * k[j]

    # 对称化 + 对角钳制
    for i in range(n):
        if new_p[i][i] < 1e-6:
            new_p[i][i] = 1e-6
        for j in range(i + 1, n):
            avg = (new_p[i][j] + new_p[j][i]) * 0.5
            new_p[i][j] = avg
            new_p[j][i] = avg

    # 原地写回
    for i in range(n):
        p[i][:] = new_p[i]


def _gaussian_likelihood(innov, s):
    """高斯似然：N(innov; 0, S) = (2πS)^{-1/2} * exp(-innov²/(2S))。"""
    if s < 1e-30:
        return 1e-30
    return math.exp(-0.5 * innov * innov / s) / math.sqrt(2.0 * math.pi * s)


# ── CV 模型 (4D) ────────────────────────────────────────────────────────


class _CvModel:
    def __init__(self):
        self.x = [0.0] * 4
        self.p = _identity(4)
        self.initialized = False

    def initialize(self, east, north, ve, vn, pos_var, vel_var):
        self.x = [east, north, ve, vn]
        self.p = _identity(4)
        self.p[0][0] = pos_var
        self.p[1][1] = pos_var
        self.p[2][2] = vel_var
        self.p[3][3] = vel_var
        self.initialized = True

    def common_state(self):
        return list(self.x)

    def common_cov(self):
        return [row[:] for row in self.p]

    def predict(self, dt):
        if not self.initialized:
            return
        # 状态转移
        self.x[0] += self.x[2] * dt
        self.x[1] += self.x[3] * dt

        f = _identity(4)
        f[0][2] = dt
        f[1][3] = dt

        # 过程噪声 (DWNA, accel_var=1 m/s²)
        accel_var = 1.0
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        q = _zeros(4)
        q[0][0] = dt4 / 4.0 * accel_var
        q[1][1] = dt4 / 4.0 * accel_var
        q[2][2] = dt2 * accel_var
        q[3][3] = dt2 * accel_var
        q[0][2] = dt3 / 2.0 * accel_var
        q[2][0] = dt3 / 2.0 * accel_var
        q[1][3] = dt3 / 2.0 * accel_var
        q[3][1] = dt3 / 2.0 * accel_var

        fp = _matmul(_matmul(f, self.p), _transpose(f))
        for i in range(4):
            for j in range(4):
                fp[i][j] += q[i][j]
        self.p = fp

    def update_bearing(self, uav_e, uav_n, measured_bearing):
        """方位角观测更新。返回 (innovation, S) 供 IMM 似然计算。"""
        innov, s, h = _bearing_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_bearing
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _BEARING_NOISE_VAR, s)
        return innov, s

    def update_range(self, uav_e, uav_n, measured_range):
        innov, s, h = _range_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_range
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _RANGE_NOISE_VAR, s)


# ── CA 模型 (6D) ────────────────────────────────────────────────────────


class _CaModel:
    def __init__(self):
        self.x = [0.0] * 6
        self.p = _identity(6)
        self.initialized = False

    def initialize(self, east, north, ve, vn, ae, an, pos_var, vel_var, accel_var):
        self.x = [east, north, ve, vn, ae, an]
        self.p = _identity(6)
        self.p[0][0] = pos_var
        self.p[1][1] = pos_var
        self.p[2][2] = vel_var
        self.p[3][3] = vel_var
        self.p[4][4] = accel_var
        self.p[5][5] = accel_var
        self.initialized = True

    def common_state(self):
        return list(self.x[:4])

    def common_cov(self):
        return [row[:4] for row in self.p[:4]]

    def predict(self, dt):
        if not self.initialized:
            return
        # 状态转移：pos += vel*dt + 0.5*accel*dt², vel += accel*dt
        self.x[0] += self.x[2] * dt + 0.5 * self.x[4] * dt * dt
        self.x[1] += self.x[3] * dt + 0.5 * self.x[5] * dt * dt
        self.x[2] += self.x[4] * dt
        self.x[3] += self.x[5] * dt
        # 加速度保持不变 (CA 模型假设)

        f = _identity(6)
        dt2_2 = 0.5 * dt * dt
        f[0][2] = dt
        f[1][3] = dt
        f[0][4] = dt2_2
        f[1][5] = dt2_2
        f[2][4] = dt
        f[3][5] = dt

        # 过程噪声 (jerk 噪声模型, jerk_var=0.5 m/s³)
        jerk_var = 0.5**2
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt
        dt5 = dt4 * dt
        dt6 = dt5 * dt

        q_pp = dt6 / 36.0 * jerk_var
        q_pv = dt5 / 12.0 * jerk_var
        q_pa = dt4 / 6.0 * jerk_var
        q_vv = dt4 / 4.0 * jerk_var
        q_va = dt3 / 2.0 * jerk_var
        q_aa = dt2 * jerk_var

        q = _zeros(6)
        # 位置-位置
        q[0][0] = q_pp
        q[1][1] = q_pp
        # 速度-速度
        q[2][2] = q_vv
        q[3][3] = q_vv
        # 加速度-加速度
        q[4][4] = q_aa
        q[5][5] = q_aa
        # 位置-速度
        q[0][2] = q_pv
        q[2][0] = q_pv
        q[1][3] = q_pv
        q[3][1] = q_pv
        # 位置-加速度
        q[0][4] = q_pa
        q[4][0] = q_pa
        q[1][5] = q_pa
        q[5][1] = q_pa
        # 速度-加速度
        q[2][4] = q_va
        q[4][2] = q_va
        q[3][5] = q_va
        q[5][3] = q_va

        fp = _matmul(_matmul(f, self.p), _transpose(f))
        for i in range(6):
            for j in range(6):
                fp[i][j] += q[i][j]
        self.p = fp

    def update_bearing(self, uav_e, uav_n, measured_bearing):
        innov, s, h = _bearing_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_bearing
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _BEARING_NOISE_VAR, s)
        return innov, s

    def update_range(self, uav_e, uav_n, measured_range):
        innov, s, h = _range_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_range
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _RANGE_NOISE_VAR, s)


# ── CT 模型 (5D) ────────────────────────────────────────────────────────


class _CtModel:
    def __init__(self):
        self.x = [0.0] * 5
        self.p = _identity(5)
        self.initialized = False

    def initialize(self, east, north, ve, vn, omega, pos_var, vel_var, omega_var):
        self.x = [east, north, ve, vn, omega]
        self.p = _identity(5)
        self.p[0][0] = pos_var
        self.p[1][1] = pos_var
        self.p[2][2] = vel_var
        self.p[3][3] = vel_var
        self.p[4][4] = omega_var
        self.initialized = True

    def common_state(self):
        return list(self.x[:4])

    def common_cov(self):
        return [row[:4] for row in self.p[:4]]

    def predict(self, dt):
        if not self.initialized:
            return

        omega = self.x[4]
        wt = omega * dt

        # 状态转移：精确 CT 模型
        # sinc_wt = sin(wt)/omega, versine_wt = (1-cos(wt))/omega
        # 小角度用 Taylor 展开避免除以 omega 的符号问题
        if abs(wt) < 0.1:
            dt2 = dt * dt
            wt2 = wt * wt
            sinc_wt = dt * (1.0 - wt2 / 6.0 + wt2 * wt2 / 120.0)
            versine_wt = omega * dt2 / 2.0 * (1.0 - wt2 / 12.0)
        else:
            sinc_wt = math.sin(wt) / omega
            versine_wt = (1.0 - math.cos(wt)) / omega

        # pos += ve*sinc - vn*versine, pos += ve*versine + vn*sinc
        east_new = self.x[0] + self.x[2] * sinc_wt - self.x[3] * versine_wt
        north_new = self.x[1] + self.x[2] * versine_wt + self.x[3] * sinc_wt
        cos_wt = math.cos(wt)
        sin_wt = math.sin(wt)
        ve_new = self.x[2] * cos_wt - self.x[3] * sin_wt
        vn_new = self.x[2] * sin_wt + self.x[3] * cos_wt
        self.x[0] = east_new
        self.x[1] = north_new
        self.x[2] = ve_new
        self.x[3] = vn_new
        # omega 保持不变

        # 雅可比 F（EKF 线性化）
        f = _identity(5)
        if abs(wt) < 1e-6:
            # omega → 0 退化为 CV
            f[0][2] = dt
            f[1][3] = dt
        else:
            sin_wt = math.sin(wt)
            cos_wt = math.cos(wt)
            inv_w = 1.0 / omega
            inv_w2 = inv_w * inv_w

            # ∂pos/∂vel
            f[0][2] = inv_w * sin_wt
            f[0][3] = -inv_w * (1.0 - cos_wt)
            f[1][2] = inv_w * (1.0 - cos_wt)
            f[1][3] = inv_w * sin_wt

            # ∂vel/∂vel
            f[2][2] = cos_wt
            f[2][3] = -sin_wt
            f[3][2] = sin_wt
            f[3][3] = cos_wt

            # ∂pos/∂omega（注意用的是更新后的速度分量，与 Rust 版一致）
            f[0][4] = inv_w2 * (
                self.x[2] * (wt * cos_wt - sin_wt)
                + self.x[3] * (wt * sin_wt - 1.0 + cos_wt)
            )
            f[1][4] = inv_w2 * (
                -self.x[2] * (wt * sin_wt - 1.0 + cos_wt)
                + self.x[3] * (wt * cos_wt - sin_wt)
            )

            # ∂vel/∂omega
            f[2][4] = -dt * (self.x[2] * sin_wt + self.x[3] * cos_wt)
            f[3][4] = dt * (self.x[2] * cos_wt - self.x[3] * sin_wt)

        # 过程噪声
        accel_var = 1.0
        omega_noise_var = 0.1**2  # 转弯率噪声
        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt3 * dt

        q = _zeros(5)
        # 位置噪声
        q[0][0] = dt4 / 4.0 * accel_var
        q[1][1] = dt4 / 4.0 * accel_var
        # 速度噪声
        q[2][2] = dt2 * accel_var
        q[3][3] = dt2 * accel_var
        # 位置-速度交叉
        q[0][2] = dt3 / 2.0 * accel_var
        q[2][0] = dt3 / 2.0 * accel_var
        q[1][3] = dt3 / 2.0 * accel_var
        q[3][1] = dt3 / 2.0 * accel_var
        # 转弯率噪声
        q[4][4] = dt2 * omega_noise_var

        fp = _matmul(_matmul(f, self.p), _transpose(f))
        for i in range(5):
            for j in range(5):
                fp[i][j] += q[i][j]
        self.p = fp

    def update_bearing(self, uav_e, uav_n, measured_bearing):
        innov, s, h = _bearing_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_bearing
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _BEARING_NOISE_VAR, s)
        return innov, s

    def update_range(self, uav_e, uav_n, measured_range):
        innov, s, h = _range_innovation_and_jacobian(
            self.x, self.p, uav_e, uav_n, measured_range
        )
        if s > 1e-30 and not math.isnan(s):
            _joseph_update(self.x, self.p, h, innov, _RANGE_NOISE_VAR, s)


# ── IMM 滤波器 ──────────────────────────────────────────────────────────

# 模型索引：0=CV, 1=CA, 2=CT


class ImmFilter:
    """Interacting Multiple Model 滤波器（CV + CA + CT）。

    API 与 rust_core.ImmFilter 完全一致。
    """

    def __init__(self, origin_lat, origin_lon):
        self.cv = _CvModel()
        self.ca = _CaModel()
        self.ct = _CtModel()
        # 模型转移概率 [from][to]
        self.transition = [
            [0.90, 0.05, 0.05],
            [0.10, 0.85, 0.05],
            [0.10, 0.05, 0.85],
        ]
        # 模型概率 [CV, CA, CT]
        self.model_probs = [1.0 / 3.0] * 3
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.initialized = False

    def initialize(self, uav_lat, uav_lon, bearing_rad, assumed_range_m):
        """初始化：从首次检测建立初始估计。"""
        uav_e, uav_n = _wgs84_to_local(
            uav_lat, uav_lon, self.origin_lat, self.origin_lon
        )
        target_e = uav_e + assumed_range_m * math.sin(bearing_rad)
        target_n = uav_n + assumed_range_m * math.cos(bearing_rad)
        pos_var = (assumed_range_m * 0.5) ** 2
        vel_var = 100.0

        self.cv.initialize(target_e, target_n, 0.0, 0.0, pos_var, vel_var)
        self.ca.initialize(target_e, target_n, 0.0, 0.0, 0.0, 0.0, pos_var, vel_var, 1.0)
        self.ct.initialize(target_e, target_n, 0.0, 0.0, 0.0, pos_var, vel_var, 0.1)
        self.model_probs = [1.0 / 3.0] * 3
        self.initialized = True

    def predict(self, dt):
        """IMM 交互 + 预测。"""
        if not self.initialized:
            return

        # ── 步骤 1：交互混合 ──
        (cv_x, cv_p, ca_x, ca_p, ct_x, ct_p) = self._mix_states()

        self.cv.x = cv_x
        self.cv.p = cv_p
        self.ca.x = ca_x
        self.ca.p = ca_p
        self.ct.x = ct_x
        self.ct.p = ct_p

        # ── 步骤 2：各模型独立预测 ──
        self.cv.predict(dt)
        self.ca.predict(dt)
        self.ct.predict(dt)

    def update_bearing(self, uav_lat, uav_lon, measured_bearing_rad):
        """方位角观测更新。"""
        if not self.initialized:
            return
        uav_e, uav_n = _wgs84_to_local(
            uav_lat, uav_lon, self.origin_lat, self.origin_lon
        )

        # 步骤 1：计算各模型的先验创新（在状态更新之前）
        innov_cv = self._innovation_for_model(uav_e, uav_n, measured_bearing_rad, 0)
        innov_ca = self._innovation_for_model(uav_e, uav_n, measured_bearing_rad, 1)
        innov_ct = self._innovation_for_model(uav_e, uav_n, measured_bearing_rad, 2)

        # 步骤 2：各模型独立更新（修改状态 + 协方差）
        _, s_cv = self.cv.update_bearing(uav_e, uav_n, measured_bearing_rad)
        _, s_ca = self.ca.update_bearing(uav_e, uav_n, measured_bearing_rad)
        _, s_ct = self.ct.update_bearing(uav_e, uav_n, measured_bearing_rad)

        # 步骤 3：用先验创新和 S 更新模型概率
        self._update_model_probabilities(
            innov_cv, s_cv, innov_ca, s_ca, innov_ct, s_ct
        )

    def update_range(self, uav_lat, uav_lon, measured_range_m):
        """距离观测更新。"""
        if not self.initialized:
            return
        uav_e, uav_n = _wgs84_to_local(
            uav_lat, uav_lon, self.origin_lat, self.origin_lon
        )
        self.cv.update_range(uav_e, uav_n, measured_range_m)
        self.ca.update_range(uav_e, uav_n, measured_range_m)
        self.ct.update_range(uav_e, uav_n, measured_range_m)

    def position_wgs84(self):
        """返回目标估计位置（WGS84）。"""
        cs = self._fused_common_state()
        return _local_to_wgs84(cs[0], cs[1], self.origin_lat, self.origin_lon)

    def velocity_mps(self):
        """返回速度估计 (v_east, v_north) m/s。"""
        cs = self._fused_common_state()
        return cs[2], cs[3]

    def speed_mps(self):
        """返回速度大小 m/s。"""
        ve, vn = self.velocity_mps()
        return math.sqrt(ve * ve + vn * vn)

    def position_uncertainty_m(self):
        """返回位置不确定性。"""
        fc = self._fused_common_cov()
        return math.sqrt(fc[0][0] + fc[1][1])

    def model_probabilities(self):
        """返回模型概率 [CV, CA, CT]。"""
        return list(self.model_probs)

    def is_initialized(self):
        """是否已初始化。"""
        return self.initialized

    def is_converged(self, threshold_m):
        """是否已收敛。"""
        return self.initialized and self.position_uncertainty_m() < threshold_m

    # ── 内部方法 ──────────────────────────────────────────────────────

    def _mix_states(self):
        """步骤 1：交互混合。返回各模型的混合状态和协方差。"""
        mu = self.model_probs
        pi = self.transition

        # 混合概率 μ_{j|i} = π_{ji} * μ_j / c_i
        # c_i = Σ_j π_{ji} * μ_j
        c = [0.0] * 3
        for i in range(3):
            for j in range(3):
                c[i] += pi[j][i] * mu[j]

        mu_cond = [[0.0] * 3 for _ in range(3)]  # mu_cond[i][j] = μ_{j|i}
        for i in range(3):
            for j in range(3):
                mu_cond[i][j] = pi[j][i] * mu[j] / max(c[i], 1e-30)

        # 混合公共状态 (4D: east, north, ve, vn)
        cs = [
            self.cv.common_state(),
            self.ca.common_state(),
            self.ct.common_state(),
        ]
        cc = [
            self.cv.common_cov(),
            self.ca.common_cov(),
            self.ct.common_cov(),
        ]

        # 对每个模型 i，混合：x0_i = Σ_j μ_{j|i} * cs[j]
        #                       P0_i = Σ_j μ_{j|i} * (Pc[j] + (cs[j]-x0_i)(cs[j]-x0_i)^T)
        def mix_one(idx):
            x0 = [0.0] * 4
            for j in range(3):
                w = mu_cond[idx][j]
                for k in range(4):
                    x0[k] += w * cs[j][k]
            p0 = _zeros(4)
            for j in range(3):
                w = mu_cond[idx][j]
                diff = [cs[j][k] - x0[k] for k in range(4)]
                for r in range(4):
                    for col in range(4):
                        p0[r][col] += w * (cc[j][r][col] + diff[r] * diff[col])
            return x0, p0

        cv_x4, cv_p4 = mix_one(0)
        ca_x4, ca_p4 = mix_one(1)
        ct_x4, ct_p4 = mix_one(2)

        # CV (4D) — 直接使用
        cv_x = cv_x4
        cv_p = cv_p4

        # CA (6D) — 位置+速度从混合，加速度=0
        ca_x = list(ca_x4) + [0.0, 0.0]
        ca_p = _zeros(6)
        for r in range(4):
            for col in range(4):
                ca_p[r][col] = ca_p4[r][col]
        ca_p[4][4] = 1.0  # 加速度初始方差
        ca_p[5][5] = 1.0

        # CT (5D) — 位置+速度从混合，omega=0
        ct_x = list(ct_x4) + [0.0]
        ct_p = _zeros(5)
        for r in range(4):
            for col in range(4):
                ct_p[r][col] = ct_p4[r][col]
        ct_p[4][4] = 0.1  # omega 初始方差

        return cv_x, cv_p, ca_x, ca_p, ct_x, ct_p

    def _innovation_for_model(self, uav_e, uav_n, measured_bearing, model_idx):
        """计算模型 i 的方位角创新（用于似然）。"""
        model = (self.cv, self.ca, self.ct)[model_idx]
        de = model.x[0] - uav_e
        dn = model.x[1] - uav_n
        return _wrap_angle(measured_bearing - math.atan2(de, dn))

    def _update_model_probabilities(
        self, innov_cv, s_cv, innov_ca, s_ca, innov_ct, s_ct
    ):
        """步骤 3：更新模型概率。"""
        # 各模型似然 Λ_i = N(innov; 0, S_i)
        lam = [
            _gaussian_likelihood(innov_cv, s_cv),
            _gaussian_likelihood(innov_ca, s_ca),
            _gaussian_likelihood(innov_ct, s_ct),
        ]

        # c̄_i = Σ_j π_{ji} * μ_j（与 _mix_states 中的 c 相同）
        mu = self.model_probs
        pi = self.transition
        c_bar = [0.0] * 3
        for i in range(3):
            for j in range(3):
                c_bar[i] += pi[j][i] * mu[j]

        # 新模型概率 μ_i = Λ_i * c̄_i / Σ
        new_probs = [lam[i] * c_bar[i] for i in range(3)]
        total = sum(new_probs)
        if total > 1e-30:
            new_probs = [p / total for p in new_probs]
        else:
            new_probs = list(mu)  # 退化时保持原概率

        self.model_probs = new_probs

    def _fused_common_state(self):
        mu = self.model_probs
        cv = self.cv.common_state()
        ca = self.ca.common_state()
        ct = self.ct.common_state()
        return [mu[0] * cv[i] + mu[1] * ca[i] + mu[2] * ct[i] for i in range(4)]

    def _fused_common_cov(self):
        mu = self.model_probs
        states = [self.cv.common_state(), self.ca.common_state(), self.ct.common_state()]
        covs = [self.cv.common_cov(), self.ca.common_cov(), self.ct.common_cov()]
        fused_x = self._fused_common_state()

        result = _zeros(4)
        for m in range(3):
            diff = [states[m][k] - fused_x[k] for k in range(4)]
            for r in range(4):
                for col in range(4):
                    result[r][col] += mu[m] * (
                        covs[m][r][col] + diff[r] * diff[col]
                    )
        return result


__all__ = ["ImmFilter"]

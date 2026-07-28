//! ekf.rs — Bearing-Only 扩展卡尔曼滤波器。
//!
//! 状态向量：[east, north, v_east, v_north]（局部切平面，米/米每秒）
//! 运动模型：恒速（CV）
//! 观测模型：方位角 bearing = atan2(east, north)，可选距离

use pyo3::prelude::*;

const STATE_DIM: usize = 4;
const EARTH_RADIUS_M: f64 = 6_371_000.0;

// ── 辅助：4×4 矩阵运算 ────────────────────────────────────────────────

type Mat4 = [[f64; STATE_DIM]; STATE_DIM];
type Vec4 = [f64; STATE_DIM];

fn mat4_zero() -> Mat4 {
    [[0.0; STATE_DIM]; STATE_DIM]
}

fn mat4_identity() -> Mat4 {
    let mut m = mat4_zero();
    for i in 0..STATE_DIM {
        m[i][i] = 1.0;
    }
    m
}

/// C = A * B
fn mat4_mul(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut c = mat4_zero();
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            for k in 0..STATE_DIM {
                c[i][j] += a[i][k] * b[k][j];
            }
        }
    }
    c
}

/// C = A * B^T（A 4×4, B 4×4 → C 4×4）
fn mat4_mul_transpose_b(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut c = mat4_zero();
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            for k in 0..STATE_DIM {
                c[i][j] += a[i][k] * b[j][k]; // b[j][k] = b^T[k][j]
            }
        }
    }
    c
}

/// C = A + B
fn mat4_add(a: &Mat4, b: &Mat4) -> Mat4 {
    let mut c = mat4_zero();
    for i in 0..STATE_DIM {
        for j in 0..STATE_DIM {
            c[i][j] = a[i][j] + b[i][j];
        }
    }
    c
}

// ── WGS84 ↔ 局部坐标转换 ──────────────────────────────────────────────

#[inline]
fn wgs84_to_local(lat: f64, lon: f64, olat: f64, olon: f64) -> (f64, f64) {
    let east = (lon - olon).to_radians() * (olat.to_radians()).cos() * EARTH_RADIUS_M;
    let north = (lat - olat).to_radians() * EARTH_RADIUS_M;
    (east, north)
}

#[inline]
fn local_to_wgs84(east: f64, north: f64, olat: f64, olon: f64) -> (f64, f64) {
    let rlat_o = olat.to_radians();
    let rlon_o = olon.to_radians();
    let dlat = north / EARTH_RADIUS_M;
    let dlon = east / (rlat_o.cos() * EARTH_RADIUS_M);
    ((rlat_o + dlat).to_degrees(), (rlon_o + dlon).to_degrees())
}

// ── EKF 实现 ───────────────────────────────────────────────────────────

#[pyclass]
pub struct BearingOnlyEKF {
    x: Vec4,            // 状态：[east, north, v_east, v_north]
    p: Mat4,            // 协方差
    origin_lat: f64,
    origin_lon: f64,
    initialized: bool,
}

#[pymethods]
impl BearingOnlyEKF {
    #[new]
    pub fn new(origin_lat: f64, origin_lon: f64) -> Self {
        Self {
            x: [0.0; STATE_DIM],
            p: mat4_identity(),  // 初始协方差会在 initialize 时设置
            origin_lat,
            origin_lon,
            initialized: false,
        }
    }

    /// 初始化：从首次检测建立初始估计。
    ///
    /// - uav_lat/lon: UAV 当前位置
    /// - bearing_rad: 目标方位角（弧度，北=0，东=π/2）
    /// - assumed_range_m: 假设的初始距离（用于确定初始位置）
    pub fn initialize(
        &mut self,
        uav_lat: f64,
        uav_lon: f64,
        bearing_rad_val: f64,
        assumed_range_m: f64,
    ) {
        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);

        // 从 UAV 位置沿方位角推进 assumed_range_m
        // bearing: 北=0 → east = range*sin(bearing), north = range*cos(bearing)
        let target_e = uav_e + assumed_range_m * bearing_rad_val.sin();
        let target_n = uav_n + assumed_range_m * bearing_rad_val.cos();

        self.x = [target_e, target_n, 0.0, 0.0];

        // 初始协方差：位置不确定度大（距离未知），速度不确定度中等
        let pos_var = (assumed_range_m * 0.5).powi(2); // 50% 距离不确定
        let vel_var = 100.0; // 10 m/s 速度不确定
        self.p = mat4_zero();
        self.p[0][0] = pos_var;
        self.p[1][1] = pos_var;
        self.p[2][2] = vel_var;
        self.p[3][3] = vel_var;

        self.initialized = true;
    }

    /// 时间更新：CV 模型预测。
    ///
    /// - dt: 时间步长（秒）
    pub fn predict(&mut self, dt: f64) {
        if !self.initialized {
            return;
        }

        // 状态转移：pos += vel * dt
        self.x[0] += self.x[2] * dt;
        self.x[1] += self.x[3] * dt;

        // 状态转移矩阵 F
        // [1  0  dt 0 ]
        // [0  1  0  dt]
        // [0  0  1  0 ]
        // [0  0  0  1 ]
        let mut f = mat4_identity();
        f[0][2] = dt;
        f[1][3] = dt;

        // 过程噪声 Q（离散白噪声加速度模型）
        let accel_var = 1.0_f64.powi(2); // 加速度方差 1 m/s²（保守，避免过度膨胀）
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;
        let mut q = mat4_zero();
        // 位置-位置块
        q[0][0] = dt4 / 4.0 * accel_var;
        q[1][1] = dt4 / 4.0 * accel_var;
        // 速度-速度块
        q[2][2] = dt2 * accel_var;
        q[3][3] = dt2 * accel_var;
        // 位置-速度交叉块
        q[0][2] = dt3 / 2.0 * accel_var;
        q[2][0] = dt3 / 2.0 * accel_var;
        q[1][3] = dt3 / 2.0 * accel_var;
        q[3][1] = dt3 / 2.0 * accel_var;

        // P = F * P * F^T + Q
        let fp = mat4_mul(&f, &self.p);
        self.p = mat4_add(&mat4_mul_transpose_b(&fp, &f), &q);
    }

    /// 观测更新：方位角观测（主观测通道）。
    ///
    /// 使用 Joseph 形式更新协方差，保证数值稳定性。
    /// - uav_lat/lon: UAV 当前位置（用于计算相对方位）
    /// - measured_bearing_rad: 测量的方位角（弧度）
    pub fn update_bearing(
        &mut self,
        uav_lat: f64,
        uav_lon: f64,
        measured_bearing_rad: f64,
    ) {
        if !self.initialized {
            return;
        }

        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);

        // 相对位置
        let de = self.x[0] - uav_e;
        let dn = self.x[1] - uav_n;
        let r2 = de * de + dn * dn;

        if r2 < 1.0 {
            return; // 目标在 UAV 正上方，雅可比退化
        }

        // 预测的方位角
        let pred_bearing = de.atan2(dn);

        // 观测雅可比 H（1×4，只用方位角）
        // h(x) = atan2(east - uav_east, north - uav_north)
        // ∂h/∂east = north_diff / r², ∂h/∂north = -east_diff / r²
        let h_e = dn / r2;
        let h_n = -de / r2;

        // 创新（innovation），处理角度环绕
        let mut innov = measured_bearing_rad - pred_bearing;
        if innov > std::f64::consts::PI {
            innov -= 2.0 * std::f64::consts::PI;
        } else if innov < -std::f64::consts::PI {
            innov += 2.0 * std::f64::consts::PI;
        }

        // 观测噪声 R（方位角噪声，~1.1° = 0.02 rad）
        let r_noise = (0.02_f64).powi(2);

        // S = H * P * H^T + R（标量）
        let mut hp = [0.0; STATE_DIM];
        for j in 0..STATE_DIM {
            hp[j] = h_e * self.p[0][j] + h_n * self.p[1][j];
        }
        let s = hp[0] * h_e + hp[1] * h_n + r_noise;

        if s < 1e-30 || s.is_nan() {
            return;
        }

        // 卡尔曼增益 K = P * H^T / S
        let mut k = [0.0; STATE_DIM];
        for i in 0..STATE_DIM {
            k[i] = (self.p[i][0] * h_e + self.p[i][1] * h_n) / s;
        }

        // 状态更新 x += K * innov
        for i in 0..STATE_DIM {
            self.x[i] += k[i] * innov;
        }

        // Joseph 形式协方差更新：P = (I-KH)P(I-KH)^T + K*R*K^T
        // 逐元素实现，保证 P 始终正半定
        let mut p_new = [[0.0_f64; STATE_DIM]; STATE_DIM];
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                // (I-KH)P(I-KH)^T 项
                let mut val = self.p[i][j];
                val -= k[i] * (h_e * self.p[0][j] + h_n * self.p[1][j]);
                val -= (h_e * self.p[i][0] + h_n * self.p[i][1]) * k[j];
                val += k[i] * s * k[j]; // 等价于 k[i] * (HPH^T+R) * k[j]
                // K*R*K^T 项（R 是标量）
                val += k[i] * r_noise * k[j];
                p_new[i][j] = val;
            }
        }
        self.p = p_new;

        // 对称化 + 对角钳制（防止数值漂移）
        for i in 0..STATE_DIM {
            if self.p[i][i] < 1e-6 {
                self.p[i][i] = 1e-6;
            }
            for j in (i + 1)..STATE_DIM {
                let avg = (self.p[i][j] + self.p[j][i]) * 0.5;
                self.p[i][j] = avg;
                self.p[j][i] = avg;
            }
        }
    }

    /// 观测更新：距离观测（可选，从 tilt 角几何推算）。
    pub fn update_range(
        &mut self,
        uav_lat: f64,
        uav_lon: f64,
        measured_range_m: f64,
    ) {
        if !self.initialized {
            return;
        }

        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);

        let de = self.x[0] - uav_e;
        let dn = self.x[1] - uav_n;
        let r = (de * de + dn * dn).sqrt();

        if r < 1e-3 {
            return;
        }

        let pred_range = r;

        // 观测雅可比 H_range (1×4)
        let h_e = de / r;
        let h_n = dn / r;

        let innov = measured_range_m - pred_range;

        let r_noise = (50.0_f64).powi(2); // 50m 标准差

        // S = H * P * H^T + R
        let mut hp = [0.0; STATE_DIM];
        for j in 0..STATE_DIM {
            hp[j] = h_e * self.p[0][j] + h_n * self.p[1][j];
        }
        let s = hp[0] * h_e + hp[1] * h_n + r_noise;

        if s < 1e-30 || s.is_nan() {
            return;
        }

        let mut k = [0.0; STATE_DIM];
        for i in 0..STATE_DIM {
            k[i] = (self.p[i][0] * h_e + self.p[i][1] * h_n) / s;
        }

        for i in 0..STATE_DIM {
            self.x[i] += k[i] * innov;
        }

        // Joseph 形式
        let mut p_new = [[0.0_f64; STATE_DIM]; STATE_DIM];
        for i in 0..STATE_DIM {
            for j in 0..STATE_DIM {
                let mut val = self.p[i][j];
                val -= k[i] * (h_e * self.p[0][j] + h_n * self.p[1][j]);
                val -= (h_e * self.p[i][0] + h_n * self.p[i][1]) * k[j];
                val += k[i] * s * k[j];
                val += k[i] * r_noise * k[j];
                p_new[i][j] = val;
            }
        }
        self.p = p_new;

        for i in 0..STATE_DIM {
            if self.p[i][i] < 1e-6 {
                self.p[i][i] = 1e-6;
            }
            for j in (i + 1)..STATE_DIM {
                let avg = (self.p[i][j] + self.p[j][i]) * 0.5;
                self.p[i][j] = avg;
                self.p[j][i] = avg;
            }
        }
    }

    /// 返回目标估计位置（WGS84）。
    pub fn position_wgs84(&self) -> (f64, f64) {
        local_to_wgs84(self.x[0], self.x[1], self.origin_lat, self.origin_lon)
    }

    /// 返回速度估计 (v_east, v_north) m/s。
    pub fn velocity_mps(&self) -> (f64, f64) {
        (self.x[2], self.x[3])
    }

    /// 返回速度大小 m/s。
    pub fn speed_mps(&self) -> f64 {
        (self.x[2].powi(2) + self.x[3].powi(2)).sqrt()
    }

    /// 返回位置不确定性（sqrt(trace(P[:2,:2]))）。
    pub fn position_uncertainty_m(&self) -> f64 {
        (self.p[0][0] + self.p[1][1]).sqrt()
    }

    /// 是否已初始化。
    pub fn is_initialized(&self) -> bool {
        self.initialized
    }

    /// 是否已收敛（位置不确定性 < threshold）。
    pub fn is_converged(&self, threshold_m: f64) -> bool {
        self.initialized && self.position_uncertainty_m() < threshold_m
    }
}

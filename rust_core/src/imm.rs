//! imm.rs — Interacting Multiple Model (IMM) 滤波器。
//!
//! 三个运动模型：
//! - CV (Constant Velocity): 4D [east, north, v_east, v_north]
//! - CA (Constant Acceleration): 6D [..., a_east, a_north]
//! - CT (Coordinated Turn): 5D [..., omega]
//!
//! 观测模型：方位角 bearing（可选距离），只依赖位置分量。

use nalgebra::{SMatrix, SVector};

#[allow(dead_code)] // init_from_common 等方法为 IMM 内部使用预留

// ── 常量 ────────────────────────────────────────────────────────────────

const EARTH_RADIUS_M: f64 = 6_371_000.0;

// ── 坐标转换 ────────────────────────────────────────────────────────────

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

// ── CV 模型 (4D) ────────────────────────────────────────────────────────

pub(crate) struct CvModel {
    pub x: SVector<f64, 4>,
    pub p: SMatrix<f64, 4, 4>,
    pub initialized: bool,
}

impl CvModel {
    pub fn new() -> Self {
        Self {
            x: SVector::<f64, 4>::zeros(),
            p: SMatrix::<f64, 4, 4>::identity(),
            initialized: false,
        }
    }

    pub fn initialize(&mut self, east: f64, north: f64, ve: f64, vn: f64, pos_var: f64, vel_var: f64) {
        self.x = SVector::<f64, 4>::new(east, north, ve, vn);
        self.p = SMatrix::<f64, 4, 4>::identity();
        self.p[(0, 0)] = pos_var;
        self.p[(1, 1)] = pos_var;
        self.p[(2, 2)] = vel_var;
        self.p[(3, 3)] = vel_var;
        self.initialized = true;
    }

    /// 从公共状态 [east, north, ve, vn] 初始化，模型特定分量置零。
    #[allow(dead_code)]
    pub fn init_from_common(&mut self, common: &[f64], pos_var: f64, vel_var: f64) {
        self.initialize(common[0], common[1], common[2], common[3], pos_var, vel_var);
    }

    /// 提取公共状态 [east, north, ve, vn]。
    pub fn common_state(&self) -> [f64; 4] {
        [self.x[0], self.x[1], self.x[2], self.x[3]]
    }

    /// 提取公共协方差 (4×4)。
    pub fn common_cov(&self) -> SMatrix<f64, 4, 4> {
        self.p
    }

    pub fn predict(&mut self, dt: f64) {
        if !self.initialized {
            return;
        }
        // 状态转移
        self.x[0] += self.x[2] * dt;
        self.x[1] += self.x[3] * dt;

        // F 矩阵
        let mut f = SMatrix::<f64, 4, 4>::identity();
        f[(0, 2)] = dt;
        f[(1, 3)] = dt;

        // 过程噪声 (DWNA, accel_var=1 m/s²)
        let accel_var = 1.0_f64;
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;

        let mut q = SMatrix::<f64, 4, 4>::zeros();
        q[(0, 0)] = dt4 / 4.0 * accel_var;
        q[(1, 1)] = dt4 / 4.0 * accel_var;
        q[(2, 2)] = dt2 * accel_var;
        q[(3, 3)] = dt2 * accel_var;
        q[(0, 2)] = dt3 / 2.0 * accel_var;
        q[(2, 0)] = dt3 / 2.0 * accel_var;
        q[(1, 3)] = dt3 / 2.0 * accel_var;
        q[(3, 1)] = dt3 / 2.0 * accel_var;

        self.p = f * self.p * f.transpose() + q;
    }

    /// 方位角观测更新。返回 (innovation, S) 供 IMM 似然计算。
    pub fn update_bearing(
        &mut self,
        uav_e: f64,
        uav_n: f64,
        measured_bearing: f64,
    ) -> (f64, f64) {
        let (innov, s, h) = bearing_innovation_and_jacobian::<4>(
            &self.x, &self.p, uav_e, uav_n, measured_bearing,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<4>(&mut self.x, &mut self.p, &h, innov, 0.02_f64.powi(2), s);
        }
        (innov, s)
    }

    /// 距离观测更新。
    pub fn update_range(&mut self, uav_e: f64, uav_n: f64, measured_range: f64) {
        let (innov, s, h) = range_innovation_and_jacobian::<4>(
            &self.x, &self.p, uav_e, uav_n, measured_range,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<4>(&mut self.x, &mut self.p, &h, innov, 50.0_f64.powi(2), s);
        }
    }
}

// ── CA 模型 (6D) ────────────────────────────────────────────────────────

pub(crate) struct CaModel {
    pub x: SVector<f64, 6>,
    pub p: SMatrix<f64, 6, 6>,
    pub initialized: bool,
}

impl CaModel {
    pub fn new() -> Self {
        Self {
            x: SVector::<f64, 6>::zeros(),
            p: SMatrix::<f64, 6, 6>::identity(),
            initialized: false,
        }
    }

    pub fn initialize(&mut self, east: f64, north: f64, ve: f64, vn: f64, ae: f64, an: f64,
                       pos_var: f64, vel_var: f64, accel_var: f64) {
        self.x = SVector::<f64, 6>::new(east, north, ve, vn, ae, an);
        self.p = SMatrix::<f64, 6, 6>::identity();
        self.p[(0, 0)] = pos_var;
        self.p[(1, 1)] = pos_var;
        self.p[(2, 2)] = vel_var;
        self.p[(3, 3)] = vel_var;
        self.p[(4, 4)] = accel_var;
        self.p[(5, 5)] = accel_var;
        self.initialized = true;
    }

    #[allow(dead_code)]
    pub fn init_from_common(&mut self, common: &[f64], pos_var: f64, vel_var: f64) {
        // CA: 加速度初始化为零
        self.initialize(common[0], common[1], common[2], common[3], 0.0, 0.0,
                        pos_var, vel_var, 1.0);
    }

    pub fn common_state(&self) -> [f64; 4] {
        [self.x[0], self.x[1], self.x[2], self.x[3]]
    }

    pub fn common_cov(&self) -> SMatrix<f64, 4, 4> {
        self.p.fixed_view::<4, 4>(0, 0).into()
    }

    pub fn predict(&mut self, dt: f64) {
        if !self.initialized {
            return;
        }
        // 状态转移：pos += vel*dt + 0.5*accel*dt², vel += accel*dt
        self.x[0] += self.x[2] * dt + 0.5 * self.x[4] * dt * dt;
        self.x[1] += self.x[3] * dt + 0.5 * self.x[5] * dt * dt;
        self.x[2] += self.x[4] * dt;
        self.x[3] += self.x[5] * dt;
        // 加速度保持不变 (CA 模型假设)

        // F 矩阵
        let mut f = SMatrix::<f64, 6, 6>::identity();
        let dt2_2 = 0.5 * dt * dt;
        f[(0, 2)] = dt;
        f[(1, 3)] = dt;
        f[(0, 4)] = dt2_2;
        f[(1, 5)] = dt2_2;
        f[(2, 4)] = dt;
        f[(3, 5)] = dt;

        // 过程噪声 (jerk 噪声模型, jerk_var=0.5 m/s³)
        let jerk_var = 0.5_f64.powi(2);
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;
        let dt5 = dt4 * dt;
        let dt6 = dt5 * dt;

        let q_pp = dt6 / 36.0 * jerk_var;
        let q_pv = dt5 / 12.0 * jerk_var;
        let q_pa = dt4 / 6.0 * jerk_var;
        let q_vv = dt4 / 4.0 * jerk_var;
        let q_va = dt3 / 2.0 * jerk_var;
        let q_aa = dt2 * jerk_var;

        let mut q = SMatrix::<f64, 6, 6>::zeros();
        // 位置-位置
        q[(0, 0)] = q_pp; q[(1, 1)] = q_pp;
        // 速度-速度
        q[(2, 2)] = q_vv; q[(3, 3)] = q_vv;
        // 加速度-加速度
        q[(4, 4)] = q_aa; q[(5, 5)] = q_aa;
        // 位置-速度
        q[(0, 2)] = q_pv; q[(2, 0)] = q_pv;
        q[(1, 3)] = q_pv; q[(3, 1)] = q_pv;
        // 位置-加速度
        q[(0, 4)] = q_pa; q[(4, 0)] = q_pa;
        q[(1, 5)] = q_pa; q[(5, 1)] = q_pa;
        // 速度-加速度
        q[(2, 4)] = q_va; q[(4, 2)] = q_va;
        q[(3, 5)] = q_va; q[(5, 3)] = q_va;

        self.p = f * self.p * f.transpose() + q;
    }

    pub fn update_bearing(
        &mut self,
        uav_e: f64,
        uav_n: f64,
        measured_bearing: f64,
    ) -> (f64, f64) {
        let (innov, s, h) = bearing_innovation_and_jacobian::<6>(
            &self.x, &self.p, uav_e, uav_n, measured_bearing,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<6>(&mut self.x, &mut self.p, &h, innov, 0.02_f64.powi(2), s);
        }
        (innov, s)
    }

    pub fn update_range(&mut self, uav_e: f64, uav_n: f64, measured_range: f64) {
        let (innov, s, h) = range_innovation_and_jacobian::<6>(
            &self.x, &self.p, uav_e, uav_n, measured_range,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<6>(&mut self.x, &mut self.p, &h, innov, 50.0_f64.powi(2), s);
        }
    }
}

// ── CT 模型 (5D) ────────────────────────────────────────────────────────

pub(crate) struct CtModel {
    pub x: SVector<f64, 5>,
    pub p: SMatrix<f64, 5, 5>,
    pub initialized: bool,
}

impl CtModel {
    pub fn new() -> Self {
        Self {
            x: SVector::<f64, 5>::zeros(),
            p: SMatrix::<f64, 5, 5>::identity(),
            initialized: false,
        }
    }

    pub fn initialize(&mut self, east: f64, north: f64, ve: f64, vn: f64, omega: f64,
                       pos_var: f64, vel_var: f64, omega_var: f64) {
        self.x = SVector::<f64, 5>::new(east, north, ve, vn, omega);
        self.p = SMatrix::<f64, 5, 5>::identity();
        self.p[(0, 0)] = pos_var;
        self.p[(1, 1)] = pos_var;
        self.p[(2, 2)] = vel_var;
        self.p[(3, 3)] = vel_var;
        self.p[(4, 4)] = omega_var;
        self.initialized = true;
    }

    #[allow(dead_code)]
    pub fn init_from_common(&mut self, common: &[f64], pos_var: f64, vel_var: f64) {
        // CT: 转弯率初始化为零
        self.initialize(common[0], common[1], common[2], common[3], 0.0,
                        pos_var, vel_var, 0.1);
    }

    pub fn common_state(&self) -> [f64; 4] {
        [self.x[0], self.x[1], self.x[2], self.x[3]]
    }

    pub fn common_cov(&self) -> SMatrix<f64, 4, 4> {
        self.p.fixed_view::<4, 4>(0, 0).into()
    }

    pub fn predict(&mut self, dt: f64) {
        if !self.initialized {
            return;
        }

        let omega = self.x[4];
        let wt = omega * dt;

        // 状态转移：精确 CT 模型
        // sinc_wt = sin(wt)/omega, versine_wt = (1-cos(wt))/omega
        // 小角度用 Taylor 展开避免除以 omega 的符号问题
        let (sinc_wt, versine_wt) = if wt.abs() < 0.1 {
            // sinc(wt)/omega = dt * (1 - wt²/6 + wt⁴/120)
            // versine(wt)/omega = omega*dt²/2 * (1 - wt²/12)
            let dt2 = dt * dt;
            let wt2 = wt * wt;
            let s = dt * (1.0 - wt2 / 6.0 + wt2 * wt2 / 120.0);
            let v = omega * dt2 / 2.0 * (1.0 - wt2 / 12.0);
            (s, v)
        } else {
            (wt.sin() / omega, (1.0 - wt.cos()) / omega)
        };

        // pos += ve*sinc - vn*versine, pos += ve*versine + vn*sinc
        let east_new = self.x[0] + self.x[2] * sinc_wt - self.x[3] * versine_wt;
        let north_new = self.x[1] + self.x[2] * versine_wt + self.x[3] * sinc_wt;
        let cos_wt = wt.cos();
        let sin_wt = wt.sin();
        let ve_new = self.x[2] * cos_wt - self.x[3] * sin_wt;
        let vn_new = self.x[2] * sin_wt + self.x[3] * cos_wt;
        self.x[0] = east_new;
        self.x[1] = north_new;
        self.x[2] = ve_new;
        self.x[3] = vn_new;
        // omega 保持不变

        // 雅可比 F（EKF 线性化）
        let mut f = SMatrix::<f64, 5, 5>::identity();
        if wt.abs() < 1e-6 {
            // omega → 0 退化为 CV
            f[(0, 2)] = dt;
            f[(1, 3)] = dt;
        } else {
            let sin_wt = wt.sin();
            let cos_wt = wt.cos();
            let inv_w = 1.0 / omega;
            let inv_w2 = inv_w * inv_w;

            // ∂pos/∂vel
            f[(0, 2)] = inv_w * sin_wt;
            f[(0, 3)] = -inv_w * (1.0 - cos_wt);
            f[(1, 2)] = inv_w * (1.0 - cos_wt);
            f[(1, 3)] = inv_w * sin_wt;

            // ∂vel/∂vel
            f[(2, 2)] = cos_wt;
            f[(2, 3)] = -sin_wt;
            f[(3, 2)] = sin_wt;
            f[(3, 3)] = cos_wt;

            // ∂pos/∂omega
            f[(0, 4)] = inv_w2 * (self.x[2] * (wt * cos_wt - sin_wt) + self.x[3] * (wt * sin_wt - 1.0 + cos_wt));
            f[(1, 4)] = inv_w2 * (-self.x[2] * (wt * sin_wt - 1.0 + cos_wt) + self.x[3] * (wt * cos_wt - sin_wt));

            // ∂vel/∂omega
            f[(2, 4)] = -dt * (self.x[2] * sin_wt + self.x[3] * cos_wt);
            f[(3, 4)] = dt * (self.x[2] * cos_wt - self.x[3] * sin_wt);
        }

        // 过程噪声
        let accel_var = 1.0_f64;
        let omega_noise_var = 0.1_f64.powi(2); // 转弯率噪声
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;

        let mut q = SMatrix::<f64, 5, 5>::zeros();
        // 位置噪声
        q[(0, 0)] = dt4 / 4.0 * accel_var;
        q[(1, 1)] = dt4 / 4.0 * accel_var;
        // 速度噪声
        q[(2, 2)] = dt2 * accel_var;
        q[(3, 3)] = dt2 * accel_var;
        // 位置-速度交叉
        q[(0, 2)] = dt3 / 2.0 * accel_var;
        q[(2, 0)] = dt3 / 2.0 * accel_var;
        q[(1, 3)] = dt3 / 2.0 * accel_var;
        q[(3, 1)] = dt3 / 2.0 * accel_var;
        // 转弯率噪声
        q[(4, 4)] = dt2 * omega_noise_var;

        self.p = f * self.p * f.transpose() + q;
    }

    pub fn update_bearing(
        &mut self,
        uav_e: f64,
        uav_n: f64,
        measured_bearing: f64,
    ) -> (f64, f64) {
        let (innov, s, h) = bearing_innovation_and_jacobian::<5>(
            &self.x, &self.p, uav_e, uav_n, measured_bearing,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<5>(&mut self.x, &mut self.p, &h, innov, 0.02_f64.powi(2), s);
        }
        (innov, s)
    }

    pub fn update_range(&mut self, uav_e: f64, uav_n: f64, measured_range: f64) {
        let (innov, s, h) = range_innovation_and_jacobian::<5>(
            &self.x, &self.p, uav_e, uav_n, measured_range,
        );
        if s > 1e-30 && !s.is_nan() {
            joseph_update::<5>(&mut self.x, &mut self.p, &h, innov, 50.0_f64.powi(2), s);
        }
    }
}

// ── 观测模型公共函数 ────────────────────────────────────────────────────

/// 方位角观测：h(x) = atan2(east - uav_e, north - uav_n)
/// 返回 (innovation, S, H_row)
fn bearing_innovation_and_jacobian<const N: usize>(
    x: &SVector<f64, N>,
    p: &SMatrix<f64, N, N>,
    uav_e: f64,
    uav_n: f64,
    measured_bearing: f64,
) -> (f64, f64, SVector<f64, N>) {
    let de = x[0] - uav_e;
    let dn = x[1] - uav_n;
    let r2 = de * de + dn * dn;

    let mut h = SVector::<f64, N>::zeros();
    if r2 < 1.0 {
        return (0.0, 1e30, h);
    }

    let pred_bearing = de.atan2(dn);
    let mut innov = measured_bearing - pred_bearing;
    if innov > std::f64::consts::PI {
        innov -= 2.0 * std::f64::consts::PI;
    } else if innov < -std::f64::consts::PI {
        innov += 2.0 * std::f64::consts::PI;
    }

    // 雅可比（只在位置分量非零）
    h[0] = dn / r2;
    h[1] = -de / r2;

    // S = H * P * H^T + R
    let hp = p * &h;
    let s = h.dot(&hp) + 0.02_f64.powi(2);

    (innov, s, h)
}

/// 距离观测：h(x) = sqrt((x[0]-uav_e)² + (x[1]-uav_n)²)
fn range_innovation_and_jacobian<const N: usize>(
    x: &SVector<f64, N>,
    p: &SMatrix<f64, N, N>,
    uav_e: f64,
    uav_n: f64,
    measured_range: f64,
) -> (f64, f64, SVector<f64, N>) {
    let de = x[0] - uav_e;
    let dn = x[1] - uav_n;
    let r = (de * de + dn * dn).sqrt();

    let mut h = SVector::<f64, N>::zeros();
    if r < 1e-3 {
        return (0.0, 1e30, h);
    }

    let innov = measured_range - r;

    h[0] = de / r;
    h[1] = dn / r;

    let hp = p * &h;
    let s = h.dot(&hp) + 50.0_f64.powi(2);

    (innov, s, h)
}

/// Joseph 形式协方差更新：P = (I-KH)P(I-KH)^T + KRK^T
/// 其中 K = P*H^T / S，R_noise_var 是观测噪声方差。
fn joseph_update<const N: usize>(
    x: &mut SVector<f64, N>,
    p: &mut SMatrix<f64, N, N>,
    h: &SVector<f64, N>,
    innov: f64,
    r_noise_var: f64,
    s: f64,
) {
    // K = P * H^T / S
    let k = (&*p * h) / s;

    // 状态更新
    *x += &k * innov;

    // Joseph 形式
    let i_kh = SMatrix::<f64, N, N>::identity() - &k * h.transpose();
    *p = &i_kh * &*p * i_kh.transpose() + &k * r_noise_var * k.transpose();

    // 对称化 + 对角钳制
    for i in 0..N {
        if p[(i, i)] < 1e-6 {
            p[(i, i)] = 1e-6;
        }
        for j in (i + 1)..N {
            let avg = (p[(i, j)] + p[(j, i)]) * 0.5;
            p[(i, j)] = avg;
            p[(j, i)] = avg;
        }
    }
}

// ── IMM 滤波器 ──────────────────────────────────────────────────────────

/// Interacting Multiple Model 滤波器（CV + CA + CT）。
pub(crate) struct ImmFilterInner {
    pub cv: CvModel,
    pub ca: CaModel,
    pub ct: CtModel,
    /// 模型转移概率 [from][to]
    pub transition: [[f64; 3]; 3],
    /// 模型概率 [CV, CA, CT]
    pub model_probs: [f64; 3],
    pub origin_lat: f64,
    pub origin_lon: f64,
    pub initialized: bool,
}

impl ImmFilterInner {
    pub fn new(origin_lat: f64, origin_lon: f64) -> Self {
        Self {
            cv: CvModel::new(),
            ca: CaModel::new(),
            ct: CtModel::new(),
            transition: [
                [0.90, 0.05, 0.05],
                [0.10, 0.85, 0.05],
                [0.10, 0.05, 0.85],
            ],
            model_probs: [1.0 / 3.0; 3],
            origin_lat,
            origin_lon,
            initialized: false,
        }
    }

    pub fn initialize(
        &mut self,
        uav_lat: f64, uav_lon: f64,
        bearing_rad: f64, assumed_range_m: f64,
    ) {
        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);
        let target_e = uav_e + assumed_range_m * bearing_rad.sin();
        let target_n = uav_n + assumed_range_m * bearing_rad.cos();
        let pos_var = (assumed_range_m * 0.5).powi(2);
        let vel_var = 100.0;

        self.cv.initialize(target_e, target_n, 0.0, 0.0, pos_var, vel_var);
        self.ca.initialize(target_e, target_n, 0.0, 0.0, 0.0, 0.0, pos_var, vel_var, 1.0);
        self.ct.initialize(target_e, target_n, 0.0, 0.0, 0.0, pos_var, vel_var, 0.1);
        self.model_probs = [1.0 / 3.0; 3];
        self.initialized = true;
    }

    /// IMM 交互 + 预测。
    pub fn predict(&mut self, dt: f64) {
        if !self.initialized {
            return;
        }

        // ── 步骤 1：交互混合 ──
        let (cv_common, cv_cov, ca_common, ca_cov, ct_common, ct_cov) = self.mix_states();

        // 用混合后的状态/协方差初始化各模型
        let cv_p = cv_cov;
        let ca_p = ca_cov;
        let ct_p = ct_cov;

        self.cv.x = cv_common;
        self.cv.p = cv_p;
        self.ca.x = ca_common;
        self.ca.p = ca_p;
        self.ct.x = ct_common;
        self.ct.p = ct_p;

        // ── 步骤 2：各模型独立预测 ──
        self.cv.predict(dt);
        self.ca.predict(dt);
        self.ct.predict(dt);
    }

    /// 方位角观测更新。
    pub fn update_bearing(&mut self, uav_lat: f64, uav_lon: f64, measured_bearing: f64) {
        if !self.initialized {
            return;
        }
        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);

        // 步骤 1：计算各模型的先验创新（在状态更新之前）
        let innov_cv = self.innovation_for_model(uav_e, uav_n, measured_bearing, 0);
        let innov_ca = self.innovation_for_model(uav_e, uav_n, measured_bearing, 1);
        let innov_ct = self.innovation_for_model(uav_e, uav_n, measured_bearing, 2);

        // 步骤 2：各模型独立更新（修改状态 + 协方差）
        let (_, s_cv) = self.cv.update_bearing(uav_e, uav_n, measured_bearing);
        let (_, s_ca) = self.ca.update_bearing(uav_e, uav_n, measured_bearing);
        let (_, s_ct) = self.ct.update_bearing(uav_e, uav_n, measured_bearing);

        // 步骤 3：用先验创新和 S 更新模型概率
        self.update_model_probabilities(innov_cv, s_cv, innov_ca, s_ca, innov_ct, s_ct);
    }

    /// 距离观测更新。
    pub fn update_range(&mut self, uav_lat: f64, uav_lon: f64, measured_range: f64) {
        if !self.initialized {
            return;
        }
        let (uav_e, uav_n) = wgs84_to_local(uav_lat, uav_lon, self.origin_lat, self.origin_lon);
        self.cv.update_range(uav_e, uav_n, measured_range);
        self.ca.update_range(uav_e, uav_n, measured_range);
        self.ct.update_range(uav_e, uav_n, measured_range);
    }

    /// 融合状态：加权各模型的公共状态。
    pub fn fused_position_wgs84(&self) -> (f64, f64) {
        let (e, n) = self.fused_position_local();
        local_to_wgs84(e, n, self.origin_lat, self.origin_lon)
    }

    pub fn fused_velocity_mps(&self) -> (f64, f64) {
        let cs = self.fused_common_state();
        (cs[2], cs[3])
    }

    pub fn fused_speed_mps(&self) -> f64 {
        let (ve, vn) = self.fused_velocity_mps();
        (ve * ve + vn * vn).sqrt()
    }

    pub fn fused_position_uncertainty_m(&self) -> f64 {
        let fc = self.fused_common_cov();
        (fc[(0, 0)] + fc[(1, 1)]).sqrt()
    }

    pub fn model_probabilities(&self) -> [f64; 3] {
        self.model_probs
    }

    pub fn is_converged(&self, threshold_m: f64) -> bool {
        self.initialized && self.fused_position_uncertainty_m() < threshold_m
    }

    // ── 内部方法 ──────────────────────────────────────────────────────

    /// 步骤 1：交互混合。返回各模型的混合状态和协方差。
    fn mix_states(&self) -> (
        SVector<f64, 4>, SMatrix<f64, 4, 4>,
        SVector<f64, 6>, SMatrix<f64, 6, 6>,
        SVector<f64, 5>, SMatrix<f64, 5, 5>,
    ) {
        let mu = self.model_probs;
        let pi = &self.transition;

        // 混合概率 μ_{j|i} = π_{ji} * μ_j / c_i
        // c_i = Σ_j π_{ji} * μ_j
        let mut c = [0.0_f64; 3];
        for i in 0..3 {
            for j in 0..3 {
                c[i] += pi[j][i] * mu[j];
            }
        }

        let mut mu_cond = [[0.0_f64; 3]; 3]; // mu_cond[i][j] = μ_{j|i}
        for i in 0..3 {
            for j in 0..3 {
                mu_cond[i][j] = pi[j][i] * mu[j] / c[i].max(1e-30);
            }
        }

        // 混合公共状态 (4D: east, north, ve, vn)
        let cs = [
            self.cv.common_state(),
            self.ca.common_state(),
            self.ct.common_state(),
        ];
        let cc = [
            self.cv.common_cov(),
            self.ca.common_cov(),
            self.ct.common_cov(),
        ];

        // 对每个模型 i，混合：x0_i = Σ_j μ_{j|i} * cs[j]
        //                       P0_i = Σ_j μ_{j|i} * (Pc[j] + (cs[j]-x0_i)(cs[j]-x0_i)^T)
        fn mix_one(
            idx: usize,
            mu_cond: &[[f64; 3]; 3],
            cs: &[[f64; 4]; 3],
            cc: &[SMatrix<f64, 4, 4>; 3],
        ) -> (SVector<f64, 4>, SMatrix<f64, 4, 4>) {
            let mut x0 = SVector::<f64, 4>::zeros();
            for j in 0..3 {
                x0 += mu_cond[idx][j] * SVector::<f64, 4>::from(cs[j]);
            }
            let mut p0 = SMatrix::<f64, 4, 4>::zeros();
            for j in 0..3 {
                let diff = SVector::<f64, 4>::from(cs[j]) - x0;
                p0 += mu_cond[idx][j] * (cc[j] + diff * diff.transpose());
            }
            (x0, p0)
        }

        let (cv_x4, cv_p4) = mix_one(0, &mu_cond, &cs, &cc);
        let (ca_x4, ca_p4) = mix_one(1, &mu_cond, &cs, &cc);
        let (ct_x4, ct_p4) = mix_one(2, &mu_cond, &cs, &cc);

        // CV (4D) — 直接使用
        let cv_x = cv_x4;
        let cv_p = cv_p4;

        // CA (6D) — 位置+速度从混合，加速度=0
        let mut ca_x = SVector::<f64, 6>::zeros();
        for i in 0..4 { ca_x[i] = ca_x4[i]; }
        let mut ca_p = SMatrix::<f64, 6, 6>::zeros();
        ca_p.fixed_view_mut::<4, 4>(0, 0).copy_from(&ca_p4);
        ca_p[(4, 4)] = 1.0; // 加速度初始方差
        ca_p[(5, 5)] = 1.0;

        // CT (5D) — 位置+速度从混合，omega=0
        let mut ct_x = SVector::<f64, 5>::zeros();
        for i in 0..4 { ct_x[i] = ct_x4[i]; }
        ct_x[4] = 0.0; // omega 初始为 0
        let mut ct_p = SMatrix::<f64, 5, 5>::zeros();
        ct_p.fixed_view_mut::<4, 4>(0, 0).copy_from(&ct_p4);
        ct_p[(4, 4)] = 0.1; // omega 初始方差

        (cv_x, cv_p, ca_x, ca_p, ct_x, ct_p)
    }

    /// 计算模型 i 的方位角创新（用于似然）。
    fn innovation_for_model(&self, uav_e: f64, uav_n: f64, measured_bearing: f64, model_idx: usize) -> f64 {
        let (de, dn) = match model_idx {
            0 => (self.cv.x[0] - uav_e, self.cv.x[1] - uav_n),
            1 => (self.ca.x[0] - uav_e, self.ca.x[1] - uav_n),
            2 => (self.ct.x[0] - uav_e, self.ct.x[1] - uav_n),
            _ => return 0.0,
        };
        let pred = de.atan2(dn);
        let mut innov = measured_bearing - pred;
        if innov > std::f64::consts::PI {
            innov -= 2.0 * std::f64::consts::PI;
        } else if innov < -std::f64::consts::PI {
            innov += 2.0 * std::f64::consts::PI;
        }
        innov
    }

    /// 步骤 3：更新模型概率。
    fn update_model_probabilities(
        &mut self,
        innov_cv: f64, s_cv: f64,
        innov_ca: f64, s_ca: f64,
        innov_ct: f64, s_ct: f64,
    ) {
        // 各模型似然 Λ_i = N(innov; 0, S_i)
        let lambda_cv = gaussian_likelihood(innov_cv, s_cv);
        let lambda_ca = gaussian_likelihood(innov_ca, s_ca);
        let lambda_ct = gaussian_likelihood(innov_ct, s_ct);

        // c̄_i = Σ_j π_{ji} * μ_j（与 mix_states 中的 c 相同）
        let mu = self.model_probs;
        let pi = &self.transition;
        let mut c_bar = [0.0_f64; 3];
        for i in 0..3 {
            for j in 0..3 {
                c_bar[i] += pi[j][i] * mu[j];
            }
        }

        // 新模型概率 μ_i = Λ_i * c̄_i / Σ
        let lambda = [lambda_cv, lambda_ca, lambda_ct];
        let mut new_probs = [0.0_f64; 3];
        let mut sum = 0.0;
        for i in 0..3 {
            new_probs[i] = lambda[i] * c_bar[i];
            sum += new_probs[i];
        }
        if sum > 1e-30 {
            for i in 0..3 {
                new_probs[i] /= sum;
            }
        } else {
            new_probs = mu; // 退化时保持原概率
        }

        self.model_probs = new_probs;
    }

    fn fused_position_local(&self) -> (f64, f64) {
        let cs = self.fused_common_state();
        (cs[0], cs[1])
    }

    fn fused_common_state(&self) -> [f64; 4] {
        let mu = self.model_probs;
        let cv = self.cv.common_state();
        let ca = self.ca.common_state();
        let ct = self.ct.common_state();
        let mut result = [0.0_f64; 4];
        for i in 0..4 {
            result[i] = mu[0] * cv[i] + mu[1] * ca[i] + mu[2] * ct[i];
        }
        result
    }

    fn fused_common_cov(&self) -> SMatrix<f64, 4, 4> {
        let mu = self.model_probs;
        let cv_c = self.cv.common_state();
        let ca_c = self.ca.common_state();
        let ct_c = self.ct.common_state();
        let cv_p = self.cv.common_cov();
        let ca_p = self.ca.common_cov();
        let ct_p = self.ct.common_cov();
        let fused_x = self.fused_common_state();

        let mut result = SMatrix::<f64, 4, 4>::zeros();
        let states = [cv_c, ca_c, ct_c];
        let covs = [cv_p, ca_p, ct_p];
        for m in 0..3 {
            let diff = SVector::<f64, 4>::from(states[m]) - SVector::<f64, 4>::from(fused_x);
            result += mu[m] * (covs[m] + diff * diff.transpose());
        }
        result
    }
}

/// 高斯似然：N(innov; 0, S) = (2πS)^{-1/2} * exp(-innov²/(2S))
fn gaussian_likelihood(innov: f64, s: f64) -> f64 {
    if s < 1e-30 {
        return 1e-30;
    }
    let two_pi_s = 2.0 * std::f64::consts::PI * s;
    (-0.5 * innov * innov / s).exp() / two_pi_s.sqrt()
}

use pyo3::prelude::*;

mod geometry;
mod ekf;
mod imm;

// ── IMM PyO3 包装 ──────────────────────────────────────────────────────

#[pyclass]
struct ImmFilter {
    inner: imm::ImmFilterInner,
}

#[pymethods]
impl ImmFilter {
    #[new]
    fn new(origin_lat: f64, origin_lon: f64) -> Self {
        Self {
            inner: imm::ImmFilterInner::new(origin_lat, origin_lon),
        }
    }

    /// 初始化：从首次检测建立初始估计。
    fn initialize(
        &mut self,
        uav_lat: f64,
        uav_lon: f64,
        bearing_rad: f64,
        assumed_range_m: f64,
    ) {
        self.inner.initialize(uav_lat, uav_lon, bearing_rad, assumed_range_m);
    }

    /// IMM 交互 + 预测。
    fn predict(&mut self, dt: f64) {
        self.inner.predict(dt);
    }

    /// 方位角观测更新。
    fn update_bearing(&mut self, uav_lat: f64, uav_lon: f64, measured_bearing_rad: f64) {
        self.inner.update_bearing(uav_lat, uav_lon, measured_bearing_rad);
    }

    /// 距离观测更新。
    fn update_range(&mut self, uav_lat: f64, uav_lon: f64, measured_range_m: f64) {
        self.inner.update_range(uav_lat, uav_lon, measured_range_m);
    }

    /// 返回目标估计位置（WGS84）。
    fn position_wgs84(&self) -> (f64, f64) {
        self.inner.fused_position_wgs84()
    }

    /// 返回速度估计 (v_east, v_north) m/s。
    fn velocity_mps(&self) -> (f64, f64) {
        self.inner.fused_velocity_mps()
    }

    /// 返回速度大小 m/s。
    fn speed_mps(&self) -> f64 {
        self.inner.fused_speed_mps()
    }

    /// 返回位置不确定性。
    fn position_uncertainty_m(&self) -> f64 {
        self.inner.fused_position_uncertainty_m()
    }

    /// 返回模型概率 [CV, CA, CT]。
    fn model_probabilities(&self) -> Vec<f64> {
        self.inner.model_probabilities().to_vec()
    }

    /// 是否已初始化。
    fn is_initialized(&self) -> bool {
        self.inner.initialized
    }

    /// 是否已收敛。
    fn is_converged(&self, threshold_m: f64) -> bool {
        self.inner.is_converged(threshold_m)
    }
}

/// Rust 核心计算服务 — PyO3 模块入口。
#[pymodule]
fn rust_core(m: &Bound<'_, PyModule>) -> PyResult<()> {
    // geometry 函数
    m.add_function(wrap_pyfunction!(geometry::haversine_m, m)?)?;
    m.add_function(wrap_pyfunction!(geometry::bearing_rad, m)?)?;
    m.add_function(wrap_pyfunction!(geometry::destination_point, m)?)?;
    m.add_function(wrap_pyfunction!(geometry::wgs84_to_local, m)?)?;
    m.add_function(wrap_pyfunction!(geometry::local_to_wgs84, m)?)?;
    // EKF 类
    m.add_class::<ekf::BearingOnlyEKF>()?;
    // IMM 类
    m.add_class::<ImmFilter>()?;
    Ok(())
}

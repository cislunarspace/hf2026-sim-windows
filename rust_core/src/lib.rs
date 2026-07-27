use pyo3::prelude::*;

mod geometry;
mod ekf;

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
    Ok(())
}

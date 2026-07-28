//! geometry.rs — WGS84 几何计算。
//!
//! 提供 haversine 距离、方位角、目标点计算、WGS84↔局部切平面坐标转换。
//! 所有角度用弧度，距离用米。

use pyo3::prelude::*;

/// WGS84 球体半径（米）。
const EARTH_RADIUS_M: f64 = 6_371_000.0;

/// 经纬度→弧度。
#[inline]
fn to_rad(deg: f64) -> f64 {
    deg.to_radians()
}

// ── 基础几何 ────────────────────────────────────────────────────────────

/// 大圆距离（米），haversine 公式。
#[pyfunction]
pub fn haversine_m(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (rlat1, rlat2) = (to_rad(lat1), to_rad(lat2));
    let dlat = to_rad(lat2 - lat1);
    let dlon = to_rad(lon2 - lon1);
    let a = (dlat / 2.0).sin().powi(2)
        + rlat1.cos() * rlat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * a.sqrt().asin()
}

/// 从 (lat1,lon1) 到 (lat2,lon2) 的初始方位角（弧度），范围 [-π, π]。
#[pyfunction]
pub fn bearing_rad(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (rlat1, rlat2) = (to_rad(lat1), to_rad(lat2));
    let dlon = to_rad(lon2 - lon1);
    let y = dlon.sin() * rlat2.cos();
    let x = rlat1.cos() * rlat2.sin() - rlat1.sin() * rlat2.cos() * dlon.cos();
    y.atan2(x)
}

/// 从起点出发，沿给定方位角移动指定距离，返回目标点 (lat, lon)。
#[pyfunction]
pub fn destination_point(lat: f64, lon: f64, distance_m: f64, bearing_rad_arg: f64) -> (f64, f64) {
    let rlat = to_rad(lat);
    let rlon = to_rad(lon);
    let ang_dist = distance_m / EARTH_RADIUS_M;

    let sin_ad = ang_dist.sin();
    let cos_ad = ang_dist.cos();

    let dest_lat = (rlat.sin() * cos_ad + rlat.cos() * sin_ad * bearing_rad_arg.cos()).asin();
    let dest_lon = rlon
        + (bearing_rad_arg.sin() * sin_ad * rlat.cos())
            .atan2(cos_ad - rlat.sin() * dest_lat.sin());

    (dest_lat.to_degrees(), dest_lon.to_degrees())
}

// ── 坐标转换（EKF 用）──────────────────────────────────────────────────

/// WGS84 → 局部切平面（东-北坐标，米）。
///
/// 在 origin 处建立切平面，返回 (east_m, north_m)。
/// 适用于 origin 附近 < 10km 的区域，误差 < 0.1%。
#[pyfunction]
pub fn wgs84_to_local(lat: f64, lon: f64, origin_lat: f64, origin_lon: f64) -> (f64, f64) {
    let (rlat_o, rlon_o) = (to_rad(origin_lat), to_rad(origin_lon));
    let (rlat, rlon) = (to_rad(lat), to_rad(lon));

    let dlat = rlat - rlat_o;
    let dlon = rlon - rlon_o;

    // 东向：经度差 × cos(纬度) × 地球半径
    let east = dlon * rlat_o.cos() * EARTH_RADIUS_M;
    // 北向：纬度差 × 地球半径
    let north = dlat * EARTH_RADIUS_M;

    (east, north)
}

/// 局部切平面（东-北坐标，米） → WGS84。
#[pyfunction]
pub fn local_to_wgs84(east_m: f64, north_m: f64, origin_lat: f64, origin_lon: f64) -> (f64, f64) {
    let (rlat_o, rlon_o) = (to_rad(origin_lat), to_rad(origin_lon));

    let dlat = north_m / EARTH_RADIUS_M;
    let dlon = east_m / (rlat_o.cos() * EARTH_RADIUS_M);

    let lat = (rlat_o + dlat).to_degrees();
    let lon = (rlon_o + dlon).to_degrees();

    (lat, lon)
}

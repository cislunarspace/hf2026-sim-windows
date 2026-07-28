"""几何计算 Python 接口（薄壳）。

底层实现在 rust_core（PyO3 扁平模块），这里只做 import 转发，
使上层代码统一从 `algorithms.estimation.geometry` 导入。
"""

from rust_core import (
    bearing_rad,
    destination_point,
    haversine_m,
    local_to_wgs84,
    wgs84_to_local,
)

__all__ = [
    "bearing_rad",
    "destination_point",
    "haversine_m",
    "local_to_wgs84",
    "wgs84_to_local",
]

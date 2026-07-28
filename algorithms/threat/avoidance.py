"""SAM 规避策略（桩）。

待讨论：高度规避 vs 水平绕行 vs 混合（根据赛题确定）。
"""

from typing import Tuple


def compute_avoidance_vector(
    uav_lat: float,
    uav_lon: float,
    uav_alt: float,
    threat_lat: float,
    threat_lon: float,
    threat_radius_m: float,
) -> Tuple[float, float]:
    """计算规避方向向量（east, north 分量，单位米）。

    Args:
        uav_lat/lon/alt: UAV 当前位置
        threat_lat/lon: 威胁中心位置
        threat_radius_m: 威胁半径（米）

    Returns:
        (deast, dnorth): 规避方向的单位向量 × 安全距离
    """
    raise NotImplementedError("SAM 规避策略待实现，见 ADR-004 待定决策")

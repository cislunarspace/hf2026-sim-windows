"""飞行跟随策略：前馈飞行点计算。

根据目标的速度估计，计算 UAV 应飞向的前馈位置（lead point）。
"""

import math

from algorithms.estimation.geometry import destination_point


def compute_lead_point(
    target_lat: float,
    target_lon: float,
    target_v_east: float,
    target_v_north: float,
    lead_time_s: float,
) -> tuple[float, float]:
    """计算前馈飞行点（目标在 lead_time_s 后的预测位置）。

    使用恒速模型外推目标位置。

    Args:
        target_lat/lon: 目标当前位置（WGS84）
        target_v_east: 目标东向速度（m/s）
        target_v_north: 目标北向速度（m/s）
        lead_time_s: 前馈时间（秒）

    Returns:
        (lead_lat, lead_lon): 前馈点位置（WGS84）
    """
    if lead_time_s <= 0:
        return target_lat, target_lon

    # 速度大小和方向
    speed = math.sqrt(target_v_east**2 + target_v_north**2)
    if speed < 0.01:
        return target_lat, target_lon

    # 方位角：速度方向（北=0，东=π/2）
    bearing = math.atan2(target_v_east, target_v_north)

    # 前馈距离
    distance_m = speed * lead_time_s

    return destination_point(target_lat, target_lon, distance_m, bearing)

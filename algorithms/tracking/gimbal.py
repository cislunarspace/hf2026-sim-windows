"""云台控制：LOS（Line-of-Sight）角度计算 + FOV 管理。

根据 UAV 和目标的位置，计算云台的 pan（方位角）和 tilt（俯仰角）。
"""
import math

from algorithms.estimation.geometry import bearing_rad, haversine_m


def compute_gimbal_angles(
    uav_lat: float,
    uav_lon: float,
    uav_alt: float,
    target_lat: float,
    target_lon: float,
    target_alt: float = 0.0,
) -> tuple[float, float]:
    """计算云台 LOS 角度，使相机瞄准目标。

    Args:
        uav_lat/lon/alt: UAV 位置（WGS84，高度米）
        target_lat/lon/alt: 目标位置（WGS84，高度米）

    Returns:
        (pan_deg, tilt_deg):
        - pan_deg: 方位角（°），北=0，东=90，南=±180，西=-90
        - tilt_deg: 俯仰角（°），水平=0，向下看=-90
    """
    # pan：水平方位角
    pan_rad = bearing_rad(uav_lat, uav_lon, target_lat, target_lon)
    pan_deg = math.degrees(pan_rad)

    # tilt：俯仰角
    ground_range = haversine_m(uav_lat, uav_lon, target_lat, target_lon)
    alt_diff = uav_alt - target_alt  # 正值表示目标在下方

    if ground_range < 1e-3:
        # 目标在正下方
        tilt_deg = -90.0 if alt_diff > 0 else 0.0
    else:
        tilt_deg = -math.degrees(math.atan2(alt_diff, ground_range))

    return pan_deg, tilt_deg


def choose_fov(mode: str) -> float:
    """根据工作模式选择相机 FOV。

    Args:
        mode: "search"（搜索，宽 FOV）或 "track"（跟踪，窄 FOV）

    Returns:
        FOV 角度（度）
    """
    if mode == "search":
        return 70.0  # 宽 FOV，地面覆盖半径 ~210m（300m 高度）
    elif mode == "track":
        return 15.0  # 窄 FOV，跟踪精度高（配合 25mm 镜头）
    else:
        raise ValueError(f"未知模式: {mode}，应为 'search' 或 'track'")

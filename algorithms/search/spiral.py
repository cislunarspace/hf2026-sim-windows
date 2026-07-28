"""阿基米德螺旋搜索路径生成。

从给定中心点生成螺旋航点序列，用于区域搜索。
螺距（相邻圈间距）由相机 FOV 覆盖宽度决定。
"""

import math

from algorithms.estimation.geometry import destination_point

# 地球半径（米）
_EARTH_RADIUS_M = 6_371_000.0


def generate_spiral(
    center_lat: float,
    center_lon: float,
    radius_m: float,
    pitch_m: float,
    start_angle_rad: float = 0.0,
) -> list[tuple[float, float]]:
    """生成阿基米德螺旋航点序列。

    Args:
        center_lat: 螺旋中心纬度
        center_lon: 螺旋中心经度
        radius_m: 最大螺旋半径（米）
        pitch_m: 螺距——相邻两圈之间的距离（米）
        start_angle_rad: 起始角度（弧度），默认 0（正北方向）

    Returns:
        航点列表 [(lat, lon), ...]，从中心向外扩展。
    """
    if pitch_m <= 0:
        raise ValueError(f"pitch_m 必须 > 0，收到 {pitch_m}")
    if radius_m <= 0:
        raise ValueError(f"radius_m 必须 > 0，收到 {radius_m}")

    waypoints: list[tuple[float, float]] = []

    # 阿基米德螺旋：r = b * θ，b = pitch / (2π)
    b = pitch_m / (2.0 * math.pi)

    # 角度步长：每步前进约 pitch_m/10 的弧长（足够密集）
    # θ 从 0 到 θ_max，其中 r(θ_max) = radius_m → θ_max = radius_m / b
    theta_max = radius_m / b

    # 步长：保证每圈至少 20 个点（小半径时更多）
    step = max(0.05, pitch_m / (20.0 * max(b, 1.0)))
    # 但不要太大，避免航点过于稀疏
    step = min(step, 0.5)

    theta = start_angle_rad
    while theta <= theta_max + step:
        r = b * theta
        if r > radius_m:
            break

        # 极坐标 → 航点：沿 theta 方向移动 r 米
        bearing = theta  # 极角 → 方位角（北=0，东=π/2）
        lat, lon = destination_point(center_lat, center_lon, r, bearing)
        waypoints.append((lat, lon))

        theta += step

    return waypoints


def fov_ground_radius(alt_m: float, fov_deg: float) -> float:
    """计算给定高度和 FOV 角度下的地面覆盖半径（米）。

    用于确定螺旋搜索的螺距：pitch = 2 * fov_ground_radius * overlap_factor。

    Args:
        alt_m: UAV 飞行高度（米）
        fov_deg: 相机视场角（度）

    Returns:
        地面覆盖半径（米）
    """
    fov_rad = math.radians(fov_deg)
    return alt_m * math.tan(fov_rad / 2.0)

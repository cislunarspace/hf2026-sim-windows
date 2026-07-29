"""割草机（往返式）覆盖搜索航点生成。

螺旋搜索半径有限（~700m），覆盖不了 4.8km 图幅——真目标沿路网全域
机动时遭遇率过低（仿真实测：250s 内只遇到 5 m/s 慢目标，9/12 m/s
快目标从未进入搜索环）。割草机按车道间距全覆盖扇区。

车道沿纬度方向（长边），在经度方向步进；相邻车道飞行方向相反
（boustrophedon），航点只放在车道两端，固定翼在端点盘旋转向。
"""

_M_PER_DEG_LAT = 111320.0


def generate_lawnmower(
    lat_min: float,
    lat_max: float,
    lon_min: float,
    lon_max: float,
    lane_spacing_m: float = 400.0,
    margin_m: float = 200.0,
) -> list[tuple[float, float]]:
    """生成往返式覆盖航点（车道端点）。返回 [(lat, lon), ...]。

    lane_spacing_m：车道间距（米），按检测地面覆盖宽度选取。
    margin_m：距区域边界的内缩（米），避免越界扣分。
    """
    import math

    lat0 = (lat_min + lat_max) / 2
    m_per_deg_lon = _M_PER_DEG_LAT * math.cos(math.radians(lat0))
    dlat = margin_m / _M_PER_DEG_LAT
    dlon = margin_m / m_per_deg_lon
    la0, la1 = lat_min + dlat, lat_max - dlat
    lo0, lo1 = lon_min + dlon, lon_max - dlon

    n_lanes = max(1, round((lo1 - lo0) * m_per_deg_lon / lane_spacing_m) + 1)
    wps: list[tuple[float, float]] = []
    for i in range(n_lanes):
        lon = lo0 + (lo1 - lo0) * (i / max(n_lanes - 1, 1))
        if i % 2 == 0:
            wps.append((la0, lon))
            wps.append((la1, lon))
        else:
            wps.append((la1, lon))
            wps.append((la0, lon))
    return wps


def sector_lawnmower(
    uid: str,
    bbox: tuple[tuple[float, float], tuple[float, float]],
    n_sectors: int = 3,
    lane_spacing_m: float = 400.0,
) -> list[tuple[float, float]]:
    """按 uid 把 bbox 沿经度切成 n_sectors 条带，生成本机条带的割草机航点。

    起始端按条带奇偶错开，减少多机初始位置靠近时的 proximity 扣分。
    """
    (lat_min, lon_min), (lat_max, lon_max) = bbox
    if uid.isdigit():
        idx = int(uid) % n_sectors
    elif "_" in uid:
        tail = uid.rsplit("_", 1)[-1]
        idx = int(tail) % n_sectors if tail.isdigit() else 0
    else:
        idx = 0
    sub_w = (lon_max - lon_min) / n_sectors
    wps = generate_lawnmower(
        lat_min, lat_max,
        lon_min + idx * sub_w, lon_min + (idx + 1) * sub_w,
        lane_spacing_m=lane_spacing_m,
    )
    if idx % 2 == 1:
        wps = wps[::-1]
    return wps

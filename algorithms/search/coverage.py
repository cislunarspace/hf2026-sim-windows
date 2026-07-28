"""Voronoi 搜索覆盖分区。

根据 UAV 位置动态划分搜索区域，每个 UAV 负责离它最近的区域。
不依赖 scipy，用网格采样法实现。

典型用法：
    partitions = voronoi_partition(
        seeds=[(27.0, 124.99), (27.0, 125.0), (27.0, 125.01)],
        bbox=((26.982, 124.980), (27.025, 125.020)),
    )
    # partitions[0]["center"] → 该 UAV 搜索中心
    # partitions[0]["radius_m"] → 建议搜索半径
"""
import math
from typing import Dict, List, Tuple

_EARTH_R = 6_371_000.0


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """两点间大圆距离（米）。"""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * _EARTH_R * math.asin(math.sqrt(a))


def _latlon_to_local(lat: float, lon: float, ref_lat: float, ref_lon: float):
    """WGS84 → 局部 ENU（米），用于距离计算。"""
    dn = (lat - ref_lat) * _EARTH_R * math.pi / 180
    de = (lon - ref_lon) * _EARTH_R * math.cos(math.radians(ref_lat)) * math.pi / 180
    return de, dn


def nearest_seed(
    point: Tuple[float, float],
    seeds: List[Tuple[float, float]],
) -> int:
    """返回离 point 最近的 seed 索引（欧氏距离，局部坐标）。"""
    lat, lon = point
    ref_lat = sum(s[0] for s in seeds) / len(seeds)
    ref_lon = sum(s[1] for s in seeds) / len(seeds)
    de, dn = _latlon_to_local(lat, lon, ref_lat, ref_lon)

    best_i, best_d2 = 0, float("inf")
    for i, (slat, slon) in enumerate(seeds):
        sde, sdn = _latlon_to_local(slat, slon, ref_lat, ref_lon)
        d2 = (de - sde) ** 2 + (dn - sdn) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def voronoi_partition(
    seeds: List[Tuple[float, float]],
    bbox: Tuple[Tuple[float, float], Tuple[float, float]],
    grid_n: int = 50,
) -> List[Dict]:
    """Voronoi 分区：根据 UAV 位置划分搜索区域。

    Args:
        seeds: UAV 位置列表 [(lat, lon), ...]
        bbox: 任务区域 ((lat_min, lon_min), (lat_max, lon_max))
        grid_n: 网格分辨率（grid_n × grid_n）

    Returns:
        每个 UAV 的分区信息列表：
        [{"center": (lat, lon), "radius_m": float, "area_km2": float}, ...]
    """
    n = len(seeds)
    if n == 0:
        return []
    if n == 1:
        (lat_min, lon_min), (lat_max, lon_max) = bbox
        center = ((lat_min + lat_max) / 2, (lon_min + lon_max) / 2)
        lat_km = _haversine_m(lat_min, lon_min, lat_max, lon_min) / 1000
        lon_km = _haversine_m(lat_min, lon_min, lat_min, lon_max) / 1000
        area_km2 = lat_km * lon_km
        radius_m = math.sqrt(area_km2 * 1e6 / math.pi)
        return [{"center": center, "radius_m": radius_m, "area_km2": area_km2}]

    (lat_min, lon_min), (lat_max, lon_max) = bbox
    ref_lat = (lat_min + lat_max) / 2
    ref_lon = (lon_min + lon_max) / 2

    # 生成网格点并分配
    cell_lat = (lat_max - lat_min) / grid_n
    cell_lon = (lon_max - lon_min) / grid_n
    # 每个 UAV 的网格点集合：(sum_de, sum_dn, count)
    accum = [(0.0, 0.0, 0) for _ in range(n)]

    for i in range(grid_n):
        pt_lat = lat_min + (i + 0.5) * cell_lat
        for j in range(grid_n):
            pt_lon = lon_min + (j + 0.5) * cell_lon
            idx = nearest_seed((pt_lat, pt_lon), seeds)
            de, dn = _latlon_to_local(pt_lat, pt_lon, ref_lat, ref_lon)
            sde, sdn, cnt = accum[idx]
            accum[idx] = (sde + de, sdn + dn, cnt + 1)

    # 计算每个分区的质心和面积
    total_cells = grid_n * grid_n
    # bbox 面积（km²）
    lat_km = _haversine_m(lat_min, lon_min, lat_max, lon_min) / 1000
    lon_km = _haversine_m(lat_min, lon_min, lat_min, lon_max) / 1000
    bbox_area_km2 = lat_km * lon_km

    results = []
    for i in range(n):
        sde, sdn, cnt = accum[i]
        if cnt > 0:
            avg_de = sde / cnt
            avg_dn = sdn / cnt
            # 局部坐标 → WGS84
            center_lat = ref_lat + avg_dn / (_EARTH_R * math.pi / 180)
            center_lon = ref_lon + avg_de / (_EARTH_R * math.cos(math.radians(ref_lat)) * math.pi / 180)
        else:
            center_lat, center_lon = seeds[i]

        area_km2 = bbox_area_km2 * cnt / total_cells
        # 等效圆半径
        radius_m = math.sqrt(area_km2 * 1e6 / math.pi) if area_km2 > 0 else 100.0

        results.append({
            "center": (center_lat, center_lon),
            "radius_m": radius_m,
            "area_km2": area_km2,
        })

    return results

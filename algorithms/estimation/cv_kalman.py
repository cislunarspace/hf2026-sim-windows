"""CV 卡尔曼滤波（位置量测）——赛题一目标跟踪用。

相机检测直接给出目标经纬度（±50m 高斯噪声），是完整位置量测，
不需要 bearing-only IMM。这里用两个解耦的 1D 匀速卡尔曼（东/北向
各一），纯 Python、数值稳定；坐标用等距圆柱近似（区域 <5km）。

ImmFilter（bearing+range）在此场景实测发散：估计值在两帧间跳离
新鲜量测 ~200m（st_debug_240 局 t=65.8），导致云台指外、跟踪中断。
位置量测下的 CV 卡尔曼是对这个问题的最小正确工具。
"""

import math

_EARTH_RADIUS_M = 6_378_137.0


def _wgs84_to_local(lat: float, lon: float, olat: float, olon: float):
    east = math.radians(lon - olon) * math.cos(math.radians(olat)) * _EARTH_RADIUS_M
    north = math.radians(lat - olat) * _EARTH_RADIUS_M
    return east, north


def _local_to_wgs84(east: float, north: float, olat: float, olon: float):
    dlat = north / _EARTH_RADIUS_M
    dlon = east / (math.cos(math.radians(olat)) * _EARTH_RADIUS_M)
    return olat + math.degrees(dlat), olon + math.degrees(dlon)


class _CvAxis:
    """单轴匀速卡尔曼：状态 [p, v]，量测 p，白噪声加速度模型。"""

    def __init__(self, q: float, r: float):
        self.q = q  # 加速度谱密度 (m/s²)²/s
        self.r = r  # 量测噪声方差 (m²)
        self.p = 0.0
        self.v = 0.0
        self.p00 = 1e6
        self.p01 = 0.0
        self.p11 = 1e4
        self.ready = False

    def initialize(self, z: float, v: float = 0.0) -> None:
        self.p = z
        self.v = v
        self.p00 = self.r
        self.p01 = 0.0
        # 速度初值方差取 (5 m/s)²：初值 0 与真实 8 m/s 的差距靠量测拉起，
        # 1e4 会让速度估计头几秒过冲到 20+ m/s（扫参实测），25 则无过冲
        self.p11 = 25.0
        self.ready = True

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        self.p += self.v * dt
        dt2 = dt * dt
        q00 = self.q * dt2 * dt2 / 4.0
        q01 = self.q * dt2 * dt / 2.0
        q11 = self.q * dt2
        p00, p01, p11 = self.p00, self.p01, self.p11
        self.p00 = p00 + 2.0 * dt * p01 + dt2 * p11 + q00
        self.p01 = p01 + dt * p11 + q01
        self.p11 = p11 + q11

    def update(self, z: float) -> None:
        y = z - self.p
        s = self.p00 + self.r
        k0 = self.p00 / s
        k1 = self.p01 / s
        self.p += k0 * y
        self.v += k1 * y
        p00, p01, p11 = self.p00, self.p01, self.p11
        self.p00 = (1.0 - k0) * p00
        self.p01 = (1.0 - k0) * p01
        self.p11 = p11 - k1 * p01


class CvFilter:
    """双轴（东/北）匀速卡尔曼，WGS84 经纬度接口。"""

    def __init__(self, origin_lat: float, origin_lon: float,
                 q: float = 4.0, noise_m: float = 50.0):
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        r = noise_m * noise_m
        self._e = _CvAxis(q, r)
        self._n = _CvAxis(q, r)

    def is_initialized(self) -> bool:
        return self._e.ready

    def initialize(self, lat: float, lon: float,
                   ve: float = 0.0, vn: float = 0.0) -> None:
        """初始化位置；有路线先验时可带入先验速度（消除斜坡滞后）。"""
        e, n = _wgs84_to_local(lat, lon, self.origin_lat, self.origin_lon)
        self._e.initialize(e, ve)
        self._n.initialize(n, vn)

    def predict(self, dt: float) -> None:
        self._e.predict(dt)
        self._n.predict(dt)

    def update_position(self, lat: float, lon: float) -> None:
        e, n = _wgs84_to_local(lat, lon, self.origin_lat, self.origin_lon)
        self._e.update(e)
        self._n.update(n)

    def position_wgs84(self) -> tuple[float, float]:
        return _local_to_wgs84(self._e.p, self._n.p,
                               self.origin_lat, self.origin_lon)

    def velocity_mps(self) -> tuple[float, float]:
        """(ve, vn)：东向、北向速度。"""
        return self._e.v, self._n.v

    def speed_mps(self) -> float:
        return math.hypot(self._e.v, self._n.v)

    def position_std_m(self) -> float:
        """位置标准差（取两轴较大者），用于收敛判定。"""
        return math.sqrt(max(self._e.p00, self._n.p00))

    def is_converged(self, std_m: float) -> bool:
        return self.is_initialized() and self.position_std_m() < std_m

"""运动学判别：最小二乘速度估计。

用于 VERIFY 阶段的诱饵判别。检测位置含 ~50m 逐帧高斯噪声时，
IMM 速度估计会把静止诱饵放大到 3~8 m/s（阈值失效）；iid 噪声下
最小二乘直线拟合是统计上更合适的估计：12s 窗口（120 帧 @10Hz）
速度标准差 ~1.3 m/s，阈值 3.5 m/s 时静止诱饵误判率 ~3%、
5 m/s 真目标召回 ~90%（离线仿真验证）。
"""

import math

_M_PER_DEG_LAT = 111320.0


def ols_speed_mps(samples: list[tuple[float, float, float]]) -> float:
    """对 [(t, lat, lon), ...] 样本做最小二乘拟合，返回速度（m/s）。

    局部平面近似：以首样本为原点，纬度差×111320、经度差×111320·cos(lat)
    换算成米，分别对北向/东向拟合斜率，取合成速率。
    """
    n = len(samples)
    if n < 2:
        return 0.0
    t0, lat0, lon0 = samples[0]
    cos0 = math.cos(math.radians(lat0))
    xs = [t - t0 for t, _, _ in samples]
    north = [(la - lat0) * _M_PER_DEG_LAT for _, la, _ in samples]
    east = [(lo - lon0) * _M_PER_DEG_LAT * cos0 for _, _, lo in samples]

    mx = sum(xs) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    if sxx <= 0.0:
        return 0.0

    def _slope(ys: list[float]) -> float:
        my = sum(ys) / n
        return sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx

    vn = _slope(north)
    ve = _slope(east)
    return math.hypot(vn, ve)

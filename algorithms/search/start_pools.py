"""Start 池烘焙（离线由 config 生成，勿手改）。

判别辅助：真目标路线池（config/points.json，26 条 road*）与诱饵路线池
（config/random_routes_20.json，18 条 route_*）物理分离、Start 零重叠。
开局检测位置落在哪个池的 Start 附近，是合法的判别信号（代码内置先验，
decide 不读文件）。验证阶段真目标换新路线（手册 §5）时真池失效，本模块
只作辅助：判"诱"才否决（有冷却可恢复），判"真倾向"只提高置信不硬判。
"""

# 真目标 Start（points.json 全部 26 条路线的 Start）
TARGET_STARTS: list[tuple[float, float]] = [
    (27.001090, 125.000860),
    (27.012220, 125.001340),
    (26.996979, 124.986701),
    (26.996007, 125.010451),
    (27.012465, 124.991424),
    (27.019100, 125.001860),
    (27.007340, 125.003140),
    (27.016410, 124.996500),
    (27.009560, 124.997270),
    (27.015104, 124.999201),
    (27.014479, 124.991493),
    (27.012396, 124.995868),
    (27.004410, 124.995104),
    (27.008576, 124.988229),
    (27.004201, 124.992396),
    (27.001562, 124.995660),
    (26.999479, 124.997535),
    (26.996146, 125.010451),
    (27.009410, 124.994480),
    (27.009410, 124.993161),
    (27.009340, 124.991354),
    (27.012396, 124.991563),
    (27.008715, 124.996840),
    (27.007674, 125.005521),
    (27.005382, 124.996979),
    (27.008854, 124.996910),
]

# 诱饵 Start（random_routes_20.json 全部 18 条路线的 Start）
DECOY_STARTS: list[tuple[float, float]] = [
    (27.019132, 124.983090),
    (27.006979, 125.018090),
    (27.012743, 124.999687),
    (26.993646, 124.983160),
    (27.010313, 124.990729),
    (26.988576, 125.006979),
    (27.003438, 124.982465),
    (27.004062, 125.014896),
    (27.005938, 125.011007),
    (27.000660, 124.997882),
    (27.007465, 124.997049),
    (27.004271, 124.992535),
    (27.012326, 125.007674),
    (26.995104, 124.987882),
    (26.998924, 124.985243),
    (27.021354, 124.992535),
    (26.996840, 125.006632),
    (27.016424, 124.996632),
]

# Start 池匹配阈值（量化验证 B：5~35s 均值位置距本池 Start 54~100m、
# 跨池混淆发生在两池距离 <130m 的邻近区；150/250m 双阈值留噪声余量）
_START_MATCH_M = 150.0    # 属于某池的判定距离
_START_OTHER_M = 250.0    # 判定"不属于另一池"的距离


def nearest_start_m(lat: float, lon: float, pool: list[tuple[float, float]]) -> float:
    """到池内最近 Start 的距离（m）。"""
    from algorithms.estimation.geometry import haversine_m

    best = float("inf")
    for sla, slo in pool:
        d = haversine_m(lat, lon, sla, slo)
        if d < best:
            best = d
    return best


def _uid_share(uid: str, n_shares: int) -> int:
    """uid → 分片序号：优先取尾部数字（"20001" 或 "uav_1"）对 n_shares
    取模，与 route_waypoints_for_uid 的 isdigit 风格一致；无数字后缀时按
    字符和稳定散列（uav_alpha/bravo/charlie 类 uid 也能分散到不同分片）。"""
    tail = uid.rsplit("_", 1)[-1]
    if tail.isdigit():
        return int(tail) % n_shares
    return sum(ord(c) for c in uid) % n_shares


def coverage_waypoints_for_uid(uid: str, n_shares: int = 3) -> list[tuple[float, float]]:
    """区域覆盖航点：TARGET_STARTS(26) + DECOY_STARTS(18) 共 44 点混合，
    按 uid 数字后缀取模 n_shares 分片（步长 n_shares 取点，首点按 uid 序号
    偏移错开），三架 UAV 覆盖不相交、合取为全集。验证集换路线后新 Start
    仍在 A* 路网节点附近（环境要素不变=路网不变），44 点覆盖路网关键交汇处。"""
    idx = _uid_share(uid, n_shares)
    points = TARGET_STARTS + DECOY_STARTS
    return points[idx::n_shares]


def match_start_pool(lat: float, lon: float) -> str:
    """返回 'decoy' | 'true' | 'suspect'。

    检测位置（开局头 ~40s 内有效，目标从 Start 出发后仍在其附近）：
      - 距某诱饵 Start <150m 且距最近真 Start >250m → 'decoy'（否决信号）
      - 距某真 Start   <150m 且距最近诱 Start >250m → 'true'（真倾向）
      - 其余 → 'suspect'（两池都近/都不近，交速度与二次验证定夺）
    """
    d_decoy = nearest_start_m(lat, lon, DECOY_STARTS)
    d_true = nearest_start_m(lat, lon, TARGET_STARTS)
    if d_decoy < _START_MATCH_M and d_true > _START_OTHER_M:
        return "decoy"
    if d_true < _START_MATCH_M and d_decoy > _START_OTHER_M:
        return "true"
    return "suspect"

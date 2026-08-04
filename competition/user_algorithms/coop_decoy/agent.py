"""赛题二协同诱饵鉴别 Agent。

四状态 FSM：
  SEARCH → VERIFY → TRACK
                ↑       ↓ (摧毁/超时) → SEARCH
  JOIN ←── (收到队友广播确认目标)

算法：
  - OLS 最小二乘速度判别诱饵（12s 采样窗口，抗 ~50m 检测噪声；
    CvFilter 同步更新供 TRACK 接管与上报）
  - 螺旋搜索（uid 扇区分配）
  - 盘旋跟踪 + 广播协同
  - K=2 同时盯防 20s 摧毁
"""

import math
import os
from collections import deque
from enum import Enum

_DEBUG = os.environ.get("COOP_AGENT_DEBUG") == "1"

# ── 路线匹配器：用 26 条真目标路线区分真目标/诱饵 ─────────────────────
# 真目标路线（points.json）有 7-28 个中间航点，形成复杂折线；
# 诱饵路线（random_routes_20.json）只有 Start→End，无中间航点。
# 通过点到折线距离判断观测轨迹是否匹配已知真目标路线。
from algorithms.search.route_prior import ROUTES as _TRUE_ROUTES


def _pt_to_polyline_dist_m(
    lat: float, lon: float, route: list[tuple[float, float, float]]
) -> float:
    """点到折线的最小距离（m）。route 为 (lat, lon, wait_s) 列表。"""
    best = float("inf")
    for i in range(len(route) - 1):
        a_lat, a_lon = route[i][0], route[i][1]
        b_lat, b_lon = route[i + 1][0], route[i + 1][1]
        # 投影到线段 AB，限制在 [0,1]
        ab_lat = b_lat - a_lat
        ab_lon = b_lon - a_lon
        ap_lat = lat - a_lat
        ap_lon = lon - a_lon
        ab2 = ab_lat * ab_lat + ab_lon * ab_lon
        if ab2 < 1e-18:
            d = haversine_m(lat, lon, a_lat, a_lon)
        else:
            t = (ap_lat * ab_lat + ap_lon * ab_lon) / ab2
            t = max(0.0, min(1.0, t))
            proj_lat = a_lat + t * ab_lat
            proj_lon = a_lon + t * ab_lon
            d = haversine_m(lat, lon, proj_lat, proj_lon)
        if d < best:
            best = d
    return best


class _RouteMatcher:
    """滑动窗口轨迹匹配器：累积观测位置，计算与 26 条真目标路线的匹配分。

    match_score() 返回最优路线的平均点到折线距离（m）。
    <100m → 强匹配（真目标）；>200m → 无匹配（诱饵）。
    """

    def __init__(self, window_s: float = 15.0, min_samples: int = 5):
        self._window_s = window_s
        self._min_samples = min_samples
        self._obs: deque[tuple[float, float, float]] = deque()  # (t, lat, lon)

    def append(self, t: float, lat: float, lon: float) -> None:
        self._obs.append((t, lat, lon))

    def _prune(self, now: float) -> None:
        while self._obs and now - self._obs[0][0] > self._window_s:
            self._obs.popleft()

    def match_score(self, now: float) -> float | None:
        """返回最优路线的平均点到折线距离（m），样本不足返回 None。"""
        self._prune(now)
        if len(self._obs) < self._min_samples:
            return None
        best_mean = float("inf")
        for route in _TRUE_ROUTES:
            total = 0.0
            for _, lat, lon in self._obs:
                total += _pt_to_polyline_dist_m(lat, lon, route)
            mean_d = total / len(self._obs)
            if mean_d < best_mean:
                best_mean = mean_d
        return best_mean

    def has_displacement(self, threshold_m: float = 100.0) -> bool:
        """观测窗口内是否有足够位移（m）。用于区分"先动后停"和"从头静止"。"""
        if len(self._obs) < 2:
            return False
        first = self._obs[0]
        max_d = 0.0
        for _, lat, lon in self._obs:
            d = haversine_m(first[1], first[2], lat, lon)
            if d > max_d:
                max_d = d
        return max_d >= threshold_m

    def reset(self) -> None:
        self._obs.clear()

# 逐拍调试探针：decide() 末尾写入本机 FSM 状态，stand 版评测探针逐拍落盘，
# 用于还原"配对/进场/协锁"时序（评测器侧无此信息）。默认空 dict，无性能影响。
AGENT_DEBUG_STATE: dict[str, dict] = {}

from algorithms.estimation.cv_kalman import CvFilter
from algorithms.estimation.geometry import bearing_rad, haversine_m
from algorithms.estimation.motion import ols_speed_mps
from algorithms.search.start_pools import coverage_waypoints_for_uid
from algorithms.tracking.gimbal import compute_gimbal_angles

from competition.sdk.core.commands import (
    Command,
    broadcast,
    fly_to,
    point_gimbal,
    report_target,
    set_gimbal_fov,
)
from competition.sdk.scenarios.coop_decoy import CoopAgent
from competition.sdk.scenarios.coop_decoy.observation import CoopObs

# ── 常量 ──────────────────────────────────────────────────────────────────

_SEARCH_ALT = 500.0  # 搜索高度（m）：飞高拉宽相机地面脚印（~570m vs 200m 的 ~230m），
# 配合 40 m/s 扫描效率 ~4 倍于 200m/25m/s（赛题二运动学上限 40 m/s、无高度锁）
_SEARCH_SPEED = 40.0  # 搜索速度（m/s，运动学上限）
_TRACK_SPEED = 15.0  # 跟踪速度（m/s）：慢盘 LOS 转速更低，目标更久留在视场内
_TRACK_LOITER_M = 250.0  # 长机跟踪盘旋半径（m）：100m 圈 LOS 转速快、检出率
# 仅 ~40%；250m 圈转速慢一半以上，配合 120° FOV 让目标持续在视场内
_WING_LOITER_M = 600.0  # 僚机盘旋半径（m）：比 500 大 100m，更远更稳
# （LOS 转速更低、目标久留视场）；与长机圈最近距离 350m > 200m
# proximity 罚线；700m 斜距 + 120° FOV 地面脚印仍 >1km
_JOIN_LOITER_M = 500.0  # JOIN 收敛盘旋半径（m）：与僚机 TRACK 圈一致，入圈
# 后不改半径；与长机 250m 圈错开 350m，不触发 proximity 扣分
_JOIN_TRACK_THRESHOLD_M = 750.0  # JOIN→TRACK 转移阈值（m）：> 650m 僚机圈，
# 留 100m 余量确保在圈上检测到即可转 TRACK
_LEAD_TIME_S = 1.5  # 前馈时间（s）

# ── v9 编队打击：三机编队搜索 + 在线任务分配 ─────────────────────────
# 战术：三机横排编队快速扫描（僚机跟随编队长 20001，横向偏移 400m），
# 发现判真目标 → 两台打击（距离近的响应）、一台保持搜索 → 打击完成
# （D: 冻结确认）→ 双机回收跟随编队长恢复队形。全部在线决策（距离比较）。
_LEADER_UID = "20001"  # 固定编队长：沿覆盖航点导航，僚机跟随
_WING_OFFSET_M = 400.0  # 僚机相对编队长的横向偏移（m）：>200m proximity 罚线，
# 且编队覆盖条带宽 800m；TRACK 圈 250/500m 最近 250m 也 >200m
_STRIKE_DECISION_MARGIN_M = 100.0  # 打击响应距离决策余量（m）：队友近 100m+ 则让位
_MISSION_LAT_MIN, _MISSION_LAT_MAX = 26.98, 27.02  # 任务区（build_briefing 同源）
_MISSION_LON_MIN, _MISSION_LON_MAX = 124.98, 125.02
_BOUNDARY_MARGIN_M = 200.0  # 任务区边界内收余量（m）：boundary 罚分在出界 >500m 才扣，
# 但僚机偏移把目标点拉出边界会让 UAV 持续贴边飞行，内收 200m 留安全余量

_VERIFY_WINDOW_S = 20.0  # OLS 判别时间窗口（s）。实测 decide 10Hz、检出率
# ~35%，20s 窗口约 70 样本。蒙特卡洛（70m 噪声）：12 m/s 入带[7,14.5]
# 94.7%、9 m/s 92.8%、5 m/s 仅 10.9%。按"时间跨度满 20s"判别，不要求
# 固定样本数——稀疏检出（间隙）不损害 OLS（按时间戳回归）。
_JUMP_THRESHOLD_M = 250.0  # 相邻检测突变阈值（m）：锁跳变/错误关联防护。
# 70m 噪声下相邻样本位移 ~N(0,99)（两独立噪声差），>250m ≈ 0.6%；
# 锁跳变（来回跳 400m）每跳必触发
_JUMP_MIN = 2  # 一个判别窗口内突变次数 ≥2 判"跳变"→ 否决（非连续移动）
# 判别带 [7.0, 14.5]（蒙特卡洛标定：70m 噪声 + 20s 窗口）：
#   ≥9.0：12 m/s 覆盖 98%、5 m/s 漂移仅 0.4% → 单窗口快判；
#   [7.0, 9.0)：9 m/s 主区间 + 5 m/s 漂移 10.5%，二次独立窗口确认
#   （误报 10.5%² ≈ 1.1%）。
_VERIFY_SPEED_MIN = 7.0  # OLS 速度下限（m/s）：≥7.0 必真（真目标 9/12，诱饵 5.0）
_VERIFY_SPEED_MAX = 14.5  # OLS 速度上限（m/s）：超过地面车辆极速（12）+ 噪声余量
_VERIFY_FAST_MIN = 9.0  # OLS 单窗口快判下限：≥9.0 立即 TRACK（5 m/s 漂移 0.4%）
_VERIFY_PASSES = 2  # [7.0, 9.0) 区间二次独立窗口确认（5 m/s 误报 ~1.1%）
_VERIFY_LOST_ABORT_S = 6.0  # VERIFY 中连续丢失超过此时长则放弃（不记诱饵）。
# 10Hz decide、检出率 ~35%（间隙均值 ~0.3s）；6s 容忍 FOV 遮挡又不卡死
_FAST_PASS_S = 3.0  # 快速通过：滤波速度持续落在速度带的时长（s）。
_FAST_PASS_MIN = 7.0  # fast-pass 速度下限：9/12 m/s 档确定性速度；
# 5 m/s 漂移 >7.0 仅 ~8%（6.5 下限会让 5 m/s 诱饵 15.7% 直接快通）
_WING_BAIL_S = 15.0  # 静止 bailout 时长（s）：CvFilter 速度 <4.0 持续 15s 退出。
# 阈值 4.0（原 5.5）：5 m/s 档真目标与诱饵同速，5.5 会误杀真目标——
# 4.0 以下才是静止/异常（5 m/s 档不会触发）
_WING_BAIL_SPEED = 4.0  # 静止 bailout 速度阈值（m/s）
_SLOW_CHECK_SPEED = 5.5  # 低速核查阈值（m/s）：5 m/s 档速度不可分，
# 低于它持续 10s 即进入验证模式（_is_probe=True），由引擎行为判别真伪
_SLOW_CHECK_S = 10.0  # 低速核查时长（s）：CvFilter <5.5 持续 10s 后置
# _is_probe=True 继续盯（不再查 Start 池退出）——9/12 档误判进来的目标
# 由 v≥5.5 恢复 _is_probe=False，5 档真伪交给 dwell 满 20s 后的验证尾段
_PROBE_SPEED_MIN = 2.0  # VERIFY 出带低速进 TRACK 验证模式的下限（m/s）：
# OLS 速度 [2.0, 7.0) 是 5 m/s 档（真/诱同速不可分），不再查 Start 池、
# 不 reject，直接进 TRACK 由引擎行为判别（盯满 20s 后冻结=真、继续移动=诱）
_PROBE_STOP_SPEED = 1.5  # 验证尾段"冻结"阈值（m/s）：真目标被 ≥2 架盯满
# 20s 后引擎冻结不再移动，滤波速度骤降至 <1.5
_PROBE_CHECK_S = 5.0  # 验证尾段判定时长（s）：滤波速度连续 5s <1.5 → 真目标
# 已冻结摧毁；连续 5s ≥1.5 → 诱饵（identified）记冷却
_PROBE_LOST_ABORT_S = 10.0  # probe 验证模式检测全丢放弃阈值（s）：dwell 无法
# 累计、尾段无法读数时直接回 SEARCH（不记否决，5s 平冷却防死循环）
_REJECT_COOLDOWN_S = 20.0  # 判别否决/中止后的重检测冷却（s）：防同帧循环重进 VERIFY，
# 又不永久标记——停顿中的真目标冷却后重遇可重新判别
_REJECT_RADIUS_M = 500.0  # 冷却生效的检测距离（m）：机 20s 已飞出 ~500m，
# 真目标 20s 后仍在原区附近可重新判别；跳变搭档车（≥198m）也在半径内
_ABORT_COOLDOWN_S = 5.0  # VERIFY 接触丢失中止的平冷却（s）：不构成判别结论，
# 只要防当拍重进空转；用判别否决的升档冷却会在密集车场形成死亡螺旋
# （debug3 局实测：首次中止后 rcd 恒真，240s 零 VERIFY、全场锁死）
_ABORT_RADIUS_M = 300.0  # 中止冷却的生效半径（m）：只挡同一辆车

_TRACK_DWELL_S = 20.0  # 盯防摧毁时间（s）
_TRACK_GRACE_S = 2.0  # 丢失容忍时间（s）
_TRACK_TIMEOUT_S = 90.0  # 跟踪超时（s）：长机须咬住目标等僚机
# （跨区飞来 ~50s + 僚机 VERIFY 12s + 协同 20s），35s 等到一半就放弃了
_TRACK_TIMEOUT_PAIRED_S = 180.0  # 协锁中的跟踪超时（s）：双机已配对时 90s
# 超时会拆对、把评测器正在累计的 coop 清零（v03 诊断：leader/wingman 各自
# 90s 退出，10003 攒到 124 coop ticks 被清零）——配对中给足时间等评测器
# 的 20s 连续双锁，真伪由验证尾段/冻结确认裁决，超时只兜底病理情况
_ORPHAN_PROMOTE_S = 5.0  # 孤儿僚机晋升阈值（s）：leader 的 A:/T: claim 消失
# 超过此时长即晋升为长机并重新 announce——孤儿只发 J: 占位，其他 UAV 不会
# 据此 JOIN，它盯着的目标（可能是真目标）永远凑不齐第二架（v06 诊断：
# 20002 对 5 m/s 真目标 99.8% 连续有效却全程无人加入）
_CONFIRMED_REANNOUNCE_S = 10.0  # 已判真长机的 announce 重发间隔（s）：只发
# 一次的 announce 会在 40s 后过期，晚到/刚空闲的 UAV 就不知道有确定目标；
# 周期性重发让 9/12 目标优先被配对（probe 的 5 m/s 候选不重发，避免抢资源）

_JOIN_TIMEOUT_S = 60.0  # JOIN 超时（s），扇区间距 ~2km @25m/s 需 ~80s 收敛
_ANNOUNCE_EXPIRE_S = 40.0  # announce 过期时间（s）。僚机 40 m/s 飞行，
# 15s 只能飞 600m——远机（>600m）收到 announce 后在旧窗口内到不了目标
# 就放弃（debug6 局实测：长机 TRACK 60s 无僚机加入，三机全在单打独斗）。
# 40s × 40 m/s = 1.6km 覆盖大部分扇区间距；JOIN 超时 60s 兜底

_BC_INTERVAL = 0.5  # 广播间隔（s，2Hz）
_HB_INTERVAL = 1.0  # 位置心跳间隔（s，1Hz；P:lat,lon，用于 proximity 避让）
_REPORT_INTERVAL = 1.0  # 上报间隔（s）

_ASSUME_RANGE_M = 800.0  # 首次检测假设距离（m）
_VERIFY_RADIUS = 450.0  # VERIFY 盘旋半径（m）：保持距离判别（目标稳定在
# FOV，检测连续）；450m 圈 >200m 罚线，判真后 TRACK 再接近
_TRACK_FOV = 30.0  # 跟踪 FOV（°）：60° 宽锥在真实引擎下 TRACK 阶段锁诱饵并上报
# （实测 RMSE 41→484m、n_reports 11→82）；30° 窄锥排除大部分相邻诱饵，
# 检测连续性由 CvFilter 滤波 + 宽锥 VERIFY 的样本积累保障
_WING_FOV = 30.0  # 僚机 FOV（°）：与 leader 一致，排除相邻诱饵
_SEARCH_FOV = 60.0  # 搜索 FOV（°）

# 赛题二场景 bbox（北京附近海域）
_BBOX: tuple[tuple[float, float], tuple[float, float]] = (
    (26.982, 124.980),
    (27.025, 125.020),
)


class State(Enum):
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    JOIN = "JOIN"


class CoopDecoyAgent(CoopAgent):
    """赛题二参赛 Agent：CvFilter 滤波 + 路线先验搜索 + 盘旋跟踪 + 广播协同。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state = State.SEARCH
        self._filter: CvFilter | None = None
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s: float = 0.0
        self._verify_pass_count: int = 0  # OLS 连续入带窗口数（二次验证）
        self._jump_count: int = 0  # 当前窗口内检测突变次数（锁跳变防护）
        self._route_matcher = _RouteMatcher(window_s=15.0, min_samples=5)
        self._fast_pass_s: float = 0.0  # 快速通过累计时长
        self._wing_bail_s: float = 0.0  # 静止 bailout 累计时长
        self._slow_check_s: float = 0.0  # 低速核查累计时长（5 m/s 档）
        self._is_probe: bool = False  # True=验证式跟踪（5 m/s 档候选，
        # 真伪由盯满 20s 后的引擎行为——冻结/继续移动——判别）
        self._probe_check_s: float = 0.0  # 验证尾段"冻结"证据累计时长（引擎轴）
        self._probe_moving_s: float = 0.0  # 验证尾段"仍在移动"证据累计时长（引擎轴）
        self._confirmed_real: bool = False  # True=OLS 入带判真（9/12 档），
        # 只有这类目标才上报——报诱饵位置会匹配到最近存活真目标、打爆 RMSE
        self._orphan_s: float = 0.0  # 孤儿状态累计时长（leader claim 缺失）
        self._promoted: bool = False  # 本拍晋升长机，广播 A: announce
        self._leader_pos: tuple[float, float] | None = None  # leader 最新报告位置
        self._leader_time: float = -1e9  # leader 报告接收时刻
        self._last_announce_time: float = -1e9  # 上次 A: announce 时刻
        self._sim_time = 0.0
        self._target: tuple[float, float] | None = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_destroyed: list[tuple[float, float]] = []
        self._shared_target: tuple[float, float] | None = None
        self._shared_uid: str | None = None  # announce 来源 uid（结对仲裁用）
        self._shared_target_time: float = -1.0  # 收到 announce 的 sim_time，-1=未收到
        self._shared_confirmed: bool = True  # announce 的 confirmed 标记（默认确认）
        self._strike_target: tuple[float, float] | None = None  # 打击进行中的共享目标（我让位未响应）
        self._strike_time: float = -1.0  # 记录打击进行中的 sim_time
        self._join_time: float = 0.0
        self._is_wingman: bool = False  # True=僚机（收到 announce 加入），False=长机
        self._gimbal_phase: float = 0.0  # SEARCH 云台扫描相位
        self._last_reject_pos: tuple[float, float] | None = None
        self._last_reject_time: float = -1e9
        self._reject_streak: int = 0
        self._last_abort_pos: tuple[float, float] | None = None  # VERIFY 中止记录
        self._last_abort_time: float = -1e9
        self._time_synced: bool = False
        self._sim_dt: float = 0.0  # 本拍仿真时间增量（滤波 predict 用）
        self._last_sim_time: float | None = None
        self._dispatch_depth: int = 0  # 状态重入深度（振荡保护）
        self._last_hb_time: float = -1e9
        self._teammates: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)
        self._joiners: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)，J: 占位
        self._trackers: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)，A:/T: 长机
        self._avoid_pos: tuple[float, float] | None = None  # 仲裁退出目标的避让窗口
        self._avoid_until: float = -1e9

    def reset(self) -> None:
        self._state = State.SEARCH
        self._filter = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._verify_samples = []
        self._verify_lost_s = 0.0
        self._verify_pass_count = 0
        self._jump_count = 0
        self._fast_pass_s = 0.0
        self._wing_bail_s = 0.0
        self._slow_check_s = 0.0
        self._is_probe = False
        self._probe_check_s = 0.0
        self._probe_moving_s = 0.0
        self._confirmed_real = False
        self._orphan_s = 0.0
        self._promoted = False
        self._leader_pos = None
        self._leader_time = -1e9
        self._last_announce_time = -1e9
        self._sim_time = 0.0
        self._target = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_destroyed = []
        self._shared_target = None
        self._shared_uid = None
        self._shared_target_time = -1.0
        self._join_time = 0.0
        self._is_wingman = False
        self._gimbal_phase = 0.0
        self._last_reject_pos = None
        self._last_reject_time = -1e9
        self._reject_streak = 0
        self._last_abort_pos = None
        self._last_abort_time = -1e9
        self._time_synced = False
        self._sim_dt = 0.0
        self._last_sim_time = None
        self._dispatch_depth = 0
        self._last_hb_time = -1e9
        self._teammates = {}
        self._joiners = {}
        self._trackers = {}
        self._avoid_pos = None
        self._avoid_until = -1e9

    def decide(self, obs: CoopObs, dt: float) -> list[Command]:
        self._sync_time(obs, dt)
        cmds: list[Command] = []

        # 处理队友消息 + 过期清理
        self._ingest_comms(obs.comm_inbox)
        self._expire_shared_target()

        # 位置心跳（1Hz）：队友据此做 <200m proximity 避让
        if self._sim_time - self._last_hb_time >= _HB_INTERVAL:
            self._last_hb_time = self._sim_time
            cmds.append(broadcast(f"P:{obs.self.lat:.4f},{obs.self.lon:.4f}"))

        # 状态分发（重入深度保护）
        self._dispatch_depth = 0
        if _DEBUG:
            # 逐拍累计 VERIFY/TRACK 检出率（不受秒级打印过滤影响）
            if not hasattr(self, "_dbg_det_total"):
                self._dbg_det_total = 0
                self._dbg_det_hit = 0
            self._dbg_det_total += 1
            if obs.self.detection.detected:
                self._dbg_det_hit += 1
            # 每 10 引擎秒打印一次累计检出率
            if int(self._sim_time) % 10 == 0 and int(self._sim_time * 10) % 10 == 0:
                rate = self._dbg_det_hit / max(self._dbg_det_total, 1)
                print(
                    f"[RATE {self.my_uid}] t={self._sim_time:6.1f} {self._state.value:6s} "
                    f"累计拍={self._dbg_det_total} 检出={self._dbg_det_hit} 率={rate:.1%}",
                    flush=True,
                )
            if int(self._sim_time * 10) % 10 == 0:
                print(
                    f"[COOP {self.my_uid}] t={self._sim_time:6.1f} {self._state.value:6s} "
                    f"pos=({obs.self.lat:.5f},{obs.self.lon:.5f}) tgt={self._target} "
                    f"wing={self._is_wingman} det={obs.self.detection.detected}",
                    flush=True,
                )
        AGENT_DEBUG_STATE[self.my_uid] = {
            "state": self._state.value,
            "wingman": int(self._is_wingman),
            "confirmed": int(self._confirmed_real),
            "tgt": f"{self._target[0]:.5f},{self._target[1]:.5f}"
            if self._target else "",
            "shared": f"{self._shared_target[0]:.5f},{self._shared_target[1]:.5f}"
            if self._shared_target else "",
            "leader_age": round(self._sim_time - self._leader_time, 2)
            if self._leader_pos else 999.0,
            "dwell": round(self._dwell_time, 2),
        }
        return cmds + self._dispatch(obs, dt)

    def _dispatch(self, obs: CoopObs, dt: float) -> list[Command]:
        """按当前状态分发；状态转移后的重入统一走这里并限深。

        实测曾出现 VERIFY↔JOIN 同一拍内乒乓（announce 目标与 200~300m 外
        另一辆车的检测互相把对方当入口条件），递归重入直到 RecursionError
        被 runner 吞掉、整拍失控。限深后本拍返回空命令，下一拍继续。
        """
        self._dispatch_depth += 1
        if self._dispatch_depth > 6:
            return []
        if self._state == State.SEARCH:
            return self._do_search(obs, dt)
        if self._state == State.VERIFY:
            return self._do_verify(obs, dt)
        if self._state == State.TRACK:
            return self._do_track(obs, dt)
        return self._do_join(obs, dt)

    # ── 时间基准 ──────────────────────────────────────────────────────────

    def _sync_time(self, obs: CoopObs, dt: float) -> None:
        """同步引擎 sim_time（briefing.score_view 每拍更新），读不到回退 dt 累加。
        同时维护 self._sim_dt（本拍仿真时间增量，供滤波 predict）。

        必须用引擎时间而不是 dt 累加：runner 的控制节拍远快于引擎
        （实测 120 个控制周期 agent 时间 30.5s 引擎只走 12s，差 2.5 倍），
        用 dt 累加会让 OLS 速度低估 2.5 倍（12 m/s 真目标读成 ~5）、
        dwell/冷却等全部时间基准失真。
        """
        st = getattr(getattr(obs.briefing, "score_view", None), "sim_time", None)
        if isinstance(st, (int, float)):
            st = float(st)
            if not self._time_synced:
                # 首次同步：把以 0 初始化的时间戳字段平移到引擎时间轴
                self._last_report_time = st
                self._last_bc_time = st
                self._last_det_time = st
                self._time_synced = True
            sim_dt = st - self._last_sim_time if self._last_sim_time is not None else 0.0
            self._last_sim_time = st
            self._sim_time = st
            self._sim_dt = min(max(sim_dt, 0.0), 1.0)
        else:
            self._sim_time += dt
            self._sim_dt = dt

    # ── 通信 ──────────────────────────────────────────────────────────────

    def _ingest_comms(self, inbox) -> None:
        """解析队友广播，提取确认目标。A: 消息优先（announce）。"""
        for msg in inbox:
            p = msg.payload
            if p.startswith("A:"):
                # announce：确认真目标，需要僚机；第 3 段（可选）是 confirmed 标记
                try:
                    parts = p[2:].split(",")
                    la, lo = parts[0], parts[1]
                    self._shared_target = (float(la), float(lo))
                    self._shared_uid = msg.sender_uid
                    self._shared_target_time = self._sim_time
                    self._leader_pos = (float(la), float(lo))
                    self._leader_time = self._sim_time
                    self._shared_confirmed = (int(parts[2]) != 0
                                              if len(parts) > 2 else True)
                except Exception:
                    pass
            elif p.startswith("J:"):
                # 占位：有僚机正在收敛/协锁该目标，第三机不要再扎进去
                try:
                    la, lo = p[2:].split(",")
                    if msg.sender_uid != self.my_uid:
                        self._joiners[msg.sender_uid] = (
                            float(la),
                            float(lo),
                            self._sim_time,
                        )
                except Exception:
                    pass
            elif p.startswith("D:"):
                # 队友判定的已摧毁目标：同步进已摧毁列表（不再跟踪/判别）；
                # 打击结束 → 清除"打击进行中"标记，让位机恢复编队跟随
                try:
                    la, lo = p[2:].split(",")
                    pos = (float(la), float(lo))
                    if all(
                        haversine_m(pos[0], pos[1], d[0], d[1]) >= 150.0
                        for d in self._known_destroyed
                    ):
                        self._known_destroyed.append(pos)
                    if (
                        self._strike_target is not None
                        and haversine_m(pos[0], pos[1],
                                        self._strike_target[0],
                                        self._strike_target[1]) < 150.0
                    ):
                        self._strike_target = None
                        self._strike_time = -1.0
                except Exception:
                    pass
            elif p.startswith("P:"):
                # 队友位置心跳：proximity 避让用
                try:
                    la, lo = p[2:].split(",")
                    if msg.sender_uid != self.my_uid:
                        self._teammates[msg.sender_uid] = (
                            float(la),
                            float(lo),
                            self._sim_time,
                        )
                except Exception:
                    pass
            elif p.startswith("T:") and self._shared_target is None:
                # tracking 位置（仅在没有 announce 时使用）
                try:
                    la, lo = p[2:].split(",")
                    self._shared_target = (float(la), float(lo))
                    self._shared_uid = msg.sender_uid
                    self._shared_target_time = self._sim_time
                except Exception:
                    pass
            # A:/T: 的发出者是该目标的长机（TRACK 中），记入 _trackers
            # 供长/僚角色仲裁（双长机同圈盘旋是 proximity 扣分主因）
            if p.startswith(("A:", "T:")):
                try:
                    la, lo = p[2:].split(",")
                    if msg.sender_uid != self.my_uid:
                        self._trackers[msg.sender_uid] = (
                            float(la),
                            float(lo),
                            self._sim_time,
                        )
                        # 记录最新 leader 报告位置：僚机据此汇聚镜头
                        self._leader_pos = (float(la), float(lo))
                        self._leader_time = self._sim_time
                except Exception:
                    pass

    def _expire_shared_target(self) -> None:
        """过期 announce 清理：超过时限未收到新消息则放弃共享目标。"""
        if (
            self._shared_target is not None
            and self._shared_target_time >= 0.0
            and self._sim_time - self._shared_target_time > _ANNOUNCE_EXPIRE_S
        ):
            self._shared_target = None
            self._shared_uid = None
            self._shared_target_time = -1.0

    def _clamp_to_mission(self, lat: float, lon: float) -> tuple[float, float]:
        """任务区边界裁剪（boundary 罚分规避）：目标点拉回边界内收余量内。"""
        m = _BOUNDARY_MARGIN_M / 111320.0  # 纬度余量（经度按纬度缩放）
        lat = min(max(lat, _MISSION_LAT_MIN + m), _MISSION_LAT_MAX - m)
        lon_m = m / max(math.cos(math.radians(lat)), 0.1)
        lon = min(max(lon, _MISSION_LON_MIN + lon_m), _MISSION_LON_MAX - lon_m)
        return lat, lon

    def _avoid_teammates(self, obs: CoopObs, lat: float, lon: float,
                         radius_m: float = 250.0) -> tuple[float, float]:
        """proximity 罚分规避（<200m 每次扣 2 分）：队友在 300m 内时把
        盘旋/飞行目标点沿远离队友方向偏移（同心圈同目标盘旋相位自由可到
        0 间距；JOIN 接近路径会穿过长机圈——统一用目标点偏移拉开）。"""
        for tla, tlo, tt in self._teammates.values():
            if self._sim_time - tt > 5.0:
                continue
            d = haversine_m(obs.self.lat, obs.self.lon, tla, tlo)
            if d < 300.0:
                brg = bearing_rad(tla, tlo, obs.self.lat, obs.self.lon)
                lat = obs.self.lat + radius_m * math.cos(brg) / 111320.0
                lon = obs.self.lon + radius_m * math.sin(brg) / (
                    111320.0 * math.cos(math.radians(obs.self.lat)))
                break
        return lat, lon

    def _make_announce(self, tgt_lat: float, tgt_lon: float,
                       confirmed: bool = True) -> Command:
        """广播 announce：确认真目标，需要僚机。"""
        # 5 位小数（~1m）：僚机据 announce 位置做窄 FOV 汇聚瞄准，3 位小数
        # （~100m）会把目标瞄出 30° 视场（v09 实测 0 协锁）。
        # confirmed 标记：probe（5 m/s 候选，大概率诱饵）标 0——僚机对存疑
        # 目标先独立判别再合流，避免一窝蜂聚到诱饵（真实局双机同锁诱饵
        # 372s 的根源之一）；向后兼容：解析端缺省视为 1。
        return broadcast(f"A:{tgt_lat:.5f},{tgt_lon:.5f},{int(confirmed)}")

    def _make_broadcast(self, tgt_lat: float, tgt_lon: float) -> Command:
        """广播 tracking 位置。"""
        return broadcast(f"T:{tgt_lat:.5f},{tgt_lon:.5f}")

    def _make_join_claim(self, tgt_lat: float, tgt_lon: float) -> Command:
        """广播占位：我正在收敛/协锁该目标（proximity 避让，第三机勿入）。"""
        return broadcast(f"J:{tgt_lat:.5f},{tgt_lon:.5f}")

    def _join_slot_taken(self, lat: float, lon: float) -> str | None:
        """目标附近已有他机占位（J: 10s 内、距离 <300m）→ 返回占位 uid。"""
        for uid, (la, lo, t) in self._joiners.items():
            if uid == self.my_uid or self._sim_time - t > 10.0:
                continue
            if haversine_m(lat, lon, la, lo) < 300.0:
                return uid
        return None

    def _in_avoid_window(self, lat: float, lon: float) -> bool:
        """仲裁退出后的避让窗口（15s、300m）：防退出后当拍重进 VERIFY 空转。"""
        return (
            self._avoid_pos is not None
            and self._sim_time < self._avoid_until
            and haversine_m(lat, lon, self._avoid_pos[0], self._avoid_pos[1]) < 300.0
        )

    def _claims_near(self, lat: float, lon: float) -> tuple[str | None, str | None]:
        """目标附近的长机（A:/T:）与僚机（J:）占位，10s 内、300m 内有效。"""
        lead = wing = None
        for uid, (la, lo, t) in self._trackers.items():
            if uid != self.my_uid and self._sim_time - t <= 10.0                     and haversine_m(lat, lon, la, lo) < 300.0:
                lead = uid
        wing = self._join_slot_taken(lat, lon)
        return lead, wing

    def _mark_reject(self) -> None:
        """记录判别否决位置（OLS 出界/TRACK 超时；VERIFY 接触丢失中止
        走 _mark_abort，不进此序列）。同一位置连续否决时冷却指数升档
        （路线终点永久停驻的诱饵：20→40→80→160s，防反复鉴别空转；
        停顿真目标 WaitTime ≤30s，仍能在升档间隙被重新判别）。"""
        if self._target:
            if self._last_reject_pos and haversine_m(
                self._target[0],
                self._target[1],
                self._last_reject_pos[0],
                self._last_reject_pos[1],
            ) < _REJECT_RADIUS_M:
                self._reject_streak += 1
            else:
                self._reject_streak = 0
            self._last_reject_pos = self._target
        self._last_reject_time = self._sim_time

    def _in_reject_cooldown(self, lat: float, lon: float) -> bool:
        if self._last_reject_pos is not None and (
            haversine_m(lat, lon, self._last_reject_pos[0], self._last_reject_pos[1])
            < _REJECT_RADIUS_M
        ):
            cooldown = _REJECT_COOLDOWN_S * (2 ** min(self._reject_streak, 3))
            if self._sim_time - self._last_reject_time < cooldown:
                return True
        # VERIFY 接触丢失中止：5s 平冷却、300m 半径，防当拍重进空转
        if self._last_abort_pos is not None and (
            haversine_m(lat, lon, self._last_abort_pos[0], self._last_abort_pos[1])
            < _ABORT_RADIUS_M
        ):
            return self._sim_time - self._last_abort_time < _ABORT_COOLDOWN_S
        return False

    def _mark_abort(self) -> None:
        """VERIFY 接触丢失中止：只记短冷却，不进否决升档序列。
        中止是'没看清'，不是'判了诱饵'——升档冷却叠在密集车场会形成
        死亡螺旋（中止→升档→更难完成判别→再中止）。"""
        self._last_abort_pos = self._target
        self._last_abort_time = self._sim_time

    # ── SEARCH：割草机覆盖搜索本机条带 ─────────────────────────────────────

    def _do_search(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 生成搜索航点（如果还没有）：真/诱全部 Start 的区域覆盖扫描。
        # 训练阶段沿真路线先验扫描遭遇率高，但验证集真目标换新路线后先验
        # 失效；改扫 TARGET+DECOY 全部 Start 的并集（44 点覆盖路网关键
        # 交汇处，验证集新 Start 仍在 A* 路网节点附近）——以遍历时长换
        # 验证集不失效。
        if not self._search_waypoints:
            self._search_waypoints = coverage_waypoints_for_uid(self.my_uid, n_shares=3)
            self._wp_idx = 0

        # 收到队友确认目标 → JOIN（跳过已摧毁目标；已有僚机占位则不去——
        # K=2 只需双机，第三机扎进去只会触发 <200m proximity 扣分）。
        # v9 在线任务分配：三机中离目标近的响应打击，远的继续搜索——
        # 编队保证双机同区，打击只需两台，第三台保持覆盖不中断。
        if self._shared_target is not None and self._shared_uid != self.my_uid:
            near_destroyed = any(
                haversine_m(self._shared_target[0], self._shared_target[1], d[0], d[1])
                < 150.0
                for d in self._known_destroyed
            )
            slot_taken = self._join_slot_taken(*self._shared_target)
            if not near_destroyed and slot_taken is None:
                # 距离决策：另一架非判真机明显更近 → 它去打击，我继续搜索
                my_d = haversine_m(
                    obs.self.lat, obs.self.lon,
                    self._shared_target[0], self._shared_target[1])
                other_d = None
                for uid, (tla, tlo, tt) in self._teammates.items():
                    if uid == self._shared_uid or self._sim_time - tt > 5.0:
                        continue
                    other_d = haversine_m(
                        tla, tlo, self._shared_target[0], self._shared_target[1])
                    break
                if (
                    other_d is not None
                    and my_d > other_d + _STRIKE_DECISION_MARGIN_M
                ):
                    # 让位：记录打击进行中（不跟随编队长），继续搜索
                    self._strike_target = self._shared_target
                    self._strike_time = self._sim_time
                else:
                    self._state = State.JOIN
                    self._target = self._shared_target
                    self._join_time = 0.0
                    self._filter = None
                    return self._dispatch(obs, dt)

        # 检测到目标 → VERIFY（跳过已摧毁目标；长机+僚机已齐的目标也跳过——
        # K=2 已满员，第三个进去只会在同一 100m 圈里制造 proximity 扣分）。
        # 不做诱饵标记跳过：诱饵也在动（5 m/s 全域路线），位置标记会失效；
        # 且中途停顿的真目标读数同静止，误标记会永久隐藏它。
        if det.detected and det.target_lat is not None:
            near_destroyed = any(
                haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in self._known_destroyed
            )
            lead, wing = self._claims_near(det.target_lat, det.target_lon)
            fully_manned = lead is not None and wing is not None
            if _DEBUG:
                print(
                    f"[GATE {self.my_uid}] t={self._sim_time:6.1f} "
                    f"det=({det.target_lat:.5f},{det.target_lon:.5f}) "
                    f"nd={near_destroyed} lead={lead} wing={wing} "
                    f"avoid={self._in_avoid_window(det.target_lat, det.target_lon)} "
                    f"rcd={self._in_reject_cooldown(det.target_lat, det.target_lon)} "
                    f"shared={self._shared_target}",
                    flush=True,
                )
            if (
                not near_destroyed
                and not fully_manned
                and not self._in_avoid_window(det.target_lat, det.target_lon)
                and not self._in_reject_cooldown(det.target_lat, det.target_lon)
            ):
                self._state = State.VERIFY
                self._target = (det.target_lat, det.target_lon)
                self._filter = CvFilter(obs.self.lat, obs.self.lon)
                self._verify_samples = []
                self._verify_lost_s = 0.0
                self._route_matcher.reset()
                self._verify_pass_count = 0  # 新候选：二次验证计数归零
                self._fast_pass_s = 0.0
                self._confirmed_real = False  # 新候选：判真前不视为已确认
                self._is_wingman = False  # 自己发现的候选：判别通过即长机
                # 不在此处 announce：候选未判别，提前 announce 会让全队
                # 收敛到同一个静止诱饵（诊断证实）。判别通过进 TRACK 时再
                # announce（见 _do_track 首次广播）。
                return self._dispatch(obs, dt)

        # 沿割草机航点飞行（到达 loiter 圈内即切下一点）
        if self._search_waypoints:
            wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            dist = haversine_m(obs.self.lat, obs.self.lon, wp_lat, wp_lon)
            if dist < 200.0:
                self._wp_idx = (self._wp_idx + 1) % len(self._search_waypoints)
                wp_lat, wp_lon = self._search_waypoints[self._wp_idx]

            # v9 编队：僚机跟随编队长（20001），横向偏移 400m 同行——三机
            # 同区保证 K=2 可达（发现即双机可锁）。编队长打击中（announce
            # 未过期）或我自己让位打击（_strike_target 有效）→ 独立沿航点
            # 搜索，不跟随（否则会飞进打击区、打断覆盖）。
            in_strike = (
                self._strike_target is not None
                and self._sim_time - self._strike_time <= _ANNOUNCE_EXPIRE_S
            )
            leader_striking = (
                self._shared_uid == _LEADER_UID
                and self._shared_target is not None
                and self._sim_time - self._shared_target_time <= _ANNOUNCE_EXPIRE_S
            )
            if (
                self.my_uid != _LEADER_UID
                and not in_strike
                and not leader_striking
            ):
                leader = self._teammates.get(_LEADER_UID)
                if leader is not None and self._sim_time - leader[2] <= 5.0:
                    dlat = _WING_OFFSET_M / 111320.0
                    dlon = _WING_OFFSET_M / (
                        111320.0 * math.cos(math.radians(leader[0])))
                    sign = 1.0 if self.my_uid == "20002" else -1.0
                    wp_lat = leader[0] + sign * dlat
                    wp_lon = leader[1] + sign * dlon
            # 任务区边界裁剪（boundary 罚分规避）：偏移点/航点统一拉回边界内
            wp_lat, wp_lon = self._clamp_to_mission(wp_lat, wp_lon)

            # proximity 避让：队友在 300m 内时，沿远离队友方向退 300m 再飞
            # （<200m 每次扣 2 分；600s 局 8 次把 accuracy 得分清零）
            fly_lat, fly_lon = wp_lat, wp_lon
            for tla, tlo, tt in self._teammates.values():
                if self._sim_time - tt > 5.0:
                    continue
                d = haversine_m(obs.self.lat, obs.self.lon, tla, tlo)
                if d < 300.0:
                    brg = bearing_rad(tla, tlo, obs.self.lat, obs.self.lon)
                    fly_lat = obs.self.lat + 300.0 * math.cos(brg) / 111320.0
                    fly_lon = obs.self.lon + 300.0 * math.sin(brg) / (
                        111320.0 * math.cos(math.radians(obs.self.lat))
                    )
                    break
            cmds.append(
                fly_to(fly_lat, fly_lon, alt=_SEARCH_ALT, speed=_SEARCH_SPEED)
            )

        # 云台扇扫（pan ±90°，tilt -60° ~ -30°）——不扫描检测覆盖率极低
        self._gimbal_phase += dt * 0.5
        pan = 90.0 * math.sin(self._gimbal_phase)
        tilt = -45.0 + 15.0 * math.sin(self._gimbal_phase * 0.7)
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(_SEARCH_FOV))
        return cmds

    # ── VERIFY：OLS 速度判别（CvFilter 同步更新供 TRACK 接管） ──────────

    def _do_verify(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 锁定目标 + 采样 + CvFilter 更新
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            self._verify_lost_s = 0.0
            # 平滑位置（CvFilter）：云台指向、jump 判定、OLS 样本统一用它。
            # 原实现直接用含噪检测位置（AccuracySimulator σ=50m）：云台跟着
            # 噪声抖 → 目标偏出 FOV 中心 → 多目标时锁切换 → 位置跳 >250m →
            # jump 否决（实测每 20s 窗口 jump=3~11，全部候选被否决、永不 TRACK）。
            if self._filter is None:
                self._filter = CvFilter(obs.self.lat, obs.self.lon)
            if not self._filter.is_initialized():
                self._filter.initialize(det.target_lat, det.target_lon)
            else:
                self._filter.predict(self._sim_dt)
                self._filter.update_position(det.target_lat, det.target_lon)
            est = self._filter.position_wgs84()
            if est is None:
                est = (det.target_lat, det.target_lon)
            # 突变检测：相邻平滑位移 >250m（锁切换/错误关联，噪声已被滤除）
            if self._verify_samples:
                pla, plo = self._verify_samples[-1][1], self._verify_samples[-1][2]
                if haversine_m(est[0], est[1], pla, plo) \
                        > _JUMP_THRESHOLD_M:
                    self._jump_count += 1
            self._verify_samples.append(
                (self._sim_time, est[0], est[1])
            )
            self._route_matcher.append(self._sim_time, est[0], est[1])
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                est[0],
                est[1], uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            # VERIFY 保持 60° 宽 FOV：30° 窄锥在真实引擎下会被近距诱饵
            # 抢锁（探针数据：真目标在 FOV 内时 18% 锁到诱饵，诱饵距光轴
            # 仅 14.8°）；宽锥让 OLS 有足够样本判别，TRACK 再切窄。
            cmds.append(set_gimbal_fov(_SEARCH_FOV))
        else:
            # 无检测拍：继续指向滤波/最后已知位置（UAV 在 40 m/s 接近，
            # 云台不跟随 LOS 会迅速偏出 FOV → 接触丢失 → 中止）
            self._verify_lost_s += self._sim_dt
            est = None
            if self._filter is not None and self._filter.is_initialized():
                self._filter.predict(self._sim_dt)
                est = self._filter.position_wgs84()
            elif self._target:
                est = self._target
            if est is not None:
                pan, tilt = compute_gimbal_angles(
                    obs.self.lat,
                    obs.self.lon,
                    obs.self.alt,
                    est[0],
                    est[1], uav_heading_deg=obs.self.heading_deg)
                cmds.append(point_gimbal(pan, tilt))
                cmds.append(set_gimbal_fov(_SEARCH_FOV))

        # 连续丢失 → 放弃判别（中止不记否决，5s 平冷却防死亡螺旋）
        if self._verify_lost_s > _VERIFY_LOST_ABORT_S:
            self._mark_abort()
            self._state = State.SEARCH
            self._target = None
            self._filter = None
            self._verify_samples = []
            return self._dispatch(obs, dt)

        # 收到队友确认目标 → JOIN（优先协同）
        if self._shared_target is not None and self._target is not None:
            d = haversine_m(
                self._shared_target[0],
                self._shared_target[1],
                self._target[0],
                self._target[1],
            )
            if d > 200.0 and self._join_slot_taken(*self._shared_target) is None:
                # 不同目标且无人占位，队友确认的是另一个
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._filter = None
                self._verify_samples = []
                return self._dispatch(obs, dt)

        # OLS 时间窗判别：攒满 20s 时间跨度即判定（不要求固定样本数，
        # 稀疏检出按时间戳回归）。fast-pass 已禁用——CvFilter 在 5 m/s
        # 诱饵上收敛过程有 ~25s 处速度过冲尖峰（实测冲到 7.24），满足
        # fast-pass 条件误判真（debug10 三机全跟诱饵的根因）。
        if self._verify_samples and (
            self._sim_time - self._verify_samples[0][0] >= _VERIFY_WINDOW_S
        ):
            speed = ols_speed_mps(self._verify_samples)
            if _DEBUG and (len(self._verify_samples) >= 10):
                import math as _m
                _M = 111320.0
                t0, la0, lo0 = self._verify_samples[0]
                t1, la1, lo1 = self._verify_samples[-1]
                disp = _m.hypot((la1-la0)*_M, (lo1-lo0)*_M*_m.cos(_m.radians(la0)))
                print(
                    f"[OLS {self.my_uid}] t={self._sim_time:6.1f} "
                    f"speed={speed:5.1f} n={len(self._verify_samples)} "
                    f"span={t1-t0:.1f}s disp={disp:.0f}m",
                    flush=True,
                )
            self._verify_samples = []
            if self._jump_count >= _JUMP_MIN:
                # 锁跳变/错误关联（相邻检测位移突跳 ≥2 次）：非连续移动，
                # 无论 OLS 读数如何都否决——跳变模式会让 OLS 入带
                # （来回跳净斜率可能落在带内），先于速度带拦截。
                if _DEBUG:
                    print(f"[VERDICT {self.my_uid}] t={self._sim_time:6.1f} "
                          f"JUMP-reject jump={self._jump_count} speed={speed:.1f}",
                          flush=True)
                self._jump_count = 0
                self._verify_pass_count = 0
                self._mark_reject()
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                return self._dispatch(obs, dt)
            self._jump_count = 0
            if _DEBUG:
                # 样本跨度诊断：首末样本位置差 vs 时间差 → 表观速度
                s0 = self._verify_samples_history[0] if hasattr(self, '_verify_samples_history') and self._verify_samples_history else None
                import math as _m
                _M = 111320.0
                if len(self._verify_samples) >= 2:
                    t0, la0, lo0 = self._verify_samples[0]
                    t1, la1, lo1 = self._verify_samples[-1]
                    dn = (la1-la0)*_M; de = (lo1-lo0)*_M*_m.cos(_m.radians(la0))
                    disp = _m.hypot(dn, de)
                    print(
                        f"[OLS {self.my_uid}] t={self._sim_time:6.1f} "
                        f"speed={speed:5.1f} jump={self._jump_count} "
                        f"n={len(self._verify_samples)} span={t1-t0:.1f}s "
                        f"disp={disp:.0f}m appv={disp/max(t1-t0,0.01):.1f}",
                        flush=True,
                    )
            if _VERIFY_SPEED_MIN <= speed <= _VERIFY_SPEED_MAX:
                if _DEBUG:
                    print(f"[VERDICT {self.my_uid}] t={self._sim_time:6.1f} "
                          f"BAND-TRACK speed={speed:.1f}", flush=True)
                if speed >= _VERIFY_FAST_MIN:
                    # ≥8.0：9/12 m/s 档确定性速度（5 m/s 漂移 P(≥8)=1.4%，
                    # 蒙特卡洛）→ 单窗口即可判真；误判由 TRACK 低速核查自愈
                    self._state = State.TRACK
                    self._confirmed_real = True
                    self._is_probe = False
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0
                    self._dwell_time = 0.0
                    self._track_time = 0.0
                    self._last_det_time = self._sim_time
                    self._verify_pass_count = 0
                    self._slow_check_s = 0.0
                    return self._dispatch(obs, dt)
                # [6.5, 8.0)：5 m/s 漂移区间（13.5% 误入带），须第二独立
                # 窗口确认（误报 13.5%² ≈ 1.8%）才 TRACK。窗口间清样本
                # 继续 VERIFY（目标锁定不变）。
                self._verify_pass_count += 1
                if self._verify_pass_count >= _VERIFY_PASSES:
                    self._state = State.TRACK
                    self._confirmed_real = True
                    self._is_probe = False
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0
                    self._dwell_time = 0.0
                    self._track_time = 0.0
                    self._last_det_time = self._sim_time
                    self._verify_pass_count = 0
                    self._slow_check_s = 0.0
                    return self._dispatch(obs, dt)
            else:
                # 速度出界，三档处理：
                #   [2.0, 7.0) 低速带外（5 m/s 档真/诱同速不可分）→ 直接进
                #     TRACK 验证模式（_is_probe=True）：不再查 Start 池、不
                #     reject——训练集烘焙的 Start 池在验证集可能失效，唯一
                #     判别通道是引擎行为差异（被 ≥2 架盯满 20s 后真目标冻结、
                #     诱饵继续移动，见 TRACK 验证尾段）；
                #   <2.0 静止 → reject（真目标不停顿，盯静止对象纯浪费）；
                #   >14.5 锁跳变虚高 → reject + 冷却。
                self._verify_pass_count = 0
                if _PROBE_SPEED_MIN <= speed < _VERIFY_SPEED_MIN:
                    # 5 m/s 档：速度与诱饵相同，OLS 无法区分。
                    # 路线匹配存在不可修复的几何缺陷（2/18 诱饵路线物理重叠
                    # 于真目标路网，距离+方向检查均失效），不作为判真依据。
                    # 统一走 probe：真伪由 dwell 20s 后引擎行为（冻结/移动）裁决。
                    route_score = self._route_matcher.match_score(self._sim_time)
                    if _DEBUG:
                        print(f"[VERDICT {self.my_uid}] t={self._sim_time:6.1f} "
                              f"PROBE-TRACK speed={speed:.1f} route_score={route_score}",
                              flush=True)
                    self._state = State.TRACK
                    self._confirmed_real = False
                    self._is_probe = True
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0
                    self._dwell_time = 0.0
                    self._track_time = 0.0
                    self._last_det_time = self._sim_time
                    self._slow_check_s = 0.0
                    return self._dispatch(obs, dt)
                # 速度出界（<2.0 或 >14.5）→ reject
                self._mark_reject()
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                return self._dispatch(obs, dt)

        # 飞向目标区域（僚机候选在 400m 外圈判别，避开长机的 100m 圈）
        if self._target:
            # 保持距离盘旋（450m 圈、20 m/s）：目标稳定在 FOV（60° 在
            # 450m 处脚印 ~520m）→ 检测连续 → 120 样本 ~17s 可凑齐。
            # 原实现 40 m/s 直飞目标 100m 圈：接近中 LOS 快速变化、目标
            # 频繁偏出 FOV，检测拍率实测掉到 ~10%（debug7），120 样本
            # 永远凑不齐、OLS 判定从未触发。判真后由 TRACK 接管接近。
            # proximity 避让：队友 <300m 时圈中心偏移（同心圈同目标
            # VERIFY 相位自由可到 0 间距，<200m 每次扣 2 分）
            v_lat, v_lon = self._avoid_teammates(
                obs, self._target[0], self._target[1])
            cmds.append(
                fly_to(
                    v_lat,
                    v_lon,
                    alt=_SEARCH_ALT,
                    speed=_TRACK_SPEED,
                    loiter_radius=_VERIFY_RADIUS,
                )
            )

        return cmds

    # ── TRACK：盘旋跟踪 + 广播 + 上报 ───────────────────────────────────

    def _do_track(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._track_time += self._sim_dt

        # probe 验证模式盯丢退出：检测全丢 >10s 时 dwell 无法累计、验证尾段
        # 也无法读数 → 放弃回 SEARCH（不记否决，_mark_abort 的 5s 平冷却防
        # 死循环）。非 probe 保持最小改动，靠 90s 超时兜底。
        if self._is_probe and self._sim_time - self._last_det_time > _PROBE_LOST_ABORT_S:
            self._mark_abort()
            self._state = State.SEARCH
            self._target = None
            self._filter = None
            self._is_wingman = False
            self._dwell_time = 0.0
            self._is_probe = False
            self._probe_check_s = 0.0
            self._probe_moving_s = 0.0
            return self._dispatch(obs, dt)

        # 协同：单打独斗（自己盯的目标无人 claim）时响应队友 announce——
        # probe（5 m/s 候选，大概率诱饵）让位去 JOIN；已判真的 9/12 绝不让位
        # （v07 诊断：确认 9/12 的 UAV 曾放弃真目标去 JOIN 5 m/s 候选）。
        if (
            self._shared_target is not None
            and self._target is not None
        ):
            d = haversine_m(
                self._shared_target[0],
                self._shared_target[1],
                self._target[0],
                self._target[1],
            )
            lead, wing = self._claims_near(*self._target)
            solo = lead is None and wing is None
            if (
                d > 200.0
                and solo
                and self._is_probe
                and self._join_slot_taken(*self._shared_target) is None
            ):
                self._mark_abort()  # 放弃当前 probe，不记否决
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._filter = None
                self._is_wingman = False
                self._is_probe = False
                self._probe_check_s = 0.0
                self._probe_moving_s = 0.0
                self._dwell_time = 0.0
                return self._dispatch(obs, dt)

        # 角色仲裁（每拍）：同一目标只能一长一僚，第三机退出。
        # 独立 VERIFY→TRACK 路径不受 J: 门禁约束，多机各自判别通过后会
        # 在同一 100m 圈里盘旋（v15 局 proximity 303 次的根因），在此仲裁：
        # 双长机 → uid 小者留任长机，大者补僚机位（僚机位也被占则退出）；
        # 双僚机 → uid 小者留任，大者退出。退出后不记否决（目标是真），
        # 靠 VERIFY 入口的"长僚已齐"门禁防回卷。
        if self._target:
            lead, wing = self._claims_near(*self._target)
            if not self._is_wingman and lead is not None and str(lead) < str(self.my_uid):
                if wing is None or str(wing) >= str(self.my_uid):
                    self._is_wingman = True  # 降级补僚机位
                else:
                    self._avoid_pos = self._target
                    self._avoid_until = self._sim_time + 15.0
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._dwell_time = 0.0
                    return self._dispatch(obs, dt)
            elif self._is_wingman and wing is not None and str(wing) < str(self.my_uid):
                self._avoid_pos = self._target
                self._avoid_until = self._sim_time + 15.0
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                self._dwell_time = 0.0
                return self._dispatch(obs, dt)

        # 孤儿僚机晋升：leader 的 A:/T: claim 消失超过阈值 → 晋升为长机并
        # 重新 announce。孤儿只发 J: 占位，其他 UAV 不会据此 JOIN，它盯着的
        # 目标（可能是真目标）永远凑不齐第二架（v06 诊断：20002 对 5 m/s
        # 真目标 99.8% 连续有效却全程无人加入，因为它已不是长机、不发 A:）。
        if self._is_wingman and self._target:
            lead, _ = self._claims_near(*self._target)
            if lead is not None:
                self._orphan_s = 0.0
            else:
                self._orphan_s += self._sim_dt
                if self._orphan_s >= _ORPHAN_PROMOTE_S:
                    self._is_wingman = False
                    self._orphan_s = 0.0
                    self._promoted = True
                    self._last_bc_time = self._sim_time - _BC_INTERVAL  # 立即广播
        else:
            self._orphan_s = 0.0

        # 低速核查（长机/僚机通用）：5 m/s 档真/诱同速不可分，速度不能单独
        # 裁决，只能确认"慢"。两档处理：
        #   <4.0（静止/异常，5 m/s 档不会触发）→ 15s 后 bailout 退出；
        #   <5.5（5 m/s 档）→ 持续 10s 后置 _is_probe=True 进入验证模式
        #     继续盯（不再查 Start 池退出）——真伪由 dwell 满 20s 后的
        #     冻结/继续移动判别；v≥5.5（9/12 档）恢复 _is_probe=False。
        if (
            self._filter is not None
            and self._filter.is_converged(15.0)
        ):
            v = self._filter.speed_mps()
            if v < _WING_BAIL_SPEED:
                self._wing_bail_s += self._sim_dt
                if self._wing_bail_s >= _WING_BAIL_S:
                    self._mark_abort()
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._is_wingman = False
                    self._dwell_time = 0.0
                    self._wing_bail_s = 0.0
                    self._slow_check_s = 0.0
                    return self._dispatch(obs, dt)
            else:
                self._wing_bail_s = 0.0
                if v < _SLOW_CHECK_SPEED:
                    self._slow_check_s += self._sim_dt
                    if self._slow_check_s >= _SLOW_CHECK_S:
                        # 进入验证模式：继续盯，真伪交给验证尾段
                        self._is_probe = True
                        self._probe_check_s = 0.0
                        self._probe_moving_s = 0.0
                        self._slow_check_s = 0.0
                else:
                    self._slow_check_s = 0.0
                    self._is_probe = False
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0

        # 跟踪目标位置更新 + CvFilter 滤波（report_target 用滤波位置，
        # 回归：TRACK 曾不更新滤波，上报位置冻结在 VERIFY 结束时刻）
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            if self._target:
                d = haversine_m(
                    det.target_lat, det.target_lon, self._target[0], self._target[1]
                )
                if d < 250.0:
                    self._target = (det.target_lat, det.target_lon)
            if self._filter is None:
                self._filter = CvFilter(obs.self.lat, obs.self.lon)
            if not self._filter.is_initialized():
                self._filter.initialize(det.target_lat, det.target_lon)
            else:
                self._filter.predict(self._sim_dt)
                self._filter.update_position(det.target_lat, det.target_lon)
        else:
            if self._filter is not None and self._filter.is_initialized():
                self._filter.predict(self._sim_dt)

        # 盯防计时
        tracking = (
            det.detected
            and det.target_lat is not None
            and self._target
            and haversine_m(
                det.target_lat, det.target_lon, self._target[0], self._target[1]
            )
            < 250.0
        )
        if _DEBUG and self._target:
            print(
                f"[TRK {self.my_uid}] t={self._sim_time:6.1f} "
                f"det={int(det.detected)} gap={self._sim_time - self._last_det_time:5.2f} "
                f"dwell={self._dwell_time:6.2f} probe={int(self._is_probe)}",
                flush=True,
            )
        if tracking:
            gap = self._sim_time - self._last_det_time
            if (
                self._dwell_time > 0 and gap <= _TRACK_GRACE_S + self._sim_dt
            ) or self._dwell_time == 0:
                self._dwell_time += self._sim_dt
            else:
                self._dwell_time = self._sim_dt
            self._last_det_time = self._sim_time

        # 盯防满 20s：必须有队友同时在场（K=2 协锁）才判定摧毁。
        # 评估器按"双机同时盯防 20s"判毁，单机 20s 不算——旧逻辑单机
        # 满 20s 就标记摧毁并离开，亲手拆掉协锁（debug5 局实测：长机
        # solo 满 20s 离开 + nd=True 永久拒绝返回，目标永远杀不掉）。
        # 单机满 20s 继续盯防等僚机（announce 仍在发，T: 2Hz 持续）。
        if self._dwell_time >= _TRACK_DWELL_S and self._target:
            lead, wing = self._claims_near(*self._target)
            if lead is not None or wing is not None:
                # 盯满 20s + 队友在场。不凭通信 claim 直接宣布摧毁——
                # 评测器的 coop 需要双机同时有效检测连续 20s，任一机的
                # >2s 检测间隙都会清零；agent 自身 dwell 满 20s 时评测器
                # 往往还没满（TRACK 检出率仅 ~40%，v01 诊断 coop_ticks=0
                # 而 agent 侧多次在 dwell 20s 自宣摧毁）。统一等引擎冻结
                # 证据（滤波速度 <1.5 连续 5s = 真目标已被引擎摧毁）。
                v = self._filter.speed_mps() if self._filter is not None else None
                if v is None:
                    pass
                elif v < _PROBE_STOP_SPEED:
                    self._probe_check_s += self._sim_dt
                    self._probe_moving_s = 0.0
                else:
                    self._probe_check_s = 0.0
                    if self._is_probe:
                        # 验证模式：盯满 20s 后仍高速移动 = 诱饵（identified）
                        self._probe_moving_s += self._sim_dt
                    elif v < _VERIFY_SPEED_MIN:
                        # 非 probe 判真失误：OLS 滤波过冲把 5 m/s 诱饵判成
                        # ≥7（seed9 实测诱饵被双机同锁 372s——非 probe 永不
                        # 放弃）。真目标被冻结走 <1.5 判毁分支；9/12 正常
                        # 跟踪速度 ≥7 不累计，等评测器协锁满 20s 冻结。
                        self._probe_moving_s += self._sim_dt
                    else:
                        self._probe_moving_s = 0.0
                if self._probe_check_s >= _PROBE_CHECK_S:
                    # 引擎冻结 → 真目标已摧毁：D: 广播 + 记摧毁 + 回 SEARCH
                    self._known_destroyed.append(self._target)
                    cmds.append(
                        broadcast(f"D:{self._target[0]:.5f},{self._target[1]:.5f}")
                    )
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._is_wingman = False
                    self._dwell_time = 0.0
                    self._is_probe = False
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0
                    return cmds + self._dispatch(obs, dt)  # 保留 D: 广播命令
                if self._probe_moving_s >= _PROBE_CHECK_S:
                    # 诱饵（identified）或判真失误的 5 m/s 候选：记冷却回 SEARCH
                    self._mark_reject()
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._is_wingman = False
                    self._dwell_time = 0.0
                    self._is_probe = False
                    self._probe_check_s = 0.0
                    self._probe_moving_s = 0.0
                    return self._dispatch(obs, dt)

        # 超时未摧毁 → 记冷却后回 SEARCH（可能是诱饵或停顿真目标）。
        # 但双机已配对时 90s 不拆对：评测器的 coop 需要 20s 连续双锁，
        # 任一机 >2s 间隙就清零重来，90s 可能不够；配对中延到 180s，
        # 把拆对造成的清零让位给验证尾段/冻结确认的正常裁决。
        # probe（5 m/s 候选，大概率诱饵）不享受配对延时：真实局实测
        # leader 判真失误盯诱饵 → 广播诱饵位置 → 僚机 JOIN 一起无限盯
        # （30% 检测丢失下 dwell 满不了 20s，尾段放弃永不触发），真目标
        # 无人管——probe 90s 拆对，把 UAV 回收去找真目标。
        if self._track_time >= _TRACK_TIMEOUT_S:
            lead, wing = self._claims_near(*self._target)
            paired = lead is not None or wing is not None
            if not (paired and not self._is_probe
                    and self._track_time < _TRACK_TIMEOUT_PAIRED_S):
                self._mark_reject()
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                self._is_wingman = False
                self._dwell_time = 0.0
                return self._dispatch(obs, dt)

        # 广播：长机首次进入 TRACK 时 announce，之后定期 T: 位置
        if self._target and self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            if self._is_wingman:
                # 僚机定期占位广播：第三机看到 J: 就不再扎进同一目标
                cmds.append(self._make_join_claim(self._target[0], self._target[1]))
            elif self._promoted or self._dwell_time <= self._sim_dt * 2:
                # 长机首次确认 / 孤儿晋升：announce（需要僚机）；probe 标存疑
                cmds.append(self._make_announce(
                    self._target[0], self._target[1],
                    confirmed=(self._confirmed_real and not self._is_probe)))
                self._promoted = False
                self._last_announce_time = self._sim_time
            elif (
                self._confirmed_real
                and self._sim_time - self._last_announce_time
                >= _CONFIRMED_REANNOUNCE_S
            ):
                # 已判真长机周期性重发 announce：晚到/刚空闲的 UAV 优先配对
                # （probe 的 5 m/s 候选不重发，避免抢走 9/12 的配对资源）
                cmds.append(self._make_announce(
                    self._target[0], self._target[1], confirmed=True))
                self._last_announce_time = self._sim_time
            else:
                # 定期位置广播：用滤波位置（平滑、±50m 噪声减到 ~20m）+
                # 速度外推 0.5s（广播间隔）——僚机据此做窄 FOV 汇聚瞄准，
                # 预测位置减接收端滞后（目标 12 m/s × 0.5s = 6m）
                if self._filter and self._filter.is_initialized():
                    bc_lat, bc_lon = self._filter.position_wgs84()
                    ve, vn = self._filter.velocity_mps()
                    bc_lat = bc_lat + vn * 0.5 / 111320.0
                    bc_lon = bc_lon + ve * 0.5 / (
                        111320.0 * math.cos(math.radians(bc_lat)))
                else:
                    bc_lat, bc_lon = self._target
                cmds.append(self._make_broadcast(bc_lat, bc_lon))

        # 云台 + 飞行（僚机用更大盘旋半径避免 <200m 惩罚）
        # 云台瞄准：滤波平滑位置优先（30° FOV 下瞄含噪检测点 σ=50m @250m ≈
        # ±11° 角抖，目标在光轴边缘晃 → FOV 内诱饵抢锁，实测 TRACK 期间锁的
        # 全是附近诱饵、双机协锁从未落到同一真目标）；滤波未初始化才回退
        # 原始检测点，检测丢失走滤波预测。长机 250m / 僚机 600m 同心盘旋，
        # 最近距离 350m > 200m 罚线
        if self._target:
            aim = self._target
            if (
                self._is_wingman
                and self._leader_pos is not None
                and self._sim_time - self._leader_time <= 2.0
            ):
                # 僚机云台融合：自身滤波判真（速度落判真带 7-14.5 + 检测
                # 新鲜 ≤1s + 收敛 10s）→ 信自身（实时、无广播滞后）；否则
                # 信 leader 报告锚定。原逻辑只在"未确认 announce"时信自身，
                # 确认时纯 leader 报告——但 leader 报告有 0.5s 滞后 + 滤波
                # 残余误差，僚机 500m 外瞄偏 5-8° → 目标偏出 FOV 中心 →
                # 诱饵抢锁（真实局 JOIN 到达后锁定失败、双机从不 TRACK
                # 同一目标的根因之一）。
                own_ok = (
                    self._filter is not None
                    and self._filter.is_initialized()
                    and self._filter.is_converged(10.0)
                    and _VERIFY_SPEED_MIN <= self._filter.speed_mps()
                    <= _VERIFY_SPEED_MAX
                    and self._sim_time - self._last_det_time <= 1.0
                )
                aim = (self._filter.position_wgs84() if own_ok
                       else self._leader_pos)
            elif self._filter and self._filter.is_initialized():
                aim = self._filter.position_wgs84()
            elif det.detected and det.target_lat is not None and det.target_lon is not None:
                aim = (det.target_lat, det.target_lon)
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                aim[0],
                aim[1], uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_WING_FOV if self._is_wingman else _TRACK_FOV))
            loiter = _WING_LOITER_M if self._is_wingman else _TRACK_LOITER_M
            # v13 同侧编队已回退：僚机绕 leader 沿视线外推点（同视线→丢失
            # 相关）在真实局退化（真目标被锁 714→0 tick、penalty 15）——
            # 外推 400m 让僚机距目标 650m（FOV 脚印大+边缘丢失+位置滞后），
            # 相关性收益被抵消。恢复同心圈（250/500，半径差 250m > 200m
            # 罚线、视线差 ~26° 中等相关），proximity 由避让兜底。
            t_lat, t_lon = self._avoid_teammates(
                obs, self._target[0], self._target[1])
            cmds.append(
                fly_to(
                    t_lat,
                    t_lon,
                    alt=_SEARCH_ALT,
                    speed=_TRACK_SPEED,
                    loiter_radius=loiter,
                )
            )

            # report_target（仅 OLS 判真的 9/12 档目标）：5 m/s 档真/诱
            # 不可分，上报诱饵位置会匹配到最近存活真目标、打爆它的 RMSE
            # （污染 accuracy 维度）。三道闸门：
            #   * _confirmed_real 且 not _is_probe：OLS 判真且未被低速核查
            #     转 probe——TRACK 锁切换诱饵后滤波速度回落、核查转 probe
            #     即停报（实测判真机 TRACK 期间 33.7% 时间锁诱饵，filter
            #     惯性让速度 >5.5 维持数秒，无此闸门会持续上报诱饵位置）；
            #   * 速度 >7.0（判真下限，非 5.5）：OLS 判真失误（诱饵滤波
            #     速度 5.5-7 灰色地带）时永不转 probe、持续上报——门槛提到
            #     判真下限把灰色地带全拦，9/12 真目标正常 >7 不受影响；
            #   * 检测新鲜度 ≤1s：无锁拍（实测 57.4%）用预测位置上报，
            #     目标转弯时漂移大——漏报不罚、报错必罚，宁缺毋滥。
            if (
                self._sim_time - self._last_report_time >= _REPORT_INTERVAL
                and self._filter
                and self._filter.is_initialized()
                and self._confirmed_real
                and not self._is_probe
                and self._filter.speed_mps() > _VERIFY_SPEED_MIN
                and self._sim_time - self._last_det_time <= 1.0
            ):
                self._last_report_time = self._sim_time
                est_lat, est_lon = self._filter.position_wgs84()
                cmds.append(report_target(est_lat, est_lon))

        return cmds

    # ── JOIN：收到队友广播，收敛到共享目标 ────────────────────────────────

    def _do_join(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._join_time += self._sim_dt

        # JOIN 超时（announce 过期或始终未检测到目标）→ SEARCH
        if self._target is None or self._join_time >= _JOIN_TIMEOUT_S:
            self._state = State.SEARCH
            self._target = None
            self._shared_target = None
            self._shared_target_time = -1.0
            self._join_time = 0.0
            return self._dispatch(obs, dt)

        # 占位冲突仲裁：他机也在收敛同一目标时，uid 大者退让回 SEARCH
        # （双机同时收到同一 announce 的竞态；J: 心跳 10s 内有效）
        holder = self._join_slot_taken(*self._target)
        if holder is not None and str(holder) < str(self.my_uid):
            self._state = State.SEARCH
            self._target = None
            self._shared_target = None
            self._shared_target_time = -1.0
            self._join_time = 0.0
            return self._dispatch(obs, dt)

        # 占位广播（2Hz）：告诉第三机这个目标已有僚机
        if self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            cmds.append(self._make_join_claim(self._target[0], self._target[1]))

        # 检测不覆盖 _target：announce 位置才是收敛基准（曾把 _target 改成
        # 检测位置，检测落在 announce 点 200~300m 外时（目标+诱饵搭档）
        # VERIFY↔JOIN 同拍乒乓直至递归崩溃）。检测只用于云台瞄准（下方）。

        # 接近到 JOIN 圈内（<650m）且有检测 → 僚机直接 TRACK（边跟踪边判别）。
        # 老设计是先过 12s VERIFY 判别，但 announce 已经过长机速度判别
        # （fast-pass/OLS），僚机再走一遍 VERIFY 只是把 K=2 协锁推迟 12s+；
        # 直入 TRACK 让协锁 dwell 从到达即开始累计，假阳性由 TRACK 内的
        # 后台低速 bailout 兜底。阈值 > 600m JOIN 圈（在圈上即可转 TRACK）；
        # 僚机 600m 圈与长机 250m 圈错开 350m，不触发 <200m proximity 扣分。
        # 存疑目标（probe announce，5 m/s 候选大概率诱饵）：僚机不直入
        # TRACK，先 VERIFY 独立判别——长机 OLS 判真失误时僚机合流会一起
        # 无限盯诱饵（真实局双机同锁诱饵 372s），独立判别可拦截。
        dist_to_target = haversine_m(
            obs.self.lat, obs.self.lon, self._target[0], self._target[1]
        )
        if dist_to_target < _JOIN_TRACK_THRESHOLD_M and det.detected:
            if not self._shared_confirmed:
                self._state = State.VERIFY
                self._filter = CvFilter(obs.self.lat, obs.self.lon)
                self._verify_samples = []
                self._verify_lost_s = 0.0
                self._route_matcher.reset()
                self._verify_pass_count = 0
                self._confirmed_real = False
                self._is_wingman = False
                return self._dispatch(obs, dt)
            self._state = State.TRACK
            self._is_wingman = True
            self._confirmed_real = False  # 僚机未过 OLS 判别，不上报
            self._is_probe = False
            self._probe_check_s = 0.0
            self._probe_moving_s = 0.0
            self._dwell_time = 0.0
            self._track_time = 0.0
            self._last_det_time = self._sim_time
            self._wing_bail_s = 0.0
            self._slow_check_s = 0.0
            self._filter = CvFilter(obs.self.lat, obs.self.lon)
            return self._dispatch(obs, dt)

        # 飞向共享目标（600m 圈收敛：与长机 250m 圈最近距离 350m，>200m
        # 罚线；圈径与僚机 TRACK 圈一致，入圈后不改半径）。proximity 避让：
        # 接近路径穿过长机圈时目标点偏移拉开
        j_lat, j_lon = self._avoid_teammates(
            obs, self._target[0], self._target[1])
        cmds.append(
            fly_to(
                j_lat,
                j_lon,
                alt=_SEARCH_ALT,
                speed=_SEARCH_SPEED,
                loiter_radius=_JOIN_LOITER_M,
            )
        )

        # 云台锁定：JOIN 阶段始终瞄收敛基准（announce/leader 最新位置），不追本机
        # 检测点——否则首个检测若是诱饵会把相机带偏，进场后锁不回真目标（v13 实测
        # JOIN 途中相机不指目标区，20003 进场后 20s 无有效锁）。窄 FOV 由引擎
        # "锁离光轴最近目标"模型保证真目标优先。
        aim_pos = self._target
        if (
            self._leader_pos is not None
            and self._sim_time - self._leader_time <= 2.0
        ):
            aim_pos = self._leader_pos
        pan, tilt = compute_gimbal_angles(
            obs.self.lat,
            obs.self.lon,
            obs.self.alt,
            aim_pos[0],
            aim_pos[1], uav_heading_deg=obs.self.heading_deg)
        cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(_TRACK_FOV))

        return cmds

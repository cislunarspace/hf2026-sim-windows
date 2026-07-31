"""赛题三对抗集群全域搜索 Agent。

四状态 FSM（移植自赛题二 coop_decoy，机制已在仿真中验证）：
  SEARCH → VERIFY → TRACK
                ↑       ↓ (摧毁/超时) → SEARCH
  JOIN ←── (收到队友广播确认目标)

与赛题二的差异：
  - 10 UAV、K=3 协同盯防（1 长机 + 2 僚机；僚机槽位计数制，盘旋半径
    长机 100m / 僚机按 uid 哈希 300m 或 500m 错层，避开 <200m proximity 扣分）
  - 判别速度带：真目标 4/5/5/5/6/6/6/7/7/8 m/s（路线 Start 停 30s），
    诱饵恒 5 m/s 无停顿。滤波速度 <4.5（4 m/s 档）或 ∈[5.5,9.0]
    （6/7/8 m/s 档）判真；~5 m/s（4 真 + 20 诱饵，运动不可分）按诱饵
    判否——通过线 kill≥0.7 恰好 = 7 个非 5 m/s 目标全杀，5 m/s 类放弃
  - SAM 防空区（进入 2s 即毁，高度锁定 500m 只能水平绕行）：所有
    fly_to 目的点先经 _push_out_of_sam 推出 approximate_zones 近似 bbox
  - 动态干扰：obs.self.jammed 翻真时 1Hz 广播 "J:lat,lon" 预警；
    僚机占位消息改用 "C:lat,lon"（claim）避免与干扰预警冲突
  - SEARCH 用 uid 哈希扇区（5×2 网格）+ 扇区螺旋，替代路线先验扫描
    （10 机覆盖全图，扇区比路线分摊更匀）

快速通过带为什么只有 [5.5,9.0] 不含 <4.5（离线仿真实测）：
  CvFilter 速度初值 0（p11=25 防过冲），任何目标的滤波速度都从 0 缓慢
  爬坡，前 ~5s 一律读成 <4.5——低速带快速通过会把 5 m/s 诱饵在 t≈4s
  全部放进 TRACK。4 m/s 档真目标走 120 样本 OLS 兜底判真（OLS=4.0<4.5），
  只比快速通过慢 ~3s，代价可接受。
"""

import hashlib
import math
import os
from enum import Enum

_DEBUG = os.environ.get("SWARM_AGENT_DEBUG") == "1"

from algorithms.estimation.cv_kalman import CvFilter
from algorithms.estimation.geometry import bearing_rad, haversine_m
from algorithms.estimation.motion import ols_speed_mps
from algorithms.search.spiral import generate_spiral
from algorithms.tracking.gimbal import compute_gimbal_angles

from competition.sdk.core.commands import (
    Command,
    broadcast,
    fly_to,
    point_gimbal,
    report_target,
    set_gimbal_fov,
)
from competition.sdk.scenarios.adversarial_swarm import SwarmAgent
from competition.sdk.scenarios.adversarial_swarm.observation import SwarmObs

# ── 常量 ──────────────────────────────────────────────────────────────────

_SEARCH_ALT = 500.0  # 搜索高度（m），手册：全程锁定 500m
_SEARCH_SPEED = 30.0  # 搜索速度（m/s）
_TRACK_SPEED = 25.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 100.0  # 长机盘旋半径（m）；僚机按 uid 哈希 300/500m 错层
_WING_SLOTS = 2  # 僚机槽位数（K=3：1 长机 + 2 僚机）

_VERIFY_SAMPLES = 120  # VERIFY 判别所需检测样本数（12s @10Hz）
# 真目标速度 4/5/5/5/6/6/6/7/7/8 m/s，诱饵恒 5 m/s。<4.5 必真（4 m/s 档，
# 含路线 Start 停顿 30s 读数 ~0——诱饵从不停顿，静止即真）；[5.5,9.0]
# 覆盖 6/7/8 m/s 档；[4.5,5.5) 是 5 m/s 类（4 真 + 20 诱饵不可分）按诱饵
# 判否；>9.0 超过真目标极速（8）必是锁跳变虚高。
_VERIFY_SPEED_SLOW_MAX = 4.5  # 低速真目标带上界（m/s，不含）
_VERIFY_SPEED_FAST_MIN = 5.5  # 高速真目标带下界（m/s）
_VERIFY_SPEED_FAST_MAX = 9.0  # 高速真目标带上界（m/s）
_VERIFY_LOST_ABORT_S = 2.0  # VERIFY 中连续丢失超过此时长则放弃（不记诱饵）
_FAST_PASS_S = 3.0  # 快速通过：滤波速度持续落在快速通过带的时长（s）。
# 快速通过带只有 [5.5,9.0]：v0 初值下滤波速度从 0 爬坡，前 ~5s 任何目标
# 都读成 <4.5，低速带会把 5 m/s 诱饵全部快速放进 TRACK（离线仿真实测
# t≈4s 即触发）。4 m/s 档由 120 样本 OLS 兜底，不设对称的快速判否
_WING_BAIL_S = 15.0  # 僚机后台判别：速度持续落在诱饵带的 bailout 时长（s）。
# 真目标 WaitTime 停顿 ≤30s，但停顿读数 ~0 不在诱饵带内，不误伤
_REJECT_COOLDOWN_S = 20.0  # 判别否决/中止后的重检测冷却（s）：防同帧循环重进 VERIFY，
# 又不永久标记——停顿中的真目标冷却后重遇可重新判别
_REJECT_RADIUS_M = 500.0  # 冷却生效的检测距离（m）
_ABORT_COOLDOWN_S = 5.0  # VERIFY 接触丢失中止的平冷却（s）：不构成判别结论，
# 只要防当拍重进空转；用判别否决的升档冷却会在密集车场形成死亡螺旋
_ABORT_RADIUS_M = 300.0  # 中止冷却的生效半径（m）：只挡同一辆车

_TRACK_DWELL_S = 20.0  # 盯防摧毁时间（s，K=3 三机同时盯防）
_TRACK_GRACE_S = 2.0  # 丢失容忍时间（s）
_TRACK_TIMEOUT_S = 90.0  # 跟踪超时（s）：长机须咬住目标等两架僚机
# （跨区飞来 ~50s + 协锁 20s）

_JOIN_TIMEOUT_S = 60.0  # JOIN 超时（s），扇区间距 ~2km @30m/s 需 ~70s 收敛
_ANNOUNCE_EXPIRE_S = 15.0  # announce 过期时间（s），防止收敛到已放弃的诱饵

_BC_INTERVAL = 0.5  # 广播间隔（s，2Hz）
_HB_INTERVAL = 1.0  # 位置心跳间隔（s，1Hz；P:lat,lon，用于 proximity 避让）
_JAM_BC_INTERVAL = 1.0  # 干扰预警广播间隔（s，1Hz；J:lat,lon）
_REPORT_INTERVAL = 1.0  # 上报间隔（s）

_SPIRAL_RADIUS_M = 600.0  # 扇区螺旋半径（m）
_SPIRAL_PITCH_M = 200.0  # 螺旋螺距（m）
_WP_ARRIVE_M = 100.0  # 搜索航点到达判定距离（m）
_TRACK_FOV = 60.0  # 跟踪 FOV（°）
_SEARCH_FOV = 60.0  # 搜索 FOV（°）

_SAM_MARGIN_DEG = 0.002  # SAM bbox 外推余量（°，~200m）

# 赛题三地图 bbox（手册：26.9818~27.0250, 124.9800~125.0203）
_BBOX: tuple[tuple[float, float], tuple[float, float]] = (
    (26.9818, 124.9800),
    (27.0250, 125.0203),
)


# ── 工具函数 ──────────────────────────────────────────────────────────────


def _uid_sector(uid: str, cols: int = 5, rows: int = 2) -> tuple[float, float]:
    """uid 哈希映射到 5×2 网格扇区中心 (lat, lon)。"""
    (lat_min, lon_min), (lat_max, lon_max) = _BBOX
    h = int(hashlib.md5(uid.encode()).hexdigest(), 16)
    idx = h % (cols * rows)
    r, c = divmod(idx, cols)
    lat = lat_min + (r + 0.5) * (lat_max - lat_min) / rows
    lon = lon_min + (c + 0.5) * (lon_max - lon_min) / cols
    return (lat, lon)


def _push_out_of_sam(
    lat: float, lon: float, briefing, margin_deg: float = _SAM_MARGIN_DEG
) -> tuple[float, float]:
    """若点落在 SAM（air_defense）近似 bbox 内，推到最近边外 + 余量。

    手册：高度锁定 500m，只能水平绕行。bbox 已外扩约 20%，推出 bbox 即安全。
    通信干扰区不致命，不推避。
    """
    for z in getattr(briefing, "approximate_zones", None) or ():
        if z.kind != "air_defense":
            continue
        (la0, lo0), (la1, lo1) = z.bbox
        if la0 <= lat <= la1 and lo0 <= lon <= lo1:
            d_s, d_n = lat - la0, la1 - lat
            d_w, d_e = lon - lo0, lo1 - lon
            m = min(d_s, d_n, d_w, d_e)
            if m == d_s:
                lat = la0 - margin_deg
            elif m == d_n:
                lat = la1 + margin_deg
            elif m == d_w:
                lon = lo0 - margin_deg
            else:
                lon = lo1 + margin_deg
    return (lat, lon)


def _in_true_speed_band(v: float) -> bool:
    """OLS 判别速度带：<4.5（4 m/s 档/停顿）或 [5.5,9.0]（6/7/8 m/s 档）。"""
    return v < _VERIFY_SPEED_SLOW_MAX or _VERIFY_SPEED_FAST_MIN <= v <= _VERIFY_SPEED_FAST_MAX


def _in_fast_pass_band(v: float) -> bool:
    """快速通过带：只有 [5.5,9.0]（低速带会被 v0 爬坡污染，见文件头注释）。"""
    return _VERIFY_SPEED_FAST_MIN <= v <= _VERIFY_SPEED_FAST_MAX


def _in_decoy_band(v: float) -> bool:
    """诱饵带 [4.5,5.5)：僚机后台 bailout 用。"""
    return _VERIFY_SPEED_SLOW_MAX <= v < _VERIFY_SPEED_FAST_MIN


class State(Enum):
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    JOIN = "JOIN"


class SwarmSearchAgent(SwarmAgent):
    """赛题三参赛 Agent：CvFilter 滤波 + 扇区螺旋搜索 + K=3 盘旋协锁 + SAM 推避。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state = State.SEARCH
        self._filter: CvFilter | None = None
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s: float = 0.0
        self._fast_pass_s: float = 0.0  # 快速通过累计时长
        self._wing_bail_s: float = 0.0  # 僚机后台判别诱饵带累计时长
        self._sim_time = 0.0
        self._target: tuple[float, float] | None = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._last_jam_bc_time = -_JAM_BC_INTERVAL
        self._known_destroyed: list[tuple[float, float]] = []
        self._shared_target: tuple[float, float] | None = None
        self._shared_target_time: float = -1.0  # 收到 announce 的 sim_time，-1=未收到
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
        self._joiners: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)，C: 占位
        self._trackers: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)，A:/T: 长机
        self._avoid_pos: tuple[float, float] | None = None  # 仲裁退出目标的避让窗口
        self._avoid_until: float = -1e9
        # 僚机盘旋半径按 uid 哈希错层（300/500m）：与长机 100m 圈两两间距
        # ≥200m，避开 proximity 罚线
        h = int(hashlib.md5(my_uid.encode()).hexdigest(), 16)
        self._wingman_loiter = 300.0 + (h % 2) * 200.0

    def reset(self) -> None:
        self._state = State.SEARCH
        self._filter = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._verify_samples = []
        self._verify_lost_s = 0.0
        self._fast_pass_s = 0.0
        self._wing_bail_s = 0.0
        self._sim_time = 0.0
        self._target = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._last_jam_bc_time = -_JAM_BC_INTERVAL
        self._known_destroyed = []
        self._shared_target = None
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

    def decide(self, obs: SwarmObs, dt: float) -> list[Command]:
        self._sync_time(obs, dt)
        cmds: list[Command] = []

        # 处理队友消息 + 过期清理
        self._ingest_comms(obs.comm_inbox)
        self._expire_shared_target()

        # 位置心跳（1Hz）：队友据此做 <200m proximity 避让
        if self._sim_time - self._last_hb_time >= _HB_INTERVAL:
            self._last_hb_time = self._sim_time
            cmds.append(broadcast(f"P:{obs.self.lat:.4f},{obs.self.lon:.4f}"))

        # 动态干扰自感知 → 广播预警（1Hz）。僚机占位用 C:，J: 只表示干扰
        if obs.self.jammed and self._sim_time - self._last_jam_bc_time >= (
            _JAM_BC_INTERVAL
        ):
            self._last_jam_bc_time = self._sim_time
            cmds.append(broadcast(f"J:{obs.self.lat:.3f},{obs.self.lon:.3f}"))

        # 状态分发（重入深度保护）
        self._dispatch_depth = 0
        if _DEBUG and int(self._sim_time * 10) % 10 == 0:
            print(
                f"[SWARM {self.my_uid}] t={self._sim_time:6.1f} {self._state.value:6s} "
                f"pos=({obs.self.lat:.5f},{obs.self.lon:.5f}) tgt={self._target} "
                f"wing={self._is_wingman} det={obs.self.detection.detected}",
                flush=True,
            )
        return cmds + self._dispatch(obs, dt)

    def _dispatch(self, obs: SwarmObs, dt: float) -> list[Command]:
        """按当前状态分发；状态转移后的重入统一走这里并限深。

        赛题二实测曾出现 VERIFY↔JOIN 同一拍内乒乓（announce 目标与
        200~300m 外另一辆车的检测互相把对方当入口条件），递归重入直到
        RecursionError 被 runner 吞掉、整拍失控。限深后本拍返回空命令，
        下一拍继续。
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

    def _sync_time(self, obs: SwarmObs, dt: float) -> None:
        """同步引擎 sim_time（briefing.score_view 每拍更新），读不到回退 dt 累加。
        同时维护 self._sim_dt（本拍仿真时间增量，供滤波 predict）。

        必须用引擎时间而不是 dt 累加：runner 的控制节拍远快于引擎
        （赛题二实测 120 个控制周期 agent 时间 30.5s 引擎只走 12s，
        差 2.5 倍），用 dt 累加会让 OLS 速度低估 2.5 倍（8 m/s 真目标
        读成 ~3）、dwell/冷却等全部时间基准失真。
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
        """解析队友广播，提取确认目标。A: 消息优先（announce）。

        协议：A: announce（长机确认）/ T: 长机跟踪位置 / C: 僚机占位 /
        D: 已摧毁 / P: 位置心跳 / J: 干扰预警（v1 仅忽略，不据此改航线）。
        """
        for msg in inbox:
            p = msg.payload
            if p.startswith("A:"):
                # announce：确认真目标，需要僚机
                try:
                    la, lo = p[2:].split(",")
                    self._shared_target = (float(la), float(lo))
                    self._shared_target_time = self._sim_time
                except Exception:
                    pass
            elif p.startswith("C:"):
                # 占位：有僚机正在收敛/协锁该目标，槽位满（2 架）就不要再扎进去
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
                # 队友判定的已摧毁目标：同步进已摧毁列表（不再跟踪/判别）
                try:
                    la, lo = p[2:].split(",")
                    pos = (float(la), float(lo))
                    if all(
                        haversine_m(pos[0], pos[1], d[0], d[1]) >= 150.0
                        for d in self._known_destroyed
                    ):
                        self._known_destroyed.append(pos)
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
                    self._shared_target_time = self._sim_time
                except Exception:
                    pass
            # A:/T: 的发出者是该目标的长机（TRACK 中），记入 _trackers
            # 供长/僚角色仲裁（多机同圈盘旋是 proximity 扣分主因）
            if p.startswith(("A:", "T:")):
                try:
                    la, lo = p[2:].split(",")
                    if msg.sender_uid != self.my_uid:
                        self._trackers[msg.sender_uid] = (
                            float(la),
                            float(lo),
                            self._sim_time,
                        )
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
            self._shared_target_time = -1.0

    def _make_announce(self, tgt_lat: float, tgt_lon: float) -> Command:
        """广播 announce：确认真目标，需要僚机。"""
        return broadcast(f"A:{tgt_lat:.3f},{tgt_lon:.3f}")

    def _make_broadcast(self, tgt_lat: float, tgt_lon: float) -> Command:
        """广播 tracking 位置。"""
        return broadcast(f"T:{tgt_lat:.3f},{tgt_lon:.3f}")

    def _make_join_claim(self, tgt_lat: float, tgt_lon: float) -> Command:
        """广播占位：我正在收敛/协锁该目标（C: = claim，与干扰预警 J: 区分）。"""
        return broadcast(f"C:{tgt_lat:.3f},{tgt_lon:.3f}")

    def _wing_uids_near(self, lat: float, lon: float) -> list[str]:
        """目标附近的僚机占位 uid 列表（C: 10s 内、距离 <300m）。"""
        return [
            uid
            for uid, (la, lo, t) in self._joiners.items()
            if uid != self.my_uid
            and self._sim_time - t <= 10.0
            and haversine_m(lat, lon, la, lo) < 300.0
        ]

    def _join_slot_full(self, lat: float, lon: float) -> bool:
        """目标附近僚机槽位已满（≥2 架占位）→ 第三机不要再扎进去。"""
        return len(self._wing_uids_near(lat, lon)) >= _WING_SLOTS

    def _lead_near(self, lat: float, lon: float) -> str | None:
        """目标附近的长机 uid（A:/T: 10s 内、距离 <300m），无则 None。"""
        for uid, (la, lo, t) in self._trackers.items():
            if (
                uid != self.my_uid
                and self._sim_time - t <= 10.0
                and haversine_m(lat, lon, la, lo) < 300.0
            ):
                return uid
        return None

    def _in_avoid_window(self, lat: float, lon: float) -> bool:
        """仲裁退出后的避让窗口（15s、300m）：防退出后当拍重进 VERIFY 空转。"""
        return (
            self._avoid_pos is not None
            and self._sim_time < self._avoid_until
            and haversine_m(lat, lon, self._avoid_pos[0], self._avoid_pos[1]) < 300.0
        )

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

    # ── SEARCH：扇区螺旋搜索 ─────────────────────────────────────────────

    def _do_search(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 生成搜索航点（如果还没有）：uid 哈希扇区中心 + 阿基米德螺旋。
        # 不用赛题二的路线先验扫描：10 机分工，扇区网格覆盖比按路线分摊
        # 更匀；且赛题三有 20 条诱饵全域路线，路线先验会把机群吸到车流上
        if not self._search_waypoints:
            center = _uid_sector(self.my_uid)
            self._search_waypoints = generate_spiral(
                center[0],
                center[1],
                radius_m=_SPIRAL_RADIUS_M,
                pitch_m=_SPIRAL_PITCH_M,
            )
            self._wp_idx = 0

        # 收到队友确认目标 → JOIN（跳过已摧毁目标；僚机槽位满（2 架）则不
        # 去——K=3 只需三机，第四机扎进去只会触发 <200m proximity 扣分）
        if self._shared_target is not None:
            near_destroyed = any(
                haversine_m(self._shared_target[0], self._shared_target[1], d[0], d[1])
                < 150.0
                for d in self._known_destroyed
            )
            if not near_destroyed and not self._join_slot_full(*self._shared_target):
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._filter = None
                return self._dispatch(obs, dt)

        # 检测到目标 → VERIFY（跳过已摧毁目标；长机+双僚已齐的目标也跳
        # 过——K=3 已满员，第四个进去只会在同一圈里制造 proximity 扣分）。
        # 不做诱饵位置标记跳过：诱饵也在动（5 m/s 全域路线），位置标记会
        # 失效；且中途停顿的真目标读数同静止，误标记会永久隐藏它。
        if det.detected and det.target_lat is not None:
            near_destroyed = any(
                haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in self._known_destroyed
            )
            lead = self._lead_near(det.target_lat, det.target_lon)
            fully_manned = lead is not None and self._join_slot_full(
                det.target_lat, det.target_lon
            )
            if _DEBUG:
                print(
                    f"[GATE {self.my_uid}] t={self._sim_time:6.1f} "
                    f"det=({det.target_lat:.5f},{det.target_lon:.5f}) "
                    f"nd={near_destroyed} lead={lead} "
                    f"wings={len(self._wing_uids_near(det.target_lat, det.target_lon))} "
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
                self._fast_pass_s = 0.0
                self._is_wingman = False  # 自己发现的候选：判别通过即长机
                # 不在此处 announce：候选未判别，提前 announce 会让全队
                # 收敛到同一个静止诱饵（赛题二诊断证实）。判别通过进
                # TRACK 时再 announce（见 _do_track 首次广播）。
                return self._dispatch(obs, dt)

        # 沿螺旋航点飞行（到达判定圈内即切下一点）
        if self._search_waypoints:
            wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            dist = haversine_m(obs.self.lat, obs.self.lon, wp_lat, wp_lon)
            if dist < _WP_ARRIVE_M:
                self._wp_idx = (self._wp_idx + 1) % len(self._search_waypoints)
                wp_lat, wp_lon = self._search_waypoints[self._wp_idx]

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
            # SAM 推避：航点/避让点都可能落进防空区，目的点先推出 bbox
            fly_lat, fly_lon = _push_out_of_sam(fly_lat, fly_lon, obs.briefing)
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

    def _do_verify(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 锁定目标 + 采样 + CvFilter 更新
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            self._verify_lost_s = 0.0
            self._verify_samples.append(
                (self._sim_time, det.target_lat, det.target_lon)
            )
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                det.target_lat,
                det.target_lon, uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))

            if self._filter is not None:
                if not self._filter.is_initialized():
                    self._filter.initialize(det.target_lat, det.target_lon)
                else:
                    self._filter.predict(self._sim_dt)
                    self._filter.update_position(det.target_lat, det.target_lon)
        else:
            # 无检测拍：继续指向滤波/最后已知位置（UAV 在 30 m/s 接近，
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
                cmds.append(set_gimbal_fov(_TRACK_FOV))

        # 连续丢失 → 放弃判别（中止不记否决，5s 平冷却防死亡螺旋）
        if self._verify_lost_s > _VERIFY_LOST_ABORT_S:
            self._mark_abort()
            self._state = State.SEARCH
            self._target = None
            self._filter = None
            self._verify_samples = []
            return self._dispatch(obs, dt)

        # 收到队友确认目标 → JOIN（优先协同；槽位满则不去）
        if self._shared_target is not None and self._target is not None:
            d = haversine_m(
                self._shared_target[0],
                self._shared_target[1],
                self._target[0],
                self._target[1],
            )
            if d > 200.0 and not self._join_slot_full(*self._shared_target):
                # 不同目标且有僚机空位，队友确认的是另一个
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._filter = None
                self._verify_samples = []
                return self._dispatch(obs, dt)

        # 快速通过：CvFilter 收敛后速度持续 3s 落在快速通过带 [5.5,9.0]
        # → 真目标，立即 TRACK（不必等满 120 样本；低速带不快速通过，
        # v0 爬坡会把 5 m/s 诱饵读成 <4.5，见文件头注释；4 m/s 档与
        # 5 m/s 判否都交给 120 样本 OLS 兜底）
        if (
            self._filter is not None
            and self._filter.is_converged(15.0)
            and _in_fast_pass_band(self._filter.speed_mps())
        ):
            self._fast_pass_s += self._sim_dt
            if self._fast_pass_s >= _FAST_PASS_S:
                self._state = State.TRACK
                self._dwell_time = 0.0
                self._track_time = 0.0
                self._last_det_time = self._sim_time
                self._verify_samples = []
                return self._dispatch(obs, dt)
        else:
            self._fast_pass_s = 0.0

        # 样本足够：OLS 最小二乘速度判别（<4.5 或 [5.5,9.0] 速度带）
        if len(self._verify_samples) >= _VERIFY_SAMPLES:
            speed = ols_speed_mps(self._verify_samples)
            self._verify_samples = []
            if _in_true_speed_band(speed):
                # 真目标 → TRACK（_is_wingman 在进入 VERIFY 时已设定，此处保留）
                self._state = State.TRACK
                self._dwell_time = 0.0
                self._track_time = 0.0
                self._last_det_time = self._sim_time
                return self._dispatch(obs, dt)
            else:
                # 速度出界：5 m/s 类（诱饵或 5 m/s 档真目标，不可分按诱饵
                # 处理），或 >9.0 的锁跳变虚高（真目标极速 8）。记冷却后
                # 回 SEARCH。
                self._mark_reject()
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                return self._dispatch(obs, dt)

        # 飞向目标区域（SAM 推避；僚机候选在 400m 外圈判别，避开长机的
        # 100m 圈）
        if self._target:
            lat, lon = _push_out_of_sam(
                self._target[0], self._target[1], obs.briefing
            )
            cmds.append(
                fly_to(
                    lat,
                    lon,
                    alt=_SEARCH_ALT,
                    speed=_SEARCH_SPEED,
                    loiter_radius=_LOITER_RADIUS * 4 if self._is_wingman else _LOITER_RADIUS,
                )
            )

        return cmds

    # ── TRACK：盘旋跟踪 + 广播 + 上报 ───────────────────────────────────

    def _do_track(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._track_time += dt

        # 角色仲裁（每拍）：同一目标只能一长二僚，第四机退出。
        # 独立 VERIFY→TRACK 路径不受 C: 门禁约束，多机各自判别通过后会
        # 在同一 100m 圈里盘旋（赛题二 v15 局 proximity 303 次的根因），
        # 在此仲裁：双长机 → uid 小者留任长机，大者补僚机位（僚机位
        # 已满则退出）；僚机超员 → uid 最大的退出。退出后不记否决
        # （目标是真），靠 VERIFY 入口的"长僚已齐"门禁防回卷。
        if self._target:
            lead = self._lead_near(*self._target)
            wings = self._wing_uids_near(*self._target)
            smaller_wings = [u for u in wings if str(u) < str(self.my_uid)]
            if not self._is_wingman and lead is not None and str(lead) < str(self.my_uid):
                if len(smaller_wings) < _WING_SLOTS:
                    self._is_wingman = True  # 降级补僚机位
                else:
                    self._avoid_pos = self._target
                    self._avoid_until = self._sim_time + 15.0
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._dwell_time = 0.0
                    return self._dispatch(obs, dt)
            elif self._is_wingman and len(smaller_wings) >= _WING_SLOTS:
                self._avoid_pos = self._target
                self._avoid_until = self._sim_time + 15.0
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                self._dwell_time = 0.0
                return self._dispatch(obs, dt)

        # 僚机后台判别：直入 TRACK 不设 VERIFY，若速度持续 15s 落在诱饵
        # 带 [4.5,5.5)（假阳性诱饵）则中止退出——不记否决（可能是
        # WaitTime 停顿的真目标，停顿读数 ~0 不在诱饵带内）
        if (
            self._is_wingman
            and self._filter is not None
            and self._filter.is_converged(15.0)
        ):
            if _in_decoy_band(self._filter.speed_mps()):
                self._wing_bail_s += self._sim_dt
                if self._wing_bail_s >= _WING_BAIL_S:
                    self._mark_abort()
                    self._state = State.SEARCH
                    self._target = None
                    self._filter = None
                    self._is_wingman = False
                    self._dwell_time = 0.0
                    self._wing_bail_s = 0.0
                    return self._dispatch(obs, dt)
            else:
                self._wing_bail_s = 0.0

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
        if tracking:
            gap = self._sim_time - self._last_det_time
            if (
                self._dwell_time > 0 and gap <= _TRACK_GRACE_S + dt
            ) or self._dwell_time == 0:
                self._dwell_time += dt
            else:
                self._dwell_time = dt
            self._last_det_time = self._sim_time

        # 盯防满 20s：必须两名队友同时在场（K=3 协锁）才判定摧毁。
        # 评估器按"三机同时盯防 20s"判毁，单机/双机 20s 不算——旧逻辑
        # 满 20s 就标记摧毁并离开，亲手拆掉协锁（赛题二 debug5 局实测：
        # 长机 solo 满 20s 离开 + nd=True 永久拒绝返回，目标永远杀不掉）。
        # 人没齐继续盯防等僚机（announce 仍在发，T: 2Hz 持续）。
        if self._dwell_time >= _TRACK_DWELL_S and self._target:
            mates = set(self._wing_uids_near(*self._target))
            lead = self._lead_near(*self._target)
            if lead is not None:
                mates.add(lead)
            if len(mates) >= _WING_SLOTS:
                self._known_destroyed.append(self._target)
                cmds.append(
                    broadcast(f"D:{self._target[0]:.3f},{self._target[1]:.3f}")
                )
                self._state = State.SEARCH
                self._target = None
                self._filter = None
                self._is_wingman = False
                self._dwell_time = 0.0
                return cmds + self._dispatch(obs, dt)  # 保留 D: 广播命令

        # 超时未摧毁（协同未到齐）→ 记冷却后回 SEARCH（可能是诱饵或停顿真目标）
        if self._track_time >= _TRACK_TIMEOUT_S:
            self._mark_reject()
            self._state = State.SEARCH
            self._target = None
            self._filter = None
            self._is_wingman = False
            self._dwell_time = 0.0
            return self._dispatch(obs, dt)

        # 广播：长机首次进入 TRACK 时 announce，之后定期 T: 位置；
        # 僚机定期 C: 占位（第三机看到槽位满就不再扎进同一目标）
        if self._target and self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            if self._is_wingman:
                cmds.append(self._make_join_claim(self._target[0], self._target[1]))
            elif self._dwell_time <= dt * 2:
                # 长机首次确认：announce（需要僚机）
                cmds.append(self._make_announce(self._target[0], self._target[1]))
            else:
                # 定期位置广播
                cmds.append(self._make_broadcast(self._target[0], self._target[1]))

        # 云台 + 飞行（僚机用错层的更大盘旋半径避免 <200m 惩罚）
        # 云台瞄准用滤波位置（比逐帧检测平滑，减少锁中断，
        # K=3 协同 dwell 需要三机同时连续锁定 20s、中断 >2s 清零）。
        # 长机 100m / 僚机 300m 或 500m 同心盘旋，最近距离 ≥200m
        if self._target:
            aim = self._target
            if self._filter and self._filter.is_initialized():
                aim = self._filter.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                aim[0],
                aim[1], uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
            loiter = self._wingman_loiter if self._is_wingman else _LOITER_RADIUS
            # SAM 推避：目标在防空区内时盘旋中心推出 bbox（宁可放弃该
            # 目标也不进 SAM——alive 维度要求绝不进防空区）
            lat, lon = _push_out_of_sam(
                self._target[0], self._target[1], obs.briefing
            )
            cmds.append(
                fly_to(
                    lat,
                    lon,
                    alt=_SEARCH_ALT,
                    speed=_TRACK_SPEED,
                    loiter_radius=loiter,
                )
            )

            # report_target（仅确认移动目标）
            if (
                self._sim_time - self._last_report_time >= _REPORT_INTERVAL
                and self._filter
                and self._filter.is_initialized()
                and self._filter.speed_mps() > 3.0
            ):
                self._last_report_time = self._sim_time
                est_lat, est_lon = self._filter.position_wgs84()
                cmds.append(report_target(est_lat, est_lon))

        return cmds

    # ── JOIN：收到队友广播，收敛到共享目标 ────────────────────────────────

    def _do_join(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._join_time += dt

        # JOIN 超时（announce 过期或始终未检测到目标）→ SEARCH
        if self._target is None or self._join_time >= _JOIN_TIMEOUT_S:
            self._state = State.SEARCH
            self._target = None
            self._shared_target = None
            self._shared_target_time = -1.0
            self._join_time = 0.0
            return self._dispatch(obs, dt)

        # 占位冲突仲裁：收敛同一目标的僚机超过槽位数时，uid 大者退让回
        # SEARCH（多机同时收到同一 announce 的竞态；C: 心跳 10s 内有效）
        holders = [
            u for u in self._wing_uids_near(*self._target)
            if str(u) < str(self.my_uid)
        ]
        if len(holders) >= _WING_SLOTS:
            self._state = State.SEARCH
            self._target = None
            self._shared_target = None
            self._shared_target_time = -1.0
            self._join_time = 0.0
            return self._dispatch(obs, dt)

        # 占位广播（2Hz）：告诉其他机这个目标已有我一架僚机
        if self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            cmds.append(self._make_join_claim(self._target[0], self._target[1]))

        # 检测不覆盖 _target：announce 位置才是收敛基准（赛题二曾把
        # _target 改成检测位置，检测落在 announce 点 200~300m 外时
        # （目标+诱饵搭档）VERIFY↔JOIN 同拍乒乓直至递归崩溃）。检测只
        # 用于云台瞄准（下方）。

        # 接近到 350m 且有检测 → 僚机直接 TRACK（边跟踪边判别）。
        # 老设计是先过 12s VERIFY 判别，但 announce 已经过长机速度判别
        # （fast-pass/OLS），僚机再走一遍 VERIFY 只是把 K=3 协锁推迟
        # 12s+；直入 TRACK 让协锁 dwell 从到达即开始累计，假阳性由
        # TRACK 内的后台诱饵带 bailout（15s ∈[4.5,5.5)）兜底。
        # 阈值 350m 而非 200m：长机在 100m 盘旋，僚机再近触发
        # <200m proximity 扣分
        dist_to_target = haversine_m(
            obs.self.lat, obs.self.lon, self._target[0], self._target[1]
        )
        if dist_to_target < 350.0 and det.detected:
            self._state = State.TRACK
            self._is_wingman = True
            self._dwell_time = 0.0
            self._track_time = 0.0
            self._last_det_time = self._sim_time
            self._wing_bail_s = 0.0
            self._filter = CvFilter(obs.self.lat, obs.self.lon)
            return self._dispatch(obs, dt)

        # 飞向共享目标（SAM 推避）
        lat, lon = _push_out_of_sam(
            self._target[0], self._target[1], obs.briefing
        )
        cmds.append(
            fly_to(
                lat,
                lon,
                alt=_SEARCH_ALT,
                speed=_SEARCH_SPEED,
                loiter_radius=_LOITER_RADIUS,
            )
        )

        # 云台锁定
        if det.detected and det.target_lat is not None:
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                det.target_lat,
                det.target_lon, uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
        else:
            cmds.append(set_gimbal_fov(_SEARCH_FOV))

        return cmds

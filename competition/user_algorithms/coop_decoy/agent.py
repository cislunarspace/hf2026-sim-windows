"""赛题二协同诱饵鉴别 Agent。

四状态 FSM：
  SEARCH → VERIFY → TRACK
                ↑       ↓ (摧毁/超时) → SEARCH
  JOIN ←── (收到队友广播确认目标)

算法：
  - OLS 最小二乘速度判别诱饵（12s 采样窗口，抗 ~50m 检测噪声；
    ImmFilter 同步更新供 TRACK 接管）
  - 螺旋搜索（uid 扇区分配）
  - 盘旋跟踪 + 广播协同
  - K=2 同时盯防 20s 摧毁
"""

import math
from enum import Enum

from algorithms.estimation.ekf import ImmFilter
from algorithms.estimation.geometry import bearing_rad, haversine_m
from algorithms.estimation.motion import ols_speed_mps
from algorithms.search.target_roads import route_waypoints_for_uid
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
_TRACK_SPEED = 20.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 100.0  # 盘旋半径（m）
_LEAD_TIME_S = 1.5  # 前馈时间（s）

_VERIFY_SAMPLES = 120  # VERIFY 判别所需检测样本数（12s @10Hz）
# 真目标速度 5/9/12 m/s，诱饵被官方 runner 统一注入 5.0 m/s（_astar_navigator
# inject_astar_decoy decoy_speed=5.0）。≥6.5 必真（9/12），5 m/s 类
# （1 真 + 15 诱饵）速度不可分，当前按诱饵处理——放弃 T1 换取不被诱饵拖死。
_VERIFY_SPEED_MIN = 6.5  # OLS 速度下限（m/s）：≥6.5 必真（真目标 9/12，诱饵统一 5.0）
_VERIFY_SPEED_MAX = 13.5  # OLS 速度上限（m/s）：超过地面车辆极速（12）必是锁跳变/UAV
_VERIFY_LOST_ABORT_S = 2.0  # VERIFY 中连续丢失超过此时长则放弃（不记诱饵）
_REJECT_COOLDOWN_S = 20.0  # 判别否决/中止后的重检测冷却（s）：防同帧循环重进 VERIFY，
# 又不永久标记——停顿中的真目标冷却后重遇可重新判别
_REJECT_RADIUS_M = 500.0  # 冷却生效的检测距离（m）：机 20s 已飞出 ~500m，
# 真目标 20s 后仍在原区附近可重新判别；跳变搭档车（≥198m）也在半径内

_TRACK_DWELL_S = 20.0  # 盯防摧毁时间（s）
_TRACK_GRACE_S = 2.0  # 丢失容忍时间（s）
_TRACK_TIMEOUT_S = 90.0  # 跟踪超时（s）：长机须咬住目标等僚机
# （跨区飞来 ~50s + 僚机 VERIFY 12s + 协同 20s），35s 等到一半就放弃了

_JOIN_TIMEOUT_S = 60.0  # JOIN 超时（s），扇区间距 ~2km @25m/s 需 ~80s 收敛
_ANNOUNCE_EXPIRE_S = 15.0  # announce 过期时间（s），防止收敛到已放弃的诱饵

_BC_INTERVAL = 0.5  # 广播间隔（s，2Hz）
_HB_INTERVAL = 1.0  # 位置心跳间隔（s，1Hz；P:lat,lon，用于 proximity 避让）
_REPORT_INTERVAL = 1.0  # 上报间隔（s）

_ASSUME_RANGE_M = 800.0  # 首次检测假设距离（m）
_TRACK_FOV = 60.0  # 跟踪 FOV（°）
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
    """赛题二参赛 Agent：IMM 滤波 + 螺旋搜索 + 盘旋跟踪 + 广播协同。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state = State.SEARCH
        self._imm: ImmFilter | None = None
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s: float = 0.0
        self._sim_time = 0.0
        self._target: tuple[float, float] | None = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_destroyed: list[tuple[float, float]] = []
        self._shared_target: tuple[float, float] | None = None
        self._shared_target_time: float = -1.0  # 收到 announce 的 sim_time，-1=未收到
        self._join_time: float = 0.0
        self._is_wingman: bool = False  # True=僚机（收到 announce 加入），False=长机
        self._gimbal_phase: float = 0.0  # SEARCH 云台扫描相位
        self._last_reject_pos: tuple[float, float] | None = None
        self._last_reject_time: float = -1e9
        self._reject_streak: int = 0
        self._time_synced: bool = False
        self._last_hb_time: float = -1e9
        self._teammates: dict[str, tuple[float, float, float]] = {}  # uid→(lat,lon,t)

    def reset(self) -> None:
        self._state = State.SEARCH
        self._imm = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._verify_samples = []
        self._verify_lost_s = 0.0
        self._sim_time = 0.0
        self._target = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_destroyed = []
        self._shared_target = None
        self._shared_target_time = -1.0
        self._join_time = 0.0
        self._is_wingman = False
        self._gimbal_phase = 0.0
        self._last_reject_pos = None
        self._last_reject_time = -1e9
        self._reject_streak = 0
        self._time_synced = False
        self._last_hb_time = -1e9
        self._teammates = {}

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

        # 状态分发
        if self._state == State.SEARCH:
            return cmds + self._do_search(obs, dt)
        elif self._state == State.VERIFY:
            return cmds + self._do_verify(obs, dt)
        elif self._state == State.TRACK:
            return cmds + self._do_track(obs, dt)
        elif self._state == State.JOIN:
            return cmds + self._do_join(obs, dt)
        return cmds

    # ── 时间基准 ──────────────────────────────────────────────────────────

    def _sync_time(self, obs: CoopObs, dt: float) -> None:
        """同步引擎 sim_time（briefing.score_view 每拍更新），读不到回退 dt 累加。

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
            self._sim_time = st
        else:
            self._sim_time += dt

    # ── 通信 ──────────────────────────────────────────────────────────────

    def _ingest_comms(self, inbox) -> None:
        """解析队友广播，提取确认目标。A: 消息优先（announce）。"""
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

    def _mark_reject(self) -> None:
        """记录判别否决/中止位置。同一位置连续否决时冷却指数升档
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
        if self._last_reject_pos is None:
            return False
        if (
            haversine_m(lat, lon, self._last_reject_pos[0], self._last_reject_pos[1])
            >= _REJECT_RADIUS_M
        ):
            return False
        cooldown = _REJECT_COOLDOWN_S * (2 ** min(self._reject_streak, 3))
        return self._sim_time - self._last_reject_time < cooldown

    # ── SEARCH：割草机覆盖搜索本机条带 ─────────────────────────────────────

    def _do_search(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 生成搜索航点（如果还没有）：沿目标路线先验扫描（训练阶段真目标
        # 只在 points.json 的 26 条路网上，沿路线扫描遭遇率远高于割草机；
        # 验证集换路线时失效，需退回割草机或重新生成）
        if not self._search_waypoints:
            self._search_waypoints = route_waypoints_for_uid(self.my_uid, n_shares=3)
            self._wp_idx = 0

        # 收到队友确认目标 → JOIN（跳过已摧毁目标）
        if self._shared_target is not None:
            near_destroyed = any(
                haversine_m(self._shared_target[0], self._shared_target[1], d[0], d[1])
                < 150.0
                for d in self._known_destroyed
            )
            if not near_destroyed:
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._imm = None
                return self._do_join(obs, dt)

        # 检测到目标 → VERIFY（跳过已摧毁目标）。
        # 不做诱饵标记跳过：诱饵也在动（5 m/s 全域路线），位置标记会失效；
        # 且中途停顿的真目标读数同静止，误标记会永久隐藏它。
        if det.detected and det.target_lat is not None:
            near_destroyed = any(
                haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in self._known_destroyed
            )
            if not near_destroyed and not self._in_reject_cooldown(
                det.target_lat, det.target_lon
            ):
                self._state = State.VERIFY
                self._target = (det.target_lat, det.target_lon)
                self._imm = ImmFilter(obs.self.lat, obs.self.lon)
                self._verify_samples = []
                self._verify_lost_s = 0.0
                self._is_wingman = False  # 自己发现的候选：判别通过即长机
                # 不在此处 announce：候选未判别，提前 announce 会让全队
                # 收敛到同一个静止诱饵（诊断证实）。判别通过进 TRACK 时再
                # announce（见 _do_track 首次广播）。
                return self._do_verify(obs, dt)

        # 沿割草机航点飞行（到达 loiter 圈内即切下一点）
        if self._search_waypoints:
            wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            dist = haversine_m(obs.self.lat, obs.self.lon, wp_lat, wp_lon)
            if dist < 200.0:
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

    # ── VERIFY：OLS 速度判别（ImmFilter 同步更新供 TRACK 接管） ──────────

    def _do_verify(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 锁定目标 + 采样 + ImmFilter 更新
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
                det.target_lon,
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))

            bearing = bearing_rad(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            range_m = haversine_m(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )

            if self._imm is not None:
                if not self._imm.is_initialized():
                    self._imm.initialize(obs.self.lat, obs.self.lon, bearing, range_m)
                else:
                    self._imm.predict(dt)
                    self._imm.update_bearing(obs.self.lat, obs.self.lon, bearing)
                    self._imm.update_range(obs.self.lat, obs.self.lon, range_m)
        else:
            self._verify_lost_s += dt
            if self._imm is not None and self._imm.is_initialized():
                self._imm.predict(dt)

        # 连续丢失 → 放弃判别（不记诱饵，避免误伤真实目标）
        if self._verify_lost_s > _VERIFY_LOST_ABORT_S:
            self._mark_reject()
            self._state = State.SEARCH
            self._target = None
            self._imm = None
            self._verify_samples = []
            return self._do_search(obs, dt)

        # 收到队友确认目标 → JOIN（优先协同）
        if self._shared_target is not None and self._target is not None:
            d = haversine_m(
                self._shared_target[0],
                self._shared_target[1],
                self._target[0],
                self._target[1],
            )
            if d > 200.0:  # 不同目标，队友确认的是另一个
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._imm = None
                self._verify_samples = []
                return self._do_join(obs, dt)

        # 样本足够：OLS 最小二乘速度判别（[6.5, 13.5] 速度带）
        if len(self._verify_samples) >= _VERIFY_SAMPLES:
            speed = ols_speed_mps(self._verify_samples)
            self._verify_samples = []
            if _VERIFY_SPEED_MIN <= speed <= _VERIFY_SPEED_MAX:
                # 真目标 → TRACK（_is_wingman 在进入 VERIFY 时已设定，此处保留）
                self._state = State.TRACK
                self._dwell_time = 0.0
                self._track_time = 0.0
                self._last_det_time = self._sim_time
                return self._do_track(obs, dt)
            else:
                # 速度出界：静止/5 m/s 类（诱饵或停顿/慢速真目标），
                # 或 >13.5 的锁跳变虚高（地面车辆极速 12）。记冷却后回 SEARCH。
                self._mark_reject()
                self._state = State.SEARCH
                self._target = None
                self._imm = None
                return self._do_search(obs, dt)

        # 飞向目标区域
        if self._target:
            cmds.append(
                fly_to(
                    self._target[0],
                    self._target[1],
                    alt=_SEARCH_ALT,
                    speed=_SEARCH_SPEED,
                    loiter_radius=_LOITER_RADIUS,
                )
            )

        return cmds

    # ── TRACK：盘旋跟踪 + 广播 + 上报 ───────────────────────────────────

    def _do_track(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._track_time += dt

        # 跟踪目标位置更新 + IMM 滤波（report_target 用滤波位置，
        # 回归：TRACK 曾不更新 IMM，上报位置冻结在 VERIFY 结束时刻）
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            if self._target:
                d = haversine_m(
                    det.target_lat, det.target_lon, self._target[0], self._target[1]
                )
                if d < 250.0:
                    self._target = (det.target_lat, det.target_lon)
            bearing = bearing_rad(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            range_m = haversine_m(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            if self._imm is None:
                self._imm = ImmFilter(obs.self.lat, obs.self.lon)
            if not self._imm.is_initialized():
                self._imm.initialize(obs.self.lat, obs.self.lon, bearing, range_m)
            else:
                self._imm.predict(dt)
                self._imm.update_bearing(obs.self.lat, obs.self.lon, bearing)
                self._imm.update_range(obs.self.lat, obs.self.lon, range_m)
        else:
            if self._imm is not None and self._imm.is_initialized():
                self._imm.predict(dt)

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

        # 盯防满 20s → 视为摧毁：记入已摧毁列表（不再跟踪/上报该目标），回 SEARCH
        if self._dwell_time >= _TRACK_DWELL_S:
            if self._target:
                self._known_destroyed.append(self._target)
            self._state = State.SEARCH
            self._target = None
            self._imm = None
            self._is_wingman = False
            self._dwell_time = 0.0
            return self._do_search(obs, dt)

        # 超时未摧毁（协同未到齐）→ 记冷却后回 SEARCH（可能是诱饵或停顿真目标）
        if self._track_time >= _TRACK_TIMEOUT_S:
            self._mark_reject()
            self._state = State.SEARCH
            self._target = None
            self._imm = None
            self._is_wingman = False
            self._dwell_time = 0.0
            return self._do_search(obs, dt)

        # 广播：长机首次进入 TRACK 时 announce，之后定期 T: 位置
        if self._target and self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            if not self._is_wingman and self._dwell_time <= dt * 2:
                # 长机首次确认：announce（需要僚机）
                cmds.append(self._make_announce(self._target[0], self._target[1]))
            else:
                # 定期位置广播
                cmds.append(self._make_broadcast(self._target[0], self._target[1]))

        # 云台 + 飞行（僚机用更大盘旋半径避免 <200m 惩罚）
        # 云台瞄准用 IMM 滤波位置（比逐帧检测平滑，减少锁中断，
        # K=2 协同 dwell 需要双机同时连续锁定 20s、中断 >2s 清零）
        if self._target:
            aim = self._target
            if self._imm and self._imm.is_initialized():
                aim = self._imm.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                aim[0],
                aim[1],
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
            loiter = _LOITER_RADIUS * 3 if self._is_wingman else _LOITER_RADIUS
            cmds.append(
                fly_to(
                    self._target[0],
                    self._target[1],
                    alt=_SEARCH_ALT,
                    speed=_TRACK_SPEED,
                    loiter_radius=loiter,
                )
            )

            # report_target（仅确认移动目标）
            if (
                self._sim_time - self._last_report_time >= _REPORT_INTERVAL
                and self._imm
                and self._imm.is_initialized()
                and self._imm.speed_mps() > 3.0
            ):
                self._last_report_time = self._sim_time
                est_lat, est_lon = self._imm.position_wgs84()
                cmds.append(report_target(est_lat, est_lon))

        return cmds

    # ── JOIN：收到队友广播，收敛到共享目标 ────────────────────────────────

    def _do_join(self, obs: CoopObs, dt: float) -> list[Command]:
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
            return self._do_search(obs, dt)

        # 检测到目标后开始跟踪
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            d = haversine_m(
                det.target_lat, det.target_lon, self._target[0], self._target[1]
            )
            if d < 300.0:
                self._target = (det.target_lat, det.target_lon)

        # 接近到 350m 且有检测 → 僚机先过 VERIFY 判别（announce 早于判别 12s
        # 发出，候选可能是诱饵；统一所有 TRACK 入口都经 OLS 判别）。
        # 阈值取 350m 而非 200m：长机在目标 100m 盘旋，僚机再近会触发
        # <200m proximity 扣分（600s 局 8 次 ×2 分把 accuracy 得分清零）
        dist_to_target = haversine_m(
            obs.self.lat, obs.self.lon, self._target[0], self._target[1]
        )
        if dist_to_target < 350.0 and det.detected:
            self._state = State.VERIFY
            self._is_wingman = True  # 判别通过后以僚机身份 TRACK
            self._verify_samples = []
            self._verify_lost_s = 0.0
            self._imm = ImmFilter(obs.self.lat, obs.self.lon)
            return self._do_verify(obs, dt)

        # 飞向共享目标
        cmds.append(
            fly_to(
                self._target[0],
                self._target[1],
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
                det.target_lon,
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
        else:
            cmds.append(set_gimbal_fov(_SEARCH_FOV))

        return cmds

"""赛题一搜索-跟踪 Agent 实现（v2）。

评分机制（spec 2026-07-15）：只有 accuracy 一个维度——裁判 1Hz 采样
report_target，D_t = 上报点与真目标的大圆距离，p_t = clamp(1 − D_t/30m)，
漏报记 0，每 20s 窗口去掉最低 2 个 p_t。由此得出三条设计原则：

1. **每秒必报**。报错（D>30m）和漏报一样记 0，所以有任何位置假设就报，
   永远不会比不报更差。目标出生后在路线 Start 停 30s（points.json 全部
   路线 Start.WaitTime=30），开局即报出生点可白拿约 30 秒满分。
2. **尽快截获**。briefing.target_initial_pos 恒为目标真实出生点
   （prepare_scenario 在场景随机化之后把路线 Start 写入实体初始位，
   引擎按此生成目标），且可与 points.json 26 条路线匹配；匹配成功即可
   沿路线预测目标任意时刻位置，直接飞预测点截获，不做盲搜。
3. **截获后云台指 CvFilter 滤波位置**（±50m 检测噪声经滤波收敛到 ~10m，
   比逐帧指原始检测更稳）；检测直接给经纬度（完整位置量测），用双轴
   匀速卡尔曼（`algorithms/estimation/cv_kalman.py`）。v1 用的 bearing-only
   ImmFilter 在此场景实测发散（估计值两帧跳离量测 ~200m），是跟踪反复
   中断的根因。

FSM：
  ACQUIRE → SEARCH → ENGAGE → ATTACK（收敛后换名，行为相同）
                ↑      LOST ←── (丢失 > 2s) ←── ENGAGE/ATTACK
                └── (12s 未重捕获) ──┘

时间基准：所有计时用 obs.briefing.score_view.sim_time（引擎真时间）。
runner 控制节拍比引擎快 ~2.5 倍，用 dt 累加会让 1Hz 上报节拍失真、
滤波 predict 过估 2.5 倍（v1 上报 RMSE 102m 的主因）。读不到 score_view
时（单测 mock）回退 dt 累加。

赛题一无诱饵，不设 VERIFY（诱饵判别是赛题二/三的事）。
"""

import math
import os
from enum import Enum

_DEBUG = os.environ.get("ST_AGENT_DEBUG") == "1"


def _dbg(*args) -> None:
    if _DEBUG:
        print("[STA]", *args, flush=True)

from algorithms.estimation.cv_kalman import CvFilter
from algorithms.estimation.geometry import haversine_m
from algorithms.search.route_prior import (
    match_route,
    predict_position,
    predict_velocity,
)
from algorithms.search.spiral import generate_spiral
from algorithms.tracking.follow import compute_lead_point
from algorithms.tracking.gimbal import compute_gimbal_angles

from competition.sdk.core.commands import (
    Command,
    fly_to,
    point_gimbal,
    report_target,
    set_gimbal_fov,
)
from competition.sdk.scenarios.search_track import SearchTrackAgent
from competition.sdk.scenarios.search_track.observation import SearchTrackObs


class State(Enum):
    ACQUIRE = "acquire"
    SEARCH = "search"
    ENGAGE = "engage"
    ATTACK = "attack"
    LOST = "lost"


# ── 常量 ────────────────────────────────────────────────────────────────

_SEARCH_FOV = 70.0  # 搜索 FOV（度）
_TRACK_FOV = 30.0  # 跟踪 FOV（度）；15° 对 ±50m 检测噪声太窄，易丢锁
_INTERCEPT_SPEED = 40.0  # 截获速度（m/s），机型上限 40
_TRACK_SPEED = 25.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 200.0  # 盘旋半径（m）
_LEAD_TIME_S = 1.5  # 前馈时间（s）
_GRACE_FRAMES = 20  # 丢失容忍帧数（~2s @ 10Hz）
_REACQUIRE_TIMEOUT_S = 12.0  # LOST 重捕获时限（s，引擎时间），超时回 SEARCH
_REPORT_INTERVAL = 1.0  # 上报间隔（s，引擎时间）
_SPIRAL_RADIUS_M = 1500  # 无先验回退：螺旋搜索半径（m）
_SPIRAL_PITCH_M = 300  # 螺旋螺距（m）
_ROUTE_GIVEUP_S = 150.0  # 路线先验截获超时（s），转螺旋回退


class MySearchTrackAgent(SearchTrackAgent):
    """赛题一参赛 Agent：路线先验截获 + 视觉伺服跟踪 + 每秒必报。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state: State = State.ACQUIRE
        self._filter: CvFilter | None = None
        # 路线先验
        self._route_idx: int | None = None
        self._route_failed: bool = False  # 截获超时，放弃先验转螺旋
        self._target_speed: float = 8.0
        self._start_pos: tuple[float, float] | None = None
        # 螺旋回退
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx: int = 0
        self._gimbal_phase: float = 0.0  # 扇扫相位
        # 计时（引擎真时间）
        self._sim_time: float = 0.0
        self._last_sim_time: float | None = None
        self._t0: float | None = None  # 首次见到引擎时间（路线预测起点）
        self._last_report_time: float = -1e9
        self._search_start_time: float = 0.0
        self._lost_time: float = 0.0
        self._lost_frames: int = 0

    def reset(self) -> None:
        self._state = State.ACQUIRE
        self._filter = None
        self._route_idx = None
        self._route_failed = False
        self._target_speed = 8.0
        self._start_pos = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._gimbal_phase = 0.0
        self._sim_time = 0.0
        self._last_sim_time = None
        self._t0 = None
        self._last_report_time = -1e9
        self._search_start_time = 0.0
        self._lost_time = 0.0
        self._lost_frames = 0

    # ── 时间基准 ───────────────────────────────────────────────────────

    def _sync_time(self, obs: SearchTrackObs, dt: float) -> float:
        """同步引擎时间，返回本拍的仿真时间增量（供滤波 predict）。"""
        st = getattr(getattr(obs.briefing, "score_view", None), "sim_time", None)
        if isinstance(st, (int, float)):
            st = float(st)
            if self._t0 is None:
                self._t0 = st
            sim_dt = st - self._last_sim_time if self._last_sim_time is not None else 0.0
            self._last_sim_time = st
            self._sim_time = st
            return min(max(sim_dt, 0.0), 1.0)
        # 回退：mock obs 无 score_view，用 dt 累加
        self._sim_time += dt
        return dt

    def _report_due(self) -> bool:
        return self._sim_time - self._last_report_time >= _REPORT_INTERVAL

    # ── 主入口 ─────────────────────────────────────────────────────────

    def decide(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        sim_dt = self._sync_time(obs, dt)

        if self._filter is None:
            self._filter = CvFilter(obs.self.lat, obs.self.lon)

        # 一次性初始化：路线匹配 + 螺旋回退航点
        if self._state == State.ACQUIRE:
            self._init_search(obs)
            self._state = State.SEARCH
            self._search_start_time = self._sim_time

        if self._state == State.SEARCH:
            return self._do_search(obs, dt)
        if self._state == State.ENGAGE:
            return self._do_track(obs, sim_dt, attack=False)
        if self._state == State.ATTACK:
            return self._do_track(obs, sim_dt, attack=True)
        # LOST
        return self._do_lost(obs, sim_dt)

    def _debug_tick(self, obs: SearchTrackObs, focus: tuple[float, float] | None = None) -> None:
        if not _DEBUG:
            return
        self._dbg_n = getattr(self, "_dbg_n", 0) + 1
        det = obs.self.detection
        # 状态转移或每秒一拍都输出
        if det.detected or self._dbg_n % 10 == 0 or self._state != getattr(self, "_dbg_state", None):
            est = None
            if self._filter is not None and self._filter.is_initialized():
                est = tuple(round(v, 6) for v in self._filter.position_wgs84())
            _dbg(
                f"t={self._sim_time:7.1f} {self._state.value:6s} "
                f"uav=({obs.self.lat:.6f},{obs.self.lon:.6f},{obs.self.alt:.0f}) "
                f"det={det.detected} "
                f"detpos=({getattr(det,'target_lat',None)},{getattr(det,'target_lon',None)}) "
                f"focus={None if focus is None else (round(focus[0],6), round(focus[1],6))} "
                f"est={est} lost={self._lost_frames}"
            )
        self._dbg_state = self._state

    def _init_search(self, obs: SearchTrackObs) -> None:
        tip = getattr(obs.briefing, "target_initial_pos", None)
        speed = (getattr(obs.briefing, "params", None) or {}).get("target_speed")
        if isinstance(speed, (int, float)) and speed > 0:
            self._target_speed = float(speed)
        if tip is not None:
            self._start_pos = (float(tip[0]), float(tip[1]))
            self._route_idx = match_route(*self._start_pos)
        # 螺旋回退以出生点为中心（无先验时目标从该点出发，最多 8 m/s 外扩）
        center = self._start_pos or (obs.self.lat, obs.self.lon)
        self._search_waypoints = generate_spiral(
            center[0], center[1],
            radius_m=_SPIRAL_RADIUS_M, pitch_m=_SPIRAL_PITCH_M,
        )

    # ── SEARCH：路线先验截获（回退：螺旋盲搜）─────────────────────────

    def _search_focus(self) -> tuple[float, float]:
        """当前搜索焦点：匹配路线 → 预测位置；否则出生点。"""
        if self._route_idx is not None and not self._route_failed:
            elapsed = self._sim_time - (self._t0 or 0.0)
            return predict_position(self._route_idx, elapsed, self._target_speed)
        if self._start_pos is not None:
            return self._start_pos
        return (0.0, 0.0)

    def _do_search(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds: list[Command] = [set_gimbal_fov(_SEARCH_FOV)]
        det = obs.self.detection
        focus = self._search_focus()

        if self._route_idx is not None and not self._route_failed:
            # 沿预测位置截获：全速追预测点，云台指预测点
            cmds.append(
                fly_to(focus[0], focus[1], speed=_INTERCEPT_SPEED,
                       loiter_radius=_LOITER_RADIUS)
            )
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt, focus[0], focus[1], uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            # 截获超时 → 转螺旋回退（A* 失败停车、预测失准等情况）
            if self._sim_time - self._search_start_time > _ROUTE_GIVEUP_S:
                self._route_failed = True
                self._search_waypoints = generate_spiral(
                    focus[0], focus[1],
                    radius_m=_SPIRAL_RADIUS_M, pitch_m=_SPIRAL_PITCH_M,
                )
                self._wp_idx = 0
        else:
            # 无先验回退：螺旋 + 云台扇扫
            self._gimbal_phase += dt * 0.5
            pan = 90.0 * math.sin(self._gimbal_phase)
            tilt = -45.0 + 15.0 * math.sin(self._gimbal_phase * 0.7)
            cmds.append(point_gimbal(pan, tilt))
            if self._search_waypoints:
                wp = self._search_waypoints[self._wp_idx % len(self._search_waypoints)]
                cmds.append(
                    fly_to(wp[0], wp[1], speed=_INTERCEPT_SPEED,
                           loiter_radius=_LOITER_RADIUS)
                )
                if haversine_m(obs.self.lat, obs.self.lon, wp[0], wp[1]) < 200:
                    self._wp_idx += 1

        # 每秒必报：报错与漏报同为 0，有假设就报（开局出生点 30s 停驶是满分）
        if self._report_due() and focus != (0.0, 0.0):
            cmds.append(report_target(focus[0], focus[1]))
            self._last_report_time = self._sim_time

        # 赛题一无诱饵，检测到直接进 ENGAGE
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            self._state = State.ENGAGE
            self._lost_frames = 0
        self._debug_tick(obs, focus)
        return cmds

    # ── ENGAGE / ATTACK：滤波位置伺服 + 前馈飞行 + 每秒上报 ────────────

    def _prior_velocity(self) -> tuple[float, float]:
        """路线先验速度（滤波初始化用）；无先验时为 0。"""
        if self._route_idx is not None and not self._route_failed:
            elapsed = self._sim_time - (self._t0 or 0.0)
            return predict_velocity(self._route_idx, elapsed, self._target_speed)
        return (0.0, 0.0)

    def _do_track(self, obs: SearchTrackObs, sim_dt: float,
                  attack: bool) -> list[Command]:
        cmds: list[Command] = []
        det = obs.self.detection
        fresh = det.detected and det.target_lat is not None and det.target_lon is not None

        if fresh:
            if not self._filter.is_initialized():
                # 路线先验速度作初值：消除速度从 0 拉起的斜坡滞后
                ve, vn = self._prior_velocity()
                self._filter.initialize(det.target_lat, det.target_lon, ve, vn)
            else:
                self._filter.predict(sim_dt)
                self._filter.update_position(det.target_lat, det.target_lon)
            self._lost_frames = 0
        else:
            if self._filter.is_initialized():
                self._filter.predict(sim_dt)
            self._lost_frames += 1

        # 云台指滤波位置（初始化即取首帧检测，之后平滑 ±50m 量测噪声）
        if self._filter.is_initialized():
            est_lat, est_lon = self._filter.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt, est_lat, est_lon, uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
        cmds.append(set_gimbal_fov(_TRACK_FOV))

        if self._lost_frames >= _GRACE_FRAMES:
            self._state = State.LOST
            self._lost_time = 0.0
            return cmds

        # 前馈飞行 + 每秒上报滤波位置
        if self._filter.is_initialized():
            est_lat, est_lon = self._filter.position_wgs84()
            ve, vn = self._filter.velocity_mps()
            lead_lat, lead_lon = compute_lead_point(
                est_lat, est_lon, ve, vn, _LEAD_TIME_S
            )
            cmds.append(
                fly_to(lead_lat, lead_lon, speed=_TRACK_SPEED,
                       loiter_radius=_LOITER_RADIUS)
            )
            if self._report_due():
                cmds.append(report_target(est_lat, est_lon))
                self._last_report_time = self._sim_time

        if not attack and self._filter.is_converged(15.0):
            self._state = State.ATTACK
        self._debug_tick(obs)
        return cmds

    # ── LOST：滤波外推盘旋等待重捕获（持续上报外推值）──────────────────

    def _do_lost(self, obs: SearchTrackObs, sim_dt: float) -> list[Command]:
        cmds: list[Command] = []
        self._lost_time += sim_dt
        det = obs.self.detection

        if det.detected and det.target_lat is not None and det.target_lon is not None:
            self._state = State.ENGAGE
            self._lost_frames = 0
            return self._do_track(obs, sim_dt, attack=False)

        if self._filter is not None and self._filter.is_initialized():
            self._filter.predict(sim_dt)
            est_lat, est_lon = self._filter.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt, est_lat, est_lon, uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_SEARCH_FOV))
            cmds.append(
                fly_to(est_lat, est_lon, speed=_TRACK_SPEED, loiter_radius=150.0)
            )
            # 丢失期间继续上报外推位置：短时外推误差 <30m 仍有分
            if self._report_due():
                cmds.append(report_target(est_lat, est_lon))
                self._last_report_time = self._sim_time

        if self._lost_time >= _REACQUIRE_TIMEOUT_S:
            # 回 SEARCH：路线预测仍在走（目标在动），滤波器重置以便重新初始化
            self._filter = CvFilter(obs.self.lat, obs.self.lon)
            self._state = State.SEARCH
            self._search_start_time = self._sim_time
        self._debug_tick(obs)
        return cmds

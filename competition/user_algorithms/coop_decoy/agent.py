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

from enum import Enum

from algorithms.estimation.ekf import ImmFilter
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
from competition.sdk.scenarios.coop_decoy import CoopAgent
from competition.sdk.scenarios.coop_decoy.observation import CoopObs

# ── 常量 ──────────────────────────────────────────────────────────────────

_SEARCH_ALT = 200.0  # 搜索高度（m）
_SEARCH_SPEED = 25.0  # 搜索速度（m/s）
_TRACK_SPEED = 20.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 100.0  # 盘旋半径（m）
_LEAD_TIME_S = 1.5  # 前馈时间（s）

_VERIFY_SAMPLES = 120  # VERIFY 判别所需检测样本数（12s @10Hz）
_VERIFY_SPEED_THRESH = 3.5  # OLS 速度阈值（m/s）：12s 窗口静止诱饵误判 ~3%、5m/s 召回 ~90%
_VERIFY_LOST_ABORT_S = 2.0  # VERIFY 中连续丢失超过此时长则放弃（不记诱饵）

_TRACK_DWELL_S = 20.0  # 盯防摧毁时间（s）
_TRACK_GRACE_S = 2.0  # 丢失容忍时间（s）
_TRACK_TIMEOUT_S = 35.0  # 跟踪超时（s）

_JOIN_TIMEOUT_S = 60.0  # JOIN 超时（s），扇区间距 ~2km @25m/s 需 ~80s 收敛
_ANNOUNCE_EXPIRE_S = 15.0  # announce 过期时间（s），防止收敛到已放弃的诱饵

_BC_INTERVAL = 0.5  # 广播间隔（s，2Hz）
_REPORT_INTERVAL = 1.0  # 上报间隔（s）

_SPIRAL_RADIUS_M = 700.0  # 扇区螺旋半径（m）
_SPIRAL_PITCH_M = 200.0  # 螺旋螺距（m）
_ASSUME_RANGE_M = 800.0  # 首次检测假设距离（m）
_TRACK_FOV = 60.0  # 跟踪 FOV（°）
_SEARCH_FOV = 60.0  # 搜索 FOV（°）

# 赛题二场景 bbox（北京附近海域）
_BBOX: tuple[tuple[float, float], tuple[float, float]] = (
    (26.982, 124.980),
    (27.025, 125.020),
)


# ── 工具函数 ──────────────────────────────────────────────────────────────


def _uid_sector(uid: str, n_sectors: int = 3) -> tuple[float, float]:
    """uid 映射到扇区中心 (lat, lon)。"""
    (lat_min, lon_min), (lat_max, lon_max) = _BBOX
    lat_mid = (lat_min + lat_max) / 2
    sub_w = (lon_max - lon_min) / n_sectors
    if uid.isdigit():
        idx = int(uid) % n_sectors
    elif "_" in uid:
        tail = uid.rsplit("_", 1)[-1]
        idx = int(tail) % n_sectors if tail.isdigit() else 0
    else:
        idx = 0
    return (lat_mid, lon_min + sub_w * (idx + 0.5))


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
        self._sector_center: tuple[float, float] = (0.0, 0.0)
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s: float = 0.0
        self._sim_time = 0.0
        self._target: tuple[float, float] | None = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_decoys: list[tuple[float, float]] = []
        self._known_destroyed: list[tuple[float, float]] = []
        self._shared_target: tuple[float, float] | None = None
        self._shared_target_time: float = -1.0  # 收到 announce 的 sim_time，-1=未收到
        self._join_time: float = 0.0
        self._is_wingman: bool = False  # True=僚机（收到 announce 加入），False=长机

    def reset(self) -> None:
        self._state = State.SEARCH
        self._imm = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._sector_center = _uid_sector(self.my_uid)
        self._verify_samples = []
        self._verify_lost_s = 0.0
        self._sim_time = 0.0
        self._target = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._known_decoys = []
        self._known_destroyed = []
        self._shared_target = None
        self._shared_target_time = -1.0
        self._join_time = 0.0
        self._is_wingman = False

    def decide(self, obs: CoopObs, dt: float) -> list[Command]:
        self._sim_time += dt
        cmds: list[Command] = []

        # 处理队友消息 + 过期清理
        self._ingest_comms(obs.comm_inbox)
        self._expire_shared_target()

        # 状态分发
        if self._state == State.SEARCH:
            return self._do_search(obs, dt)
        elif self._state == State.VERIFY:
            return self._do_verify(obs, dt)
        elif self._state == State.TRACK:
            return self._do_track(obs, dt)
        elif self._state == State.JOIN:
            return self._do_join(obs, dt)
        return cmds

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

    # ── SEARCH：螺旋搜索本扇区 ───────────────────────────────────────────

    def _do_search(self, obs: CoopObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 生成搜索航点（如果还没有）
        if not self._search_waypoints:
            center_lat, center_lon = self._sector_center
            self._search_waypoints = generate_spiral(
                center_lat,
                center_lon,
                radius_m=_SPIRAL_RADIUS_M,
                pitch_m=_SPIRAL_PITCH_M,
            )
            self._wp_idx = 0

        # 收到队友确认目标 → JOIN
        if self._shared_target is not None:
            near_decoy = any(
                haversine_m(self._shared_target[0], self._shared_target[1], d[0], d[1])
                < 150.0
                for d in self._known_decoys
            )
            near_destroyed = any(
                haversine_m(self._shared_target[0], self._shared_target[1], d[0], d[1])
                < 150.0
                for d in self._known_destroyed
            )
            if not near_decoy and not near_destroyed:
                self._state = State.JOIN
                self._target = self._shared_target
                self._join_time = 0.0
                self._imm = None
                return self._do_join(obs, dt)

        # 检测到目标 → VERIFY（跳过已知诱饵与已摧毁目标）
        if det.detected and det.target_lat is not None:
            near_decoy = any(
                haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in self._known_decoys
            )
            near_destroyed = any(
                haversine_m(det.target_lat, det.target_lon, d[0], d[1]) < 150.0
                for d in self._known_destroyed
            )
            if not near_decoy and not near_destroyed:
                self._state = State.VERIFY
                self._target = (det.target_lat, det.target_lon)
                self._imm = ImmFilter(obs.self.lat, obs.self.lon)
                self._verify_samples = []
                self._verify_lost_s = 0.0
                # 早期 announce：通知队友我正在验证此目标
                cmds.append(self._make_announce(det.target_lat, det.target_lon))
                return self._do_verify(obs, dt)

        # 沿螺旋航点飞行
        if self._search_waypoints:
            wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            dist = haversine_m(obs.self.lat, obs.self.lon, wp_lat, wp_lon)
            if dist < 50.0:
                self._wp_idx = (self._wp_idx + 1) % len(self._search_waypoints)
                wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            cmds.append(fly_to(wp_lat, wp_lon, alt=_SEARCH_ALT, speed=_SEARCH_SPEED))

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

        # 样本足够：OLS 最小二乘速度判别
        if len(self._verify_samples) >= _VERIFY_SAMPLES:
            speed = ols_speed_mps(self._verify_samples)
            self._verify_samples = []
            if speed >= _VERIFY_SPEED_THRESH:
                # 真目标 → TRACK（长机）
                self._state = State.TRACK
                self._is_wingman = False
                self._dwell_time = 0.0
                self._track_time = 0.0
                self._last_det_time = self._sim_time
                return self._do_track(obs, dt)
            else:
                # 诱饵 → 回 SEARCH
                if self._target:
                    self._known_decoys.append(self._target)
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

        # 跟踪超时未摧毁 → 回 SEARCH；低速目标疑似误判（真目标判成诱饵反向情况），记为诱饵
        if self._track_time >= _TRACK_TIMEOUT_S:
            if (
                self._target
                and self._imm
                and self._imm.is_initialized()
                and self._imm.speed_mps() < 2.5
            ):
                self._known_decoys.append(self._target)
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
        if self._target:
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                self._target[0],
                self._target[1],
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

        # 到达目标附近 → 切换到 TRACK
        dist_to_target = haversine_m(
            obs.self.lat, obs.self.lon, self._target[0], self._target[1]
        )
        if dist_to_target < 200.0 and det.detected:
            self._state = State.TRACK
            self._is_wingman = True  # 僚机（通过 JOIN 加入）
            self._dwell_time = 0.0
            self._track_time = 0.0
            self._last_det_time = self._sim_time
            self._imm = ImmFilter(obs.self.lat, obs.self.lon)
            return self._do_track(obs, dt)

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

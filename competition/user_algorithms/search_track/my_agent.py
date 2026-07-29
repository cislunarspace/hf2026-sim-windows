"""赛题一搜索-跟踪 Agent 实现。

六状态 FSM：
  ACQUIRE → SEARCH → VERIFY → ENGAGE → ATTACK（持续盯防上报，赛题一不判摧毁）
                LOST ←── (丢失 > 2s) ←── ENGAGE/ATTACK
                  ↓ 在最后估计位置盘旋 ≤10s 等待重捕获，超时回 SEARCH

赛题一跳过 COORDINATE（单机无协同）；赛题一 dwell_target_s=∞ 永不判摧毁，
ATTACK 后持续跟踪上报即可。

算法：
  - IMM 滤波器（Rust，CV+CA+CT 三模型）用于目标位置估计和速度估计
  - 阿基米德螺旋搜索
  - LOS 瞄准 + 前馈飞行跟踪
  - VERIFY：8 秒窗口（80 帧 @10Hz），速度阈值判别诱饵
  - 每秒 report_target 上报滤波位置
"""

import math
from enum import Enum

from algorithms.estimation.ekf import ImmFilter
from algorithms.estimation.geometry import bearing_rad, haversine_m
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
    VERIFY = "verify"
    ENGAGE = "engage"
    ATTACK = "attack"
    LOST = "lost"


# ── 常量 ────────────────────────────────────────────────────────────────

_SEARCH_FOV = 70.0  # 搜索 FOV（度）
_TRACK_FOV = 15.0  # 跟踪 FOV（度）
_SEARCH_SPEED = 30.0  # 搜索速度（m/s）
_TRACK_SPEED = 25.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 200.0  # 盘旋半径（m）
_LEAD_TIME_S = 1.5  # 前馈时间（s）
_GRACE_FRAMES = 20  # 丢失容忍帧数（~2s @ 10Hz）
_REACQUIRE_TIMEOUT_S = 10.0  # LOST 重捕获时限（s），超时回 SEARCH
_VERIFY_FRAMES = 80  # VERIFY 窗口帧数（8s @ 10Hz），3s 不足以分离速度
_ENGAGE_FRAMES = 10  # ENGAGE 等待 EKF 收敛帧数
_ATTACK_TIME_S = 20.0  # ATTACK 累计盯防时间（秒）
_REPORT_INTERVAL = 1.0  # 上报间隔（秒）
_SPIRAL_RADIUS_M = 1500  # 螺旋搜索半径（m）
_SPIRAL_PITCH_M = 300  # 螺旋螺距（m），由 FOV 覆盖宽度决定
_ASSUME_RANGE_M = 800.0  # 首次检测假设距离（m）
_VERIFY_SPEED_THRESH = 3.9  # VERIFY 速度阈值（m/s），8s 窗口下召回 94%/误判 2%


class MySearchTrackAgent(SearchTrackAgent):
    """赛题一参赛 Agent：EKF 滤波 + 螺旋搜索 + LOS 跟踪。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state: State = State.ACQUIRE
        self._ekf: ImmFilter | None = None
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx: int = 0
        self._lost_frames: int = 0
        self._verify_frames: int = 0
        self._engage_frames: int = 0
        self._attack_time: float = 0.0
        self._lost_time: float = 0.0
        self._last_report_time: float = 0.0
        self._sim_time: float = 0.0
        self._gimbal_phase: float = 0.0  # 云台扫描相位
        self._time_synced: bool = False

    def reset(self) -> None:
        self._state = State.ACQUIRE
        self._ekf = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._lost_frames = 0
        self._verify_frames = 0
        self._engage_frames = 0
        self._attack_time = 0.0
        self._lost_time = 0.0
        self._last_report_time = 0.0
        self._sim_time = 0.0
        self._gimbal_phase = 0.0
        self._time_synced = False

    def decide(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        # 同步引擎 sim_time（score_view 每拍更新），读不到回退 dt 累加。
        # 必须用引擎时间：runner 控制节拍远快于引擎（实测差 2.5 倍），
        # dt 累加会让 1Hz 上报节拍失真（评分按引擎 1Hz 采样，漏报记 0）。
        st = getattr(getattr(obs.briefing, "score_view", None), "sim_time", None)
        if isinstance(st, (int, float)):
            st = float(st)
            if not self._time_synced:
                self._last_report_time = st
                self._time_synced = True
            self._sim_time = st
        else:
            self._sim_time += dt
        cmds: list[Command] = []

        # 初始化 IMM（用 UAV 初始位置作为坐标原点）
        if self._ekf is None:
            self._ekf = ImmFilter(obs.self.lat, obs.self.lon)

        # 生成搜索航点（如果还没有）。briefing.target_initial_pos 是
        # SearchTrackPolicy 随机化**之前**的原坐标，UAV 已同步偏移过去；
        # 以原坐标为螺旋中心会偏离真实目标位置，以自身位置为中心才贴合
        if not self._search_waypoints:
            center_lat, center_lon = obs.self.lat, obs.self.lon
            self._search_waypoints = generate_spiral(
                center_lat,
                center_lon,
                radius_m=_SPIRAL_RADIUS_M,
                pitch_m=_SPIRAL_PITCH_M,
            )

        # 状态机分发
        if self._state == State.ACQUIRE:
            cmds = self._do_acquire(obs, dt)
        elif self._state == State.SEARCH:
            cmds = self._do_search(obs, dt)
        elif self._state == State.VERIFY:
            cmds = self._do_verify(obs, dt)
        elif self._state == State.ENGAGE:
            cmds = self._do_engage(obs, dt)
        elif self._state == State.ATTACK:
            cmds = self._do_attack(obs, dt)
        elif self._state == State.LOST:
            cmds = self._do_lost(obs, dt)

        return cmds

    # ── ACQUIRE：直接进入 SEARCH（briefing.target_initial_pos 是随机化前的坐标，
    # SearchTrackPolicy 把 UAV 和目标同步偏移后，agent 飞向原始坐标是错位点。
    # 检测才是可靠入口——直接走路线先验扫描）──

    def _do_acquire(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds = [set_gimbal_fov(_SEARCH_FOV)]
        # 如果 brief 给的初始位与当前位置差小于 500m，直接进入 SEARCH
        # （否则也直接 SEARCH，让检测引路——路线先验扫描比飞向错位点有效）
        self._state = State.SEARCH
        return self._do_search(obs, dt)
        # 检查是否已到达附近（或直接收到检测）
        if obs.self.detection.detected:
            self._state = State.VERIFY
            self._verify_frames = 0
            return self._do_verify(obs, dt)

        if target_pos:
            dist = haversine_m(obs.self.lat, obs.self.lon, target_pos[0], target_pos[1])
            if dist < 500:  # 到达初始位置 500m 范围内
                self._state = State.SEARCH
                return self._do_search(obs, dt)

        return cmds

    # ── SEARCH：螺旋搜索 + 云台扫描 ────────────────────────────────────

    def _do_search(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds = [set_gimbal_fov(_SEARCH_FOV)]

        # 云台扇扫（pan ±90°，tilt -60° ~ -30°）
        self._gimbal_phase += dt * 0.5  # 扫描速度
        pan = 90.0 * math.sin(self._gimbal_phase)
        tilt = -45.0 + 15.0 * math.sin(self._gimbal_phase * 0.7)
        cmds.append(point_gimbal(pan, tilt))

        # 沿螺旋航点飞行
        if self._search_waypoints:
            wp = self._search_waypoints[self._wp_idx % len(self._search_waypoints)]
            cmds.append(
                fly_to(wp[0], wp[1], speed=_SEARCH_SPEED, loiter_radius=_LOITER_RADIUS)
            )

            # 到达当前航点附近时切换到下一个
            dist = haversine_m(obs.self.lat, obs.self.lon, wp[0], wp[1])
            if dist < 200:
                self._wp_idx += 1

        # 检测到目标 → 进入 VERIFY
        if obs.self.detection.detected:
            self._state = State.VERIFY
            self._verify_frames = 0
            return self._do_verify(obs, dt)

        return cmds

    # ── VERIFY：诱饵鉴别（8 秒窗口） ──────────────────────────────────

    def _do_verify(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # 锁定目标 + EKF 更新
        if det.detected and det.target_lat is not None and det.target_lon is not None:
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

            if not self._ekf.is_initialized():
                self._ekf.initialize(obs.self.lat, obs.self.lon, bearing, range_m)
            else:
                self._ekf.predict(dt)
                self._ekf.update_bearing(obs.self.lat, obs.self.lon, bearing)
                self._ekf.update_range(obs.self.lat, obs.self.lon, range_m)
        else:
            # 丢失检测 → predict only
            if self._ekf.is_initialized():
                self._ekf.predict(dt)

        self._verify_frames += 1

        # 窗口结束：根据速度判断
        if self._verify_frames >= _VERIFY_FRAMES:
            speed = self._ekf.speed_mps() if self._ekf.is_initialized() else 0.0
            if speed >= _VERIFY_SPEED_THRESH:
                # 真目标 → ENGAGE
                self._state = State.ENGAGE
                self._engage_frames = 0
                self._lost_frames = 0
                return self._do_engage(obs, dt)
            else:
                # 诱饵 → 回 SEARCH
                self._ekf = ImmFilter(obs.self.lat, obs.self.lon)
                self._state = State.SEARCH
                return self._do_search(obs, dt)

        return cmds

    # ── ENGAGE：EKF 初始化 + 收敛等待 ────────────────────────────────

    def _do_engage(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # EKF 更新
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            bearing = bearing_rad(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            range_m = haversine_m(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            if not self._ekf.is_initialized():
                self._ekf.initialize(obs.self.lat, obs.self.lon, bearing, range_m)
            else:
                self._ekf.predict(dt)
                self._ekf.update_bearing(obs.self.lat, obs.self.lon, bearing)
                self._ekf.update_range(obs.self.lat, obs.self.lon, range_m)
            self._lost_frames = 0
        else:
            if self._ekf.is_initialized():
                self._ekf.predict(dt)
            self._lost_frames += 1

        # 丢失过多 → LOST
        if self._lost_frames >= _GRACE_FRAMES:
            self._state = State.LOST
            self._lost_time = 0.0
            return self._do_lost(obs, dt)

        # 云台 LOS 瞄准
        if self._ekf.is_initialized():
            est_lat, est_lon = self._ekf.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                est_lat,
                est_lon,
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))

            # 前馈飞行
            ve, vn = self._ekf.velocity_mps()
            lead_lat, lead_lon = compute_lead_point(
                est_lat,
                est_lon,
                ve,
                vn,
                _LEAD_TIME_S,
            )
            cmds.append(
                fly_to(
                    lead_lat, lead_lon, speed=_TRACK_SPEED, loiter_radius=_LOITER_RADIUS
                )
            )

            # 上报
            if self._sim_time - self._last_report_time >= _REPORT_INTERVAL:
                cmds.append(report_target(est_lat, est_lon))
                self._last_report_time = self._sim_time

        self._engage_frames += 1

        # EKF 收敛后进入 ATTACK
        if self._ekf.is_converged(100.0):
            self._state = State.ATTACK
            self._attack_time = 0.0
            return self._do_attack(obs, dt)

        return cmds

    # ── ATTACK：持续盯防 ─────────────────────────────────────────────

    def _do_attack(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        # EKF 更新
        if det.detected and det.target_lat is not None and det.target_lon is not None:
            bearing = bearing_rad(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            range_m = haversine_m(
                obs.self.lat, obs.self.lon, det.target_lat, det.target_lon
            )
            if not self._ekf.is_initialized():
                self._ekf.initialize(obs.self.lat, obs.self.lon, bearing, range_m)
            else:
                self._ekf.predict(dt)
                self._ekf.update_bearing(obs.self.lat, obs.self.lon, bearing)
                self._ekf.update_range(obs.self.lat, obs.self.lon, range_m)
            self._lost_frames = 0
        else:
            if self._ekf.is_initialized():
                self._ekf.predict(dt)
            self._lost_frames += 1

        # 丢失过多 → LOST
        if self._lost_frames >= _GRACE_FRAMES:
            self._state = State.LOST
            self._lost_time = 0.0
            return self._do_lost(obs, dt)

        # 云台 + 前馈飞行 + 上报
        if self._ekf.is_initialized():
            est_lat, est_lon = self._ekf.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat,
                obs.self.lon,
                obs.self.alt,
                est_lat,
                est_lon,
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))

            ve, vn = self._ekf.velocity_mps()
            lead_lat, lead_lon = compute_lead_point(
                est_lat,
                est_lon,
                ve,
                vn,
                _LEAD_TIME_S,
            )
            cmds.append(
                fly_to(
                    lead_lat, lead_lon, speed=_TRACK_SPEED, loiter_radius=_LOITER_RADIUS
                )
            )

            if self._sim_time - self._last_report_time >= _REPORT_INTERVAL:
                cmds.append(report_target(est_lat, est_lon))
                self._last_report_time = self._sim_time

        self._attack_time += dt

        return cmds

    # ── LOST：在最后估计位置盘旋等待重捕获 ─────────────────────────────

    def _do_lost(self, obs: SearchTrackObs, dt: float) -> list[Command]:
        cmds: list[Command] = []
        self._lost_time += dt
        det = obs.self.detection

        # 重新检测到 → 直接回 ENGAGE（赛题一无诱饵，EKF 仍是热的，无需重新 VERIFY）
        if det.detected and det.target_lat is not None:
            self._state = State.ENGAGE
            self._engage_frames = 0
            self._lost_frames = 0
            return self._do_engage(obs, dt)

        # 在滤波预测位置盘旋等待（EKF 按速度外推，扩大 FOV 提高重捕获概率）
        if self._ekf is not None and self._ekf.is_initialized():
            self._ekf.predict(dt)
            est_lat, est_lon = self._ekf.position_wgs84()
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt, est_lat, est_lon
            )
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_SEARCH_FOV))
            cmds.append(
                fly_to(est_lat, est_lon, speed=_TRACK_SPEED, loiter_radius=150.0)
            )

        # 超时仍无检测 → 回 SEARCH（螺旋进度保持，不从头重搜）
        if self._lost_time >= _REACQUIRE_TIMEOUT_S:
            self._state = State.SEARCH
            return self._do_search(obs, dt)

        return cmds

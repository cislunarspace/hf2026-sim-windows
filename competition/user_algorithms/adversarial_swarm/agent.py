"""赛题三对抗集群全域搜索 Agent（v1 骨架）。

五状态 FSM（继承赛题二模式）：
  SEARCH → VERIFY → TRACK
                ↑       ↓ (摧毁/超时) → SEARCH
  JOIN ←── (收到队友 announce)

与赛题二的差异：
  - 10 UAV、K=3 协同盯防（announce 吸引最多 2 架僚机，盘旋半径按 uid 错开
    避免 <200m proximity 扣分）
  - SAM 防空区：只水平绕行（手册：高度锁定 500m）。所有飞行目标点先经
    _push_out_of_sam 推出 air_defense 近似 bbox
  - 动态干扰：obs.self.jammed 翻真时广播 "J:lat,lon" 预警
  - 已摧毁目标记忆（不再跟踪/上报"尸体"）

v1 未做（后续迭代）：
  - 心跳超时判失联 + 任务重分配（手册 FAQ 3）
  - 静态/动态干扰区的航线级规避（当前仅广播预警，通信中断可容忍）
"""

import hashlib
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
from competition.sdk.scenarios.adversarial_swarm import SwarmAgent
from competition.sdk.scenarios.adversarial_swarm.observation import SwarmObs

# ── 常量 ──────────────────────────────────────────────────────────────────

_SEARCH_ALT = 500.0  # 巡航高度（m），手册：全程锁定 500m
_SEARCH_SPEED = 30.0  # 搜索速度（m/s）
_TRACK_SPEED = 25.0  # 跟踪速度（m/s）
_LOITER_RADIUS = 100.0  # 长机盘旋半径（m）

_VERIFY_SAMPLES = 120  # VERIFY 判别所需检测样本数（12s @10Hz）
_VERIFY_SPEED_THRESH = 3.5  # OLS 速度阈值（m/s）
_VERIFY_LOST_ABORT_S = 2.0  # VERIFY 中连续丢失超过此时长则放弃（不记诱饵）

_TRACK_DWELL_S = 20.0  # 盯防摧毁时间（s）
_TRACK_GRACE_S = 2.0  # 丢失容忍时间（s）
_TRACK_TIMEOUT_S = 35.0  # 跟踪超时（s）

_JOIN_TIMEOUT_S = 30.0  # JOIN 超时（s）
_ANNOUNCE_EXPIRE_S = 15.0  # announce 过期时间（s）

_BC_INTERVAL = 0.5  # 广播间隔（s，2Hz）
_JAM_BC_INTERVAL = 1.0  # 干扰预警广播间隔（s，1Hz）
_REPORT_INTERVAL = 1.0  # 上报间隔（s）

_SPIRAL_RADIUS_M = 600.0  # 扇区螺旋半径（m）
_SPIRAL_PITCH_M = 200.0  # 螺旋螺距（m）
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


class State(Enum):
    SEARCH = "SEARCH"
    VERIFY = "VERIFY"
    TRACK = "TRACK"
    JOIN = "JOIN"


class SwarmSearchAgent(SwarmAgent):
    """赛题三参赛 Agent：扇区螺旋搜索 + IMM 判别 + K=3 协同 + SAM 水平绕行。"""

    def __init__(self, my_uid: str):
        super().__init__(my_uid)
        self._state: State = State.SEARCH
        self._imm: ImmFilter | None = None
        self._search_waypoints: list[tuple[float, float]] = []
        self._wp_idx = 0
        self._sector_center: tuple[float, float] = (0.0, 0.0)
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s = 0.0
        self._sim_time = 0.0
        self._target: tuple[float, float] | None = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._last_jam_bc_time = -_JAM_BC_INTERVAL
        self._time_synced = False
        self._known_decoys: list[tuple[float, float]] = []
        self._known_destroyed: list[tuple[float, float]] = []
        self._shared_target: tuple[float, float] | None = None
        self._shared_target_time: float = -1.0
        self._join_time = 0.0
        self._is_wingman = False
        # 僚机盘旋半径按 uid 错开（300/400m），避免两架僚机 <200m
        h = int(hashlib.md5(my_uid.encode()).hexdigest(), 16)
        self._wingman_loiter = 300.0 + (h % 2) * 100.0

    def reset(self) -> None:
        self._state = State.SEARCH
        self._imm = None
        self._search_waypoints = []
        self._wp_idx = 0
        self._sector_center = _uid_sector(self.my_uid)
        self._verify_samples: list[tuple[float, float, float]] = []
        self._verify_lost_s = 0.0
        self._sim_time = 0.0
        self._target = None
        self._dwell_time = 0.0
        self._last_det_time = 0.0
        self._track_time = 0.0
        self._last_report_time = 0.0
        self._last_bc_time = 0.0
        self._last_jam_bc_time = -_JAM_BC_INTERVAL
        self._time_synced = False
        self._known_decoys = []
        self._known_destroyed = []
        self._shared_target = None
        self._shared_target_time = -1.0
        self._join_time = 0.0
        self._is_wingman = False

    def decide(self, obs: SwarmObs, dt: float) -> list[Command]:
        self._sync_time(obs, dt)
        cmds: list[Command] = []

        self._ingest_comms(obs.comm_inbox)
        self._expire_shared_target()

        # 动态干扰自感知 → 广播预警（1Hz）
        if obs.self.jammed and self._sim_time - self._last_jam_bc_time >= (
            _JAM_BC_INTERVAL
        ):
            self._last_jam_bc_time = self._sim_time
            cmds.append(broadcast(f"J:{obs.self.lat:.3f},{obs.self.lon:.3f}"))

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

    def _sync_time(self, obs: SwarmObs, dt: float) -> None:
        """同步引擎 sim_time（briefing.score_view 每拍更新），读不到回退 dt 累加。

        必须用引擎时间：runner 控制节拍远快于引擎（实测差 2.5 倍），
        dt 累加会让 OLS 速度判别和全部时间基准失真。
        """
        st = getattr(getattr(obs.briefing, "score_view", None), "sim_time", None)
        if isinstance(st, (int, float)):
            st = float(st)
            if not self._time_synced:
                self._last_report_time = st
                self._last_bc_time = st
                self._last_det_time = st
                self._time_synced = True
            self._sim_time = st
        else:
            self._sim_time += dt

    # ── 通信 ──────────────────────────────────────────────────────────────

    def _ingest_comms(self, inbox) -> None:
        """解析队友广播。A: announce 优先；T: 跟踪位置；J: 干扰预警（v1 仅忽略）。"""
        for msg in inbox:
            p = msg.payload
            if p.startswith("A:"):
                try:
                    la, lo = p[2:].split(",")
                    self._shared_target = (float(la), float(lo))
                    self._shared_target_time = self._sim_time
                except Exception:
                    pass
            elif p.startswith("T:") and self._shared_target is None:
                try:
                    la, lo = p[2:].split(",")
                    self._shared_target = (float(la), float(lo))
                    self._shared_target_time = self._sim_time
                except Exception:
                    pass

    def _expire_shared_target(self) -> None:
        if (
            self._shared_target is not None
            and self._shared_target_time >= 0.0
            and self._sim_time - self._shared_target_time > _ANNOUNCE_EXPIRE_S
        ):
            self._shared_target = None
            self._shared_target_time = -1.0

    def _near_known(self, lat: float, lon: float, radius_m: float = 150.0) -> bool:
        """点是否在已知诱饵或已摧毁目标附近。"""
        return any(
            haversine_m(lat, lon, d[0], d[1]) < radius_m
            for d in self._known_decoys + self._known_destroyed
        )

    # ── SEARCH：螺旋搜索本扇区 ───────────────────────────────────────────

    def _do_search(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        if not self._search_waypoints:
            self._search_waypoints = generate_spiral(
                self._sector_center[0],
                self._sector_center[1],
                radius_m=_SPIRAL_RADIUS_M,
                pitch_m=_SPIRAL_PITCH_M,
            )
            self._wp_idx = 0

        # 收到队友确认目标 → JOIN（跳过已知已摧毁/诱饵）
        if self._shared_target is not None and not self._near_known(
            self._shared_target[0], self._shared_target[1]
        ):
            self._state = State.JOIN
            self._target = self._shared_target
            self._join_time = 0.0
            self._imm = None
            return self._do_join(obs, dt)

        # 检测到目标 → VERIFY（跳过已知诱饵与已摧毁目标）
        if det.detected and det.target_lat is not None:
            if not self._near_known(det.target_lat, det.target_lon):
                self._state = State.VERIFY
                self._target = (det.target_lat, det.target_lon)
                self._imm = ImmFilter(obs.self.lat, obs.self.lon)
                self._verify_samples = []
                self._verify_lost_s = 0.0
                # 不在此处 announce：候选未判别，提前 announce 会让集群
                # 收敛到同一个静止诱饵。判别通过进 TRACK 时再 announce。
                return self._do_verify(obs, dt)

        # 沿螺旋航点飞行（SAM 推避）
        if self._search_waypoints:
            wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            dist = haversine_m(obs.self.lat, obs.self.lon, wp_lat, wp_lon)
            if dist < 50.0:
                self._wp_idx = (self._wp_idx + 1) % len(self._search_waypoints)
                wp_lat, wp_lon = self._search_waypoints[self._wp_idx]
            wp_lat, wp_lon = _push_out_of_sam(wp_lat, wp_lon, obs.briefing)
            cmds.append(
                fly_to(wp_lat, wp_lon, alt=_SEARCH_ALT, speed=_SEARCH_SPEED)
            )

        cmds.append(set_gimbal_fov(_SEARCH_FOV))
        return cmds

    # ── VERIFY：OLS 速度判别（ImmFilter 同步更新供 TRACK 接管） ──────────

    def _do_verify(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection

        if det.detected and det.target_lat is not None and det.target_lon is not None:
            self._verify_lost_s = 0.0
            self._verify_samples.append(
                (self._sim_time, det.target_lat, det.target_lon)
            )
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt,
                det.target_lat, det.target_lon, uav_heading_deg=obs.self.heading_deg)
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

        # 收到队友确认的其他目标 → JOIN（优先协同）
        if self._shared_target is not None and self._target is not None:
            d = haversine_m(
                self._shared_target[0], self._shared_target[1],
                self._target[0], self._target[1],
            )
            if d > 200.0 and not self._near_known(
                self._shared_target[0], self._shared_target[1]
            ):
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
                self._state = State.TRACK
                self._is_wingman = False
                self._dwell_time = 0.0
                self._track_time = 0.0
                self._last_det_time = self._sim_time
                return self._do_track(obs, dt)
            else:
                if self._target:
                    self._known_decoys.append(self._target)
                self._state = State.SEARCH
                self._target = None
                self._imm = None
                return self._do_search(obs, dt)

        # 飞向目标区域（SAM 推避）
        if self._target:
            lat, lon = _push_out_of_sam(
                self._target[0], self._target[1], obs.briefing
            )
            cmds.append(
                fly_to(
                    lat, lon,
                    alt=_SEARCH_ALT, speed=_SEARCH_SPEED,
                    loiter_radius=_LOITER_RADIUS,
                )
            )

        return cmds

    # ── TRACK：盘旋跟踪 + 广播 + 上报 ───────────────────────────────────

    def _do_track(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._track_time += dt

        # 跟踪目标位置更新 + IMM 滤波（上报用滤波位置）
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

        # 盯防满 20s → 视为摧毁，记入已摧毁列表，回 SEARCH
        if self._dwell_time >= _TRACK_DWELL_S:
            if self._target:
                self._known_destroyed.append(self._target)
            self._state = State.SEARCH
            self._target = None
            self._imm = None
            self._is_wingman = False
            self._dwell_time = 0.0
            return self._do_search(obs, dt)

        # 超时未摧毁 → 回 SEARCH；低速目标记为诱饵
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

        # 广播：长机首次 announce，之后定期 T: 位置
        if self._target and self._sim_time - self._last_bc_time >= _BC_INTERVAL:
            self._last_bc_time = self._sim_time
            if not self._is_wingman and self._dwell_time <= dt * 2:
                cmds.append(
                    broadcast(f"A:{self._target[0]:.3f},{self._target[1]:.3f}")
                )
            else:
                cmds.append(
                    broadcast(f"T:{self._target[0]:.3f},{self._target[1]:.3f}")
                )

        # 云台 + 飞行（SAM 推避；僚机用错开的更大盘旋半径）
        if self._target:
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt,
                self._target[0], self._target[1], uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
            loiter = self._wingman_loiter if self._is_wingman else _LOITER_RADIUS
            lat, lon = _push_out_of_sam(
                self._target[0], self._target[1], obs.briefing
            )
            cmds.append(
                fly_to(
                    lat, lon,
                    alt=_SEARCH_ALT, speed=_TRACK_SPEED, loiter_radius=loiter,
                )
            )

            # report_target（仅确认移动目标，用滤波位置）
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

    # ── JOIN：收敛到共享目标 ─────────────────────────────────────────────

    def _do_join(self, obs: SwarmObs, dt: float) -> list[Command]:
        cmds = []
        det = obs.self.detection
        self._join_time += dt

        if self._target is None or self._join_time >= _JOIN_TIMEOUT_S:
            self._state = State.SEARCH
            self._target = None
            self._shared_target = None
            self._shared_target_time = -1.0
            self._join_time = 0.0
            return self._do_search(obs, dt)

        if det.detected and det.target_lat is not None and det.target_lon is not None:
            d = haversine_m(
                det.target_lat, det.target_lon, self._target[0], self._target[1]
            )
            if d < 300.0:
                self._target = (det.target_lat, det.target_lon)

        dist_to_target = haversine_m(
            obs.self.lat, obs.self.lon, self._target[0], self._target[1]
        )
        if dist_to_target < 200.0 and det.detected:
            self._state = State.TRACK
            self._is_wingman = True
            self._dwell_time = 0.0
            self._track_time = 0.0
            self._last_det_time = self._sim_time
            self._imm = ImmFilter(obs.self.lat, obs.self.lon)
            return self._do_track(obs, dt)

        lat, lon = _push_out_of_sam(
            self._target[0], self._target[1], obs.briefing
        )
        cmds.append(
            fly_to(
                lat, lon,
                alt=_SEARCH_ALT, speed=_SEARCH_SPEED,
                loiter_radius=_LOITER_RADIUS,
            )
        )

        if det.detected and det.target_lat is not None:
            pan, tilt = compute_gimbal_angles(
                obs.self.lat, obs.self.lon, obs.self.alt,
                det.target_lat, det.target_lon, uav_heading_deg=obs.self.heading_deg)
            cmds.append(point_gimbal(pan, tilt))
            cmds.append(set_gimbal_fov(_TRACK_FOV))
        else:
            cmds.append(set_gimbal_fov(_SEARCH_FOV))

        return cmds

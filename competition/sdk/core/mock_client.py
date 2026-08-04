"""MockSimClient — 纯 Python 内存引擎（快速仿真）。

替换闭源 ``opensim-sim.exe``：实现 :class:`SimClient` 协议 + FakeRedis
（``_plan_segment`` 依赖 pubsub 轮询 ``astar_plan_result``），在进程内
推进世界（UAV 运动/云台/检测真值/comm 路由、目标与诱饵沿注入轨迹移动），
产出与引擎同构的 ``sim:state`` 帧。评分/评测/obs 隔离链路
（RunnerBase / coop_eval / isolation / AccuracySimulator）原样复用——
mock 局与真实局的差异只来自本文件的环境模拟层：

  * A* 返回路网折线近似（沿 ``config/points.json`` 最近路网节点的折线，
    比直线段更贴近真实沿路网机动，真实里真目标被锁率与 mock 收敛的关键）；
  * UAV 转弯动力学简化（heading 瞬转 + 到点绕圈 loiter）；
  * 无地形遮挡、无 UE 渲染相机帧。

跑法（配合 runner 的 ``--mock`` 参数）：

    python -m competition run --mock --scenario coop_decoy \\
        --agent competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent \\
        --duration 180 --output output/mock_v1
"""
from __future__ import annotations

import json
import math
import random
import threading
import time
from typing import Any, Dict, List, Optional

from .._vendored.geometry import bearing_deg, destination, haversine_m
from .commands import Command
from .world_state import WorldState, parse_world_state

# 与引擎 CommComponent 默认一致（scenario.json 的 comm params 为空 → 用默认）
DEFAULT_COMM_RANGE_M = 1000.0
DEFAULT_COMM_MAX_BYTES = 50
DEFAULT_COMM_MAX_RATE_HZ = 4.0

_TYPE_BY_KIND = {
    "uav": "fixed_wing_uav",
    "ground_vehicle": "ground_vehicle",
    "decoy_vehicle": "decoy_vehicle",
}


def _wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def _hash01(*parts) -> float:
    """确定性 [0,1) 哈希（拍号+目标uid）：同目标同一拍所有机共享同一判定。"""
    import hashlib
    h = hashlib.md5("|".join(str(p) for p in parts).encode()).hexdigest()
    return int(h[:8], 16) / 0xFFFFFFFF


def _sep_deg(pan_g: float, tilt_g: float, pan_t: float, tilt_t: float) -> float:
    """云台轴线与目标视线的角偏差（度）。"""
    dpan = math.radians(_wrap180(pan_t - pan_g))
    c = (math.sin(math.radians(tilt_g)) * math.sin(math.radians(tilt_t))
         + math.cos(math.radians(tilt_g)) * math.cos(math.radians(tilt_t))
         * math.cos(dpan))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ── 路网折线近似（astar_plan 校准） ────────────────────────────────────
#
# 真实引擎的 astar_plan 沿 A* 路网规划（折线，54 点），mock 早期退化为
# 直线段导致真目标轨迹差异 → 真实局真目标被锁 0 次而 mock 能锁能杀。
# 用 config/points.json 的路网节点做折线近似：找离 start 最近的路网
# 节点集合（一条路的 Start/Waypoints/End），沿该路折线推进。
_road_cache: Optional[List[List[tuple]]] = None


def _load_roads() -> List[List[tuple]]:
    """加载真目标路网（config/points.json）的折线点序列（缓存）。"""
    global _road_cache
    if _road_cache is not None:
        return _road_cache
    import json as _json
    from pathlib import Path
    root = Path(__file__).resolve().parents[3]
    path = root / "config" / "points.json"
    try:
        with open(path, encoding="utf-8-sig") as f:
            d = _json.load(f)
        roads = []
        for r in d.get("Paths", []):
            pts = []
            s = r.get("Start")
            if s:
                pts.append((float(s["Latitude"]), float(s["Longitude"])))
            for wp in r.get("Waypoints", []):
                pts.append((float(wp["Latitude"]), float(wp["Longitude"])))
            e = r.get("End")
            if e:
                pts.append((float(e["Latitude"]), float(e["Longitude"])))
            if len(pts) >= 2:
                roads.append(pts)
        _road_cache = roads
    except Exception:
        _road_cache = []
    return _road_cache


def _nearest_road(lat: float, lon: float) -> List[tuple]:
    """离 (lat, lon) 最近的路网折线（按到任意折线点的最近距离）。

    prepare_scenario 把真目标 spawn 在路线 Start，但 astar_plan 请求的
    start 可能在路网中段（目标移动后），所以按"到折线上任意点的最近
    距离"判定路网归属。"""
    roads = _load_roads()
    if not roads:
        return []
    best, best_d = None, 1e18
    for pts in roads:
        d = min(haversine_m(lat, lon, p[0], p[1]) for p in pts)
        if d < best_d:
            best_d, best = d, pts
    return best or []


def _road_path_between(start_lat: float, start_lon: float,
                       end_lat: float, end_lon: float) -> List[dict]:
    """返回从 start 到 end 的折线近似（相邻 waypoint 间的路网短段）。

    真实 astar_plan 的每个请求是 route 相邻 waypoint 之间的 A* 路径
    （几百米~1km，沿路网网格细分）；整条路线 = 多段串联（6-8km）。
    mock 近似：start/end 在同一条路网折线 → 取两点间的路网段（走线不
    抄近路）；不在同一条路（路网稀疏区）→ 直线段。
    """
    road_s = _nearest_road(start_lat, start_lon)
    road_e = _nearest_road(end_lat, end_lon)
    if not road_s:
        return [{"lat": start_lat, "lon": start_lon},
                {"lat": end_lat, "lon": end_lon}]

    def _proj_idx(road, p):
        best_i, best_d = 0, 1e18
        for i in range(len(road)):
            d = haversine_m(p[0], p[1], road[i][0], road[i][1])
            if d < best_d:
                best_d, best_i = d, i
        return best_i

    pts = [{"lat": start_lat, "lon": start_lon}]
    if road_s is road_e or _same_road(road_s, road_e):
        si = _proj_idx(road_s, (start_lat, start_lon))
        ei = _proj_idx(road_s, (end_lat, end_lon))
        lo, hi = (si, ei) if ei > si else (ei, si)
        seg = road_s[lo + 1:hi + 1]
        if ei < si:
            seg = list(reversed(seg))
        for p in seg:
            pts.append({"lat": p[0], "lon": p[1]})
    pts.append({"lat": end_lat, "lon": end_lon})
    return pts


def _same_road(a: List[tuple], b: List[tuple]) -> bool:
    """两条折线是否同一条路（首点相同即同一路线对象）。"""
    return (a is b) or (a and b and a[0] == b[0] and a[-1] == b[-1])


# ── FakeRedis：让 _astar_navigator._plan_segment 的 pubsub 轮询能读到帧 ──


class _FakePubSub:
    """每个订阅者独立 last_seen 游标；帧是共享 append-only 队列。"""

    def __init__(self, owner: "MockSimClient") -> None:
        self._owner = owner
        self._seen = 0

    def subscribe(self, *channels) -> None:
        return None

    def get_message(self, timeout: float = 0.1) -> Optional[dict]:
        deadline = time.time() + timeout
        while True:
            with self._owner._lock:
                frames = self._owner._frames
                if len(frames) > self._seen:
                    raw = frames[self._seen]
                    self._seen += 1
                    return {"type": "message", "data": json.dumps(raw)}
            if time.time() >= deadline:
                return None
            time.sleep(0.002)

    def close(self) -> None:
        return None


class _FakeRedis:
    """伪装 redis 客户端：publish no-op（进度等通道），pubsub 读共享帧。"""

    def __init__(self, owner: "MockSimClient") -> None:
        self._owner = owner

    def publish(self, channel: str, data: str) -> int:
        return 0

    def pubsub(self, ignore_subscribe_messages: bool = True) -> _FakePubSub:
        return _FakePubSub(self._owner)


# ── MockSimClient ─────────────────────────────────────────────────────────


class MockSimClient:
    """进程内世界模拟器，实现 SimClient 协议（client.py 同款签名）。

    主循环每调一次 :meth:`poll_latest`，世界前进一个控制周期
    （默认 0.1s，与 runner 的 control_rate_hz=10 对齐），并返回最新帧。
    命令在 :meth:`publish` / :meth:`publish_raw` 时即时应用。
    """

    def __init__(self, *, scenario_path: str, seed: int = 0,
                 control_dt: float = 0.1, detect_loss: float = 0.50,
                 decoy_lock_rate: float = 0.35,
                 log=None, quiet: bool = False) -> None:
        self._scenario_path = scenario_path
        self._seed = int(seed)
        self._control_dt = control_dt
        self._detect_loss = detect_loss  # 0=无丢失（测试），0.50=真实引擎校准
        self._decoy_lock_rate = decoy_lock_rate  # 诱饵抢锁概率（0~1）
        self._log = (lambda *a, **kw: None) if quiet else (log or print)
        self._redis = _FakeRedis(self)          # 供 _plan_segment 轮询
        self._lock = threading.Lock()
        self._frames: List[dict] = []           # append-only 共享帧
        self._sim_time = 0.0
        self._status = "running"
        self._entities: Dict[str, dict] = {}
        self._pending_msgs: List[dict] = []     # {sender, payload, peer}
        self._pending_plans: Dict[str, dict] = {}
        self._last_comm_t: Dict[str, float] = {}
        self._latest: Optional[WorldState] = None
        self._rng = random.Random(seed ^ 0x5EED)  # 检测丢失模型（可播种复现）
        self._rng_lock = threading.Lock()

    # ── SimClient 协议 ────────────────────────────────────────────────

    def connect(self) -> None:
        with open(self._scenario_path, encoding="utf-8-sig") as f:
            scenario = json.load(f)
        for ent in scenario.get("entities", []):
            uid = str(ent.get("id") or ent.get("name") or "")
            if not uid:
                continue
            etype = str(ent.get("type", "")).lower()
            if "uav" in etype:
                kind = "uav"
            elif "decoy" in etype:
                kind = "decoy_vehicle"
            else:
                kind = "ground_vehicle"
            p = ent.get("params", {}) or {}
            comps = ent.get("components", {}) or {}
            kin = (comps.get("kinematics", {}) or {}).get("params", {}) or {}
            gim = (comps.get("gimbal_tracking", {}) or {}).get("params", {}) or {}
            self._entities[uid] = {
                "kind": kind,
                "name": str(ent.get("name", uid)),
                "lat": float(p.get("initial_latitude", 0.0)),
                "lon": float(p.get("initial_longitude", 0.0)),
                "alt": float(p.get("initial_altitude", 500.0)),
                "heading": float(p.get("initial_heading", 0.0)),
                "speed": 0.0,
                "status": "active",
                # UAV 控制
                "dest": None,                   # set_destination 目标点
                "loitering": False,
                "loiter_angle": 0.0,
                "min_speed": float(kin.get("min_speed", 15.0)),
                "max_speed": float(kin.get("max_speed", 40.0)),
                "gimbal": {
                    "pan": 0.0, "tilt": 0.0, "fov": float(gim.get("fov", 30.0)),
                    "cmd_pan": 0.0, "cmd_tilt": 0.0, "cmd_fov": float(gim.get("fov", 30.0)),
                    "pan_rate": float(gim.get("pan_rate_limit_dps", 60.0)),
                    "tilt_rate": float(gim.get("tilt_rate_limit_dps", 45.0)),
                } if kind == "uav" else None,
                # 目标/诱饵轨迹
                "waypoints": None, "wp_idx": 0, "halted": False,
                "comm": None if kind != "uav" else {
                    "inbox": [],
                    "sent": 0, "delivered": 0, "received": 0,
                    "rejected_bytes": 0, "rejected_rate": 0,
                    "rejected_range": 0, "rejected_jam": 0,
                },
            }
        self._log(f"[mock] 世界初始化: {len(self._entities)} 实体, "
                  f"seed={self._seed}")

    def close(self) -> None:
        return None

    def wait_first_state(self, timeout: float = 120.0) -> WorldState:
        return self._emit_frame()

    def poll_latest(self, timeout: float = 0.05) -> Optional[WorldState]:
        self._advance(self._control_dt)
        return self._emit_frame()

    # ── publishing ────────────────────────────────────────────────────

    def publish(self, unique_id: str, cmd: Command) -> int:
        self._apply_command(str(unique_id), cmd.verb, dict(cmd.params))
        return 1

    def publish_engine(self, verb: str) -> int:
        if verb == "end":
            self._status = "ended"
        return 1

    def publish_raw(self, d: dict) -> int:
        self._apply_command(str(d.get("unique_id", "")),
                            str(d.get("cmd", "")), d.get("params") or {})
        return 1

    def __enter__(self) -> "MockSimClient":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    # ── 命令应用（即时生效，与引擎语义一致） ─────────────────────────

    def _apply_command(self, uid: str, verb: str, params: dict) -> None:
        e = self._entities.get(uid)
        if verb == "astar_plan":
            # 路网折线近似（真实沿路网 54 点，mock 早期直线段导致轨迹差异）
            self._pending_plans[uid] = {
                "start_lat": float(params["start_lat"]),
                "start_lon": float(params["start_lon"]),
                "end_lat": float(params["end_lat"]),
                "end_lon": float(params["end_lon"]),
            }
            self._emit_frame()  # 立即产帧，让 _plan_segment 轮询读到 result
            return
        if e is None:
            return
        if verb == "set_destination":
            e["dest"] = {
                "lat": float(params["latitude"]),
                "lon": float(params["longitude"]),
                "alt": float(params.get("altitude", e["alt"])),
                "speed": float(params["speed"]) if params.get("speed") is not None
                         else max(e["min_speed"], e["speed"]),
                "loiter_radius": float(params.get("loiter_radius", 200.0)),
                "turn_direction": str(params.get("turn_direction", "right")),
            }
            e["loitering"] = False
        elif verb == "set_heading":
            e["heading"] = float(params.get("heading", e["heading"])) % 360.0
        elif verb == "set_speed":
            e["speed"] = float(params.get("speed", 0.0))
        elif verb == "set_fov":
            e["gimbal"]["cmd_fov"] = min(120.0, max(5.0, float(params.get("angle", 30.0))))
            e["gimbal"]["fov"] = e["gimbal"]["cmd_fov"]  # 探针确认 set_fov 实时生效
        elif verb == "component.gimbal_tracking.set_orientation":
            e["gimbal"]["cmd_pan"] = float(params.get("pan", 0.0))
            e["gimbal"]["cmd_tilt"] = float(params.get("tilt", 0.0))
        elif verb == "comm.broadcast":
            self._pending_msgs.append(
                {"sender": uid, "payload": str(params.get("payload", "")), "peer": None})
        elif verb == "comm.send":
            self._pending_msgs.append({
                "sender": uid,
                "payload": str(params.get("payload", "")),
                "peer": str(params.get("peer_target_unique_id", "")),
            })
        elif verb == "set_trajectory":
            wps = params.get("waypoints") or []
            e["waypoints"] = [(float(p["lat"]), float(p["lon"])) for p in wps]
            e["wp_idx"] = 0
            e["halted"] = False
        elif verb == "set_position":
            # 摧毁后冻结（runner._halt_destroyed_targets 的 set_position）
            e["lat"] = float(params.get("latitude", e["lat"]))
            e["lon"] = float(params.get("longitude", e["lon"]))
            e["halted"] = True
        # regenerate_zones / pause / resume / step：mock 无 zone，no-op

    # ── 世界推进 ──────────────────────────────────────────────────────

    def _advance(self, dt: float) -> None:
        self._sim_time += dt
        for uid, e in self._entities.items():
            if e["kind"] == "uav":
                self._advance_uav(uid, e, dt)
            else:
                self._advance_ground(uid, e, dt)
        for e in self._entities.values():
            if e["gimbal"] is not None:
                self._advance_gimbal(e, dt)
        self._route_comm()

    def _advance_uav(self, uid: str, e: dict, dt: float) -> None:
        d = e["dest"]
        if d is None:
            return
        if e["loitering"]:
            # 到点绕圈（简化 loiter：沿半径圆周运动，heading 沿切向）
            r = max(d["loiter_radius"], 1.0)
            e["loiter_angle"] += e["speed"] * dt / r
            ang = e["loiter_angle"]
            lat, lon = destination(d["lat"], d["lon"],
                                   math.degrees(ang), r)
            e["lat"], e["lon"] = lat, lon
            e["heading"] = (math.degrees(ang) + 90.0) % 360.0
            return
        dist = haversine_m(e["lat"], e["lon"], d["lat"], d["lon"])
        if dist <= d["loiter_radius"]:
            e["loitering"] = True
            e["loiter_angle"] = math.radians(
                bearing_deg(e["lat"], e["lon"], d["lat"], d["lon"]))
            e["speed"] = d["speed"]
            return
        brg = bearing_deg(e["lat"], e["lon"], d["lat"], d["lon"])
        step = d["speed"] * dt
        e["lat"], e["lon"] = destination(e["lat"], e["lon"], brg, step)
        e["heading"] = brg
        e["speed"] = d["speed"]

    def _advance_gimbal(self, e: dict, dt: float) -> None:
        g = e["gimbal"]
        dp = _wrap180(g["cmd_pan"] - g["pan"])
        g["pan"] += max(-g["pan_rate"] * dt, min(g["pan_rate"] * dt, dp))
        dtilt = g["cmd_tilt"] - g["tilt"]
        g["tilt"] += max(-g["tilt_rate"] * dt, min(g["tilt_rate"] * dt, dtilt))

    def _advance_ground(self, uid: str, e: dict, dt: float) -> None:
        if e["halted"] or not e["waypoints"] or e["wp_idx"] >= len(e["waypoints"]) - 1:
            return
        tgt = e["waypoints"][e["wp_idx"] + 1]
        brg = bearing_deg(e["lat"], e["lon"], tgt[0], tgt[1])
        dist = haversine_m(e["lat"], e["lon"], tgt[0], tgt[1])
        step = max(0.0, e["speed"]) * dt
        if step >= dist:
            e["lat"], e["lon"] = tgt
            e["wp_idx"] += 1
        else:
            e["lat"], e["lon"] = destination(e["lat"], e["lon"], brg, step)
        e["heading"] = brg

    def _route_comm(self) -> None:
        """引擎内 comm 路由：广播/定向发送 → 距离/速率/字节检查 → inbox。"""
        for e in self._entities.values():
            if e["comm"] is not None:
                e["comm"]["inbox"] = []   # inbox 是帧内瞬时消息
        msgs, self._pending_msgs = self._pending_msgs, []
        for m in msgs:
            s = self._entities.get(m["sender"])
            if s is None or s["comm"] is None:
                continue
            now = self._sim_time
            last = self._last_comm_t.get(m["sender"], -1e9)
            if now - last < 1.0 / DEFAULT_COMM_MAX_RATE_HZ:
                s["comm"]["rejected_rate"] += 1
                continue
            self._last_comm_t[m["sender"]] = now
            payload = m["payload"]
            if len(payload.encode("utf-8")) > DEFAULT_COMM_MAX_BYTES:
                s["comm"]["rejected_bytes"] += 1
                continue
            s["comm"]["sent"] += 1
            for tuid, te in self._entities.items():
                if te["comm"] is None or tuid == m["sender"]:
                    continue
                if m["peer"] and tuid != m["peer"]:
                    continue
                if haversine_m(s["lat"], s["lon"], te["lat"], te["lon"]) > DEFAULT_COMM_RANGE_M:
                    s["comm"]["rejected_range"] += 1
                    continue
                te["comm"]["inbox"].append(
                    {"sender": m["sender"], "payload": payload,
                     "recv_time": now})
                te["comm"]["received"] += 1
                s["comm"]["delivered"] += 1

    # ── 检测真值（FOV 锥内锁"离光轴最近"，探针证实的引擎模型） ─────
    # 丢失模型：真实引擎在目标保持 FOV 内时也会周期性脱锁。校准数据：
    # dev 引擎 TRACK 状态 det0≈49%（seed6/seed7 两局 600s 实测），
    # stand 7/20 引擎 ≈50%——两版引擎行为一致。
    # 丢失的"相关性"：目标可见性是公共因素（同一目标同一拍所有机共享
    # 同一随机判定，用 时间+目标uid 的确定性哈希）——同视线两机（同侧
    # 编队）丢失同步（同丢同锁），独立视线（分居两侧）丢失独立。这是
    # v13 同侧编队的收益来源（真实 6 局 0 杀：双机锁定时段不重叠）。

    def _compute_detection(self, e: dict) -> dict:
        g = e["gimbal"]
        fov_half = g["fov"] / 2.0
        # 收集 FOV 锥内所有候选（sep, ouid, entity）
        candidates: list = []
        for ouid, oe in self._entities.items():
            if oe["kind"] not in ("ground_vehicle", "decoy_vehicle"):
                continue
            pan_t = _wrap180(bearing_deg(e["lat"], e["lon"],
                                         oe["lat"], oe["lon"]) - e["heading"])
            gnd = haversine_m(e["lat"], e["lon"], oe["lat"], oe["lon"])
            alt_diff = e["alt"] - oe["alt"]
            tilt_t = -90.0 if gnd < 1e-3 and alt_diff > 0 else (
                -math.degrees(math.atan2(alt_diff, gnd)) if gnd >= 1e-3 else 0.0)
            sep = _sep_deg(g["pan"], g["tilt"], pan_t, tilt_t)
            if sep < fov_half:
                candidates.append((sep, ouid, oe))
        if not candidates:
            return {"detected": False, "confidence": 0.0}
        candidates.sort(key=lambda c: c[0])
        # 区分 FOV 内的真目标和诱饵
        ground = [(s, uid, oe) for s, uid, oe in candidates if oe["kind"] == "ground_vehicle"]
        decoys = [(s, uid, oe) for s, uid, oe in candidates if oe["kind"] == "decoy_vehicle"]
        # 诱饵抢锁：FOV 内同时有真目标和诱饵时，按概率决定锁哪个
        if ground and decoys:
            h = _hash01(int(self._sim_time * 10), e["name"], "decoy_lock")
            if h < self._decoy_lock_rate:
                best_sep, best_uid, best = decoys[0]   # 最近的诱饵
            else:
                best_sep, best_uid, best = ground[0]    # 最近的真目标
        else:
            best_sep, best_uid, best = candidates[0]    # 只有一类，取最近
        # 目标可见性：中心 ~50%（detect_loss=0.5 校准），锥边缘更低；
        # 随机判定按 (拍, 目标uid) 哈希——所有看同一目标的机共享同一判定
        sep_ratio = best_sep / fov_half
        p_det = (1.0 - self._detect_loss) * (1.0 - 0.5 * min(1.0, sep_ratio))
        if _hash01(int(self._sim_time * 10), best_uid) > p_det:
            return {"detected": False, "confidence": 0.0}
        conf = max(0.0, min(1.0, 1.0 - best_sep / fov_half))
        is_decoy = best["kind"] == "decoy_vehicle"
        return {
            "detected": True,
            "confidence": round(conf, 6),
            "target_position": {"latitude": best["lat"], "longitude": best["lon"],
                                "altitude": best["alt"]},
            "azimuth_error": round(best_sep, 6),
            "target_type": "decoy_vehicle" if is_decoy else "ground_vehicle",
            "misid_flag": is_decoy,
            "misid_count": 1 if is_decoy else 0,
            "misid_track_duration": 0.0,
        }

    # ── 帧产出 ────────────────────────────────────────────────────────

    def _entity_frame(self, uid: str, e: dict) -> dict:
        frame: dict = {
            "type": _TYPE_BY_KIND[e["kind"]],
            "name": e["name"],
            "heading": round(e["heading"], 6),
            "velocity": round(e["speed"], 6),
            "speed": round(e["speed"], 6),
            "platform": {
                "entity_type": e["kind"],
                "position": {"latitude": round(e["lat"], 9),
                             "longitude": round(e["lon"], 9),
                             "altitude": round(e["alt"], 6)},
                "status": e["status"],
            },
        }
        if e["gimbal"] is not None:
            g = e["gimbal"]
            frame["gimbal_tracking"] = {
                "pan_angle": round(g["pan"], 6),
                "tilt_angle": round(g["tilt"], 6),
                "fov": round(g["fov"], 6),
                "auto_track": False,
                "detection": self._compute_detection(e),
            }
        if e["comm"] is not None:
            c = e["comm"]
            frame["comm"] = {
                "inbox": c["inbox"],
                "stats": {k: c[k] for k in (
                    "sent", "delivered", "received", "rejected_bytes",
                    "rejected_rate", "rejected_range", "rejected_jam")},
                "range_m": DEFAULT_COMM_RANGE_M,
                "max_bytes": DEFAULT_COMM_MAX_BYTES,
                "max_rate_hz": DEFAULT_COMM_MAX_RATE_HZ,
                "external_jammed": False,
            }
        if e["waypoints"] is not None:
            frame["trajectory"] = {
                "is_navigating": e["wp_idx"] < len(e["waypoints"]) - 1,
                "current_wp_index": e["wp_idx"],
                # 与引擎一致：target_motion.predict_target_position 读 wp["lat"/"lon"]
                "waypoints": [{"lat": w[0], "lon": w[1]}
                              for w in e["waypoints"]],
                "speed": round(e["speed"], 6),
            }
        return frame

    def _emit_frame(self) -> WorldState:
        with self._lock:
            frame: dict = {
                "timestamp": self._sim_time,
                "status": self._status,
                "sim_time": self._sim_time,
                "sim_time_str": f"{self._sim_time:.6f}",
                "step_perf": {},
                "reason": "",
                "zones": {},
            }
            for uid, e in self._entities.items():
                frame[uid] = self._entity_frame(uid, e)
            # A* 请求的路网折线近似结果（一次性；_plan_segment 并发轮询各读各的）
            for uid, plan in self._pending_plans.items():
                wps = _road_path_between(
                    plan["start_lat"], plan["start_lon"],
                    plan["end_lat"], plan["end_lon"])
                frame.setdefault(uid, {})["astar_plan_result"] = {
                    "success": True, "count": len(wps),
                    "waypoints": wps,
                }
            self._pending_plans.clear()
            self._frames.append(frame)
        ws = parse_world_state(frame)
        self._latest = ws
        return ws

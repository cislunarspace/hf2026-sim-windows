"""tests/test_swarm_agent.py — 赛题三对抗集群 Agent 测试。

使用 mock obs 验证状态机、K=3 协同、SAM 推避、速度带判别与干扰广播。
mock 工厂模式参考 tests/test_coop_decoy_agent.py。
"""

from unittest.mock import MagicMock

import pytest

try:
    from competition.sdk.core.commands import Command
    from competition.user_algorithms.adversarial_swarm.agent import (
        SwarmSearchAgent,
        _push_out_of_sam,
        _uid_sector,
    )
except ImportError:
    pytest.skip("adversarial_swarm agent 尚未实现，跳过测试", allow_module_level=True)


# ── Mock 工厂 ──────────────────────────────────────────────────────────


def _make_zone(kind="air_defense", bbox=((27.000, 124.995), (27.010, 125.005))):
    z = MagicMock()
    z.kind = kind
    z.bbox = bbox
    z.alt_max = 2500.0
    return z


def _make_obs(
    lat: float = 27.0,
    lon: float = 125.0,
    alt: float = 500.0,
    detected: bool = False,
    target_lat: float | None = None,
    target_lon: float | None = None,
    heading: float = 0.0,
    speed: float = 30.0,
    jammed: bool = False,
    zones: tuple = (),
    comm_inbox: tuple = (),
) -> MagicMock:
    """创建 mock SwarmObs。briefing.score_view.sim_time 是 MagicMock
    （非数值），agent 回退 dt 累加——与 coop 测试同一套路。"""
    obs = MagicMock()
    obs.self.uid = "uav_1"
    obs.self.lat = lat
    obs.self.lon = lon
    obs.self.alt = alt
    obs.self.heading_deg = heading
    obs.self.speed = speed
    obs.self.status = "active"
    obs.self.jammed = jammed

    obs.self.detection.detected = detected
    obs.self.detection.confidence = 0.8 if detected else 0.0
    obs.self.detection.target_lat = target_lat if detected else None
    obs.self.detection.target_lon = target_lon if detected else None
    obs.self.detection.target_type = None

    obs.briefing.approximate_zones = zones
    obs.comm_inbox = comm_inbox
    return obs


def _msg(payload: str, sender: str) -> MagicMock:
    m = MagicMock()
    m.payload = payload
    m.sender_uid = sender
    return m


def _find_cmd(cmds, verb: str):
    for cmd in cmds:
        if isinstance(cmd, Command) and cmd.verb == verb:
            return cmd
    return None


def _payloads(cmds) -> list[str]:
    return [
        c.params["payload"]
        for c in cmds
        if isinstance(c, Command) and c.verb == "comm.broadcast"
    ]


# ── 工具函数 ──────────────────────────────────────────────────────────


class TestHelpers:
    def test_uid_sector_in_bbox(self):
        for uid in ("10001", "10002", "uav_x", "7"):
            lat, lon = _uid_sector(uid)
            assert 26.9818 <= lat <= 27.0250
            assert 124.9800 <= lon <= 125.0203

    def test_push_out_of_sam_inside(self):
        briefing = MagicMock()
        briefing.approximate_zones = (_make_zone(),)
        # 点在 bbox 内（靠近南边），应被推出南边界之外
        lat, lon = _push_out_of_sam(27.001, 125.000, briefing)
        assert lat < 27.000, f"应推出南边界，实际 lat={lat}"

    def test_push_out_of_sam_outside_untouched(self):
        briefing = MagicMock()
        briefing.approximate_zones = (_make_zone(),)
        lat, lon = _push_out_of_sam(26.990, 125.000, briefing)
        assert (lat, lon) == (26.990, 125.000)

    def test_push_out_ignores_jam_zones(self):
        briefing = MagicMock()
        briefing.approximate_zones = (_make_zone(kind="comm_jam_static"),)
        lat, lon = _push_out_of_sam(27.001, 125.000, briefing)
        assert (lat, lon) == (27.001, 125.000), "干扰区不致命，不应推避"


# ── 生命周期与搜索 ──────────────────────────────────────────────────────


class TestLifecycle:
    def test_reset_and_first_decide(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        cmds = agent.decide(_make_obs(), dt=0.1)
        assert isinstance(cmds, list) and len(cmds) > 0

    def test_search_emits_fly_to(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        cmds = None
        for i in range(10):
            cmds = agent.decide(_make_obs(), dt=0.1)
        assert _find_cmd(cmds, "set_destination") is not None

    def test_search_sweeps_gimbal(self):
        """SEARCH 应输出云台扫描命令。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        cmds = agent.decide(_make_obs(), dt=0.1)
        assert _find_cmd(cmds, "component.gimbal_tracking.set_orientation") is not None

    def test_search_avoids_sam(self):
        """搜索航点落在 SAM bbox 内时，fly_to 目的地应被推出 bbox。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        agent.decide(_make_obs(), dt=0.1)  # 触发螺旋航点生成
        zone = _make_zone(bbox=((26.990, 124.990), (27.020, 125.020)))
        for i in range(5):
            cmds = agent.decide(_make_obs(zones=(zone,)), dt=0.1)
            fly = _find_cmd(cmds, "set_destination")
            assert fly is not None
            la, lo = fly.params["latitude"], fly.params["longitude"]
            inside = (26.990 <= la <= 27.020) and (124.990 <= lo <= 125.020)
            assert not inside, f"航点应避开 SAM bbox，实际 ({la}, {lo})"


# ── VERIFY 速度带判别 ──────────────────────────────────────────────────


class TestVerify:
    def test_detection_triggers_verify(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent.decide(
            _make_obs(detected=True, target_lat=27.005, target_lon=125.005), dt=0.1
        )
        assert agent._state.value == "VERIFY"

    def test_fast_target_fast_passes_to_track(self):
        """9 m/s 目标（高速带 [5.5,9.0] 上界）应经快速通过进 TRACK，
        早于 120 样本窗口。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 9 m/s 东移：每帧 Δlon = 9*0.1/(111320*cos27°) ≈ 9.07e-6°
        track_frame = None
        for i in range(120):
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 9.07e-6,
                ),
                dt=0.1,
            )
            if agent._state.value == "TRACK":
                track_frame = i
                break
        assert track_frame is not None, "9 m/s 目标应进 TRACK"
        assert track_frame < 110, f"应早于 120 样本窗口，实际第 {track_frame} 帧"

    def test_5mps_rejected_as_decoy(self):
        """5 m/s 类（4 真 + 20 诱饵运动不可分）按诱饵判否：
        不应快速通过，满 120 样本 OLS=5.0 出界后回 SEARCH。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 5 m/s 东移：每帧 Δlon ≈ 5.04e-6°
        for i in range(130):
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 5.04e-6,
                ),
                dt=0.1,
            )
            assert agent._state.value != "TRACK", "5 m/s 类不应进 TRACK"
        assert agent._state.value == "SEARCH", "满窗判否后应回 SEARCH"

    def test_slow_4mps_passes_via_ols(self):
        """4 m/s 档真目标不能快速通过（低速带被 v0 爬坡污染），
        应由 120 样本 OLS（~4.0 < 4.5）兜底判真进 TRACK。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 4 m/s 东移：每帧 Δlon ≈ 4.03e-6°
        for i in range(130):
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 4.03e-6,
                ),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "4 m/s 档应经 OLS 判真进 TRACK"

    def test_verify_fly_point_pushed_out_of_sam(self):
        """VERIFY 的飞行目的点落在 SAM bbox 内时应被推出。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        zone = _make_zone(bbox=((27.000, 125.000), (27.010, 125.010)))
        agent.decide(
            _make_obs(detected=True, target_lat=27.005, target_lon=125.005), dt=0.1
        )
        assert agent._state.value == "VERIFY"
        cmds = agent.decide(
            _make_obs(
                detected=True, target_lat=27.005, target_lon=125.005, zones=(zone,)
            ),
            dt=0.1,
        )
        fly = _find_cmd(cmds, "set_destination")
        assert fly is not None
        la, lo = fly.params["latitude"], fly.params["longitude"]
        inside = (27.000 <= la <= 27.010) and (125.000 <= lo <= 125.010)
        assert not inside, f"VERIFY 目的点应避开 SAM bbox，实际 ({la}, {lo})"


# ── K=3 协同：槽位、仲裁、协锁 ──────────────────────────────────────────


class TestJoinSlots:
    def test_announce_triggers_join_when_slots_free(self):
        agent = SwarmSearchAgent("uav_4")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (_msg("A:27.010,125.010", "uav_1"),)
        cmds = agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "JOIN"
        assert _find_cmd(cmds, "set_destination") is not None

    def test_join_blocked_when_two_wings_claimed(self):
        """已有两架僚机 C: 占位（槽位满）时，第三架不应进 JOIN。"""
        agent = SwarmSearchAgent("uav_4")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (
            _msg("A:27.010,125.010", "uav_1"),
            _msg("C:27.010,125.010", "uav_2"),
            _msg("C:27.010,125.010", "uav_3"),
        )
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "SEARCH", "槽位满（2 架僚机）应继续搜索"

    def test_join_proceeds_with_one_wing_claimed(self):
        """只有一架僚机占位时仍有空位，应进 JOIN。"""
        agent = SwarmSearchAgent("uav_4")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (
            _msg("A:27.010,125.010", "uav_1"),
            _msg("C:27.010,125.010", "uav_2"),
        )
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "JOIN"

    def test_join_tiebreak_backoff_when_two_smaller_holders(self):
        """收敛竞态：已有两架 uid 更小的僚机占位时，uid 大者退让回 SEARCH。"""
        agent = SwarmSearchAgent("uav_4")
        agent.reset()
        agent._state = agent._state.JOIN
        agent._target = (27.005, 125.005)
        agent._shared_target = (27.005, 125.005)
        agent._join_time = 0.0
        inbox = (
            _msg("C:27.005,125.005", "uav_2"),
            _msg("C:27.005,125.005", "uav_3"),
        )
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "SEARCH", "uid 大者应退让"
        assert agent._shared_target is None

    def test_join_broadcasts_claim_c(self):
        """JOIN 中应以 ~2Hz 广播 C: 占位（不再是 J:，避免与干扰预警冲突）。"""
        agent = SwarmSearchAgent("uav_2")
        agent.reset()
        agent._state = agent._state.JOIN
        agent._target = (27.005, 125.005)
        agent._join_time = 0.0
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        cmds = agent.decide(_make_obs(), dt=0.1)
        payloads = _payloads(cmds)
        assert any(p.startswith("C:") for p in payloads), f"应有 C: 占位，实际: {payloads}"
        assert not any(p.startswith("J:") for p in payloads), "未受干扰不应发 J:"

    def test_search_skips_verify_when_fully_manned(self):
        """SEARCH 检测到的目标长机 + 双僚已齐（T:+C:+C:）时不进 VERIFY。"""
        agent = SwarmSearchAgent("uav_4")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (
            _msg("T:27.001,125.001", "uav_1"),
            _msg("C:27.001,125.001", "uav_2"),
            _msg("C:27.001,125.001", "uav_3"),
        )
        obs = _make_obs(
            detected=True, target_lat=27.001, target_lon=125.001, comm_inbox=inbox
        )
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH", "长僚已齐的目标不应再进 VERIFY"


class TestCoLock:
    def _enter_track_solo_dwell(self, agent):
        """进入 TRACK 且本地 dwell 即将满 20s（单机）。"""
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 10.0
        agent._dwell_time = 19.95  # 本帧 +0.1 后满 20s
        agent._track_time = 19.95
        agent._last_det_time = 10.0
        agent._filter = None

    def test_solo_dwell_does_not_mark_destroyed(self):
        """单机 dwell 满 20s 不算摧毁（K=3 协锁）——继续盯防等僚机。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK", "单机满 20s 应继续盯防"
        assert (27.005, 125.005) not in agent._known_destroyed

    def test_one_teammate_still_not_destroyed(self):
        """只有一名队友在场（双机）也不算摧毁——K=3 需两名队友。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        inbox = (_msg("C:27.005,125.005", "uav_2"),)
        obs = _make_obs(
            detected=True, target_lat=27.005, target_lon=125.005, comm_inbox=inbox
        )
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK", "双机满 20s 应继续盯防等第三机"
        assert (27.005, 125.005) not in agent._known_destroyed

    def test_two_teammates_marks_destroyed_and_broadcasts(self):
        """dwell 满 20s 且两名队友在场（T: 长机 + C: 僚机）→ 判定摧毁：
        标记 + D: 广播 + 回 SEARCH。"""
        agent = SwarmSearchAgent("uav_3")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        inbox = (
            _msg("T:27.005,125.005", "uav_1"),
            _msg("C:27.005,125.005", "uav_2"),
        )
        obs = _make_obs(
            detected=True, target_lat=27.005, target_lon=125.005, comm_inbox=inbox
        )
        cmds = agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH", "三机协锁满 20s 应回 SEARCH"
        assert (27.005, 125.005) in agent._known_destroyed
        assert any(p.startswith("D:") for p in _payloads(cmds)), "应广播 D: 摧毁通知"

    def test_wingman_track_broadcasts_claim(self):
        """僚机 TRACK 应广播 C: 占位（而不是 T:）。"""
        agent = SwarmSearchAgent("uav_2")
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        agent._last_report_time = 0.0
        agent._dwell_time = 5.0
        agent._track_time = 5.0
        agent._last_det_time = 10.0
        agent._filter = None
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        payloads = _payloads(cmds)
        assert any(p.startswith("C:") for p in payloads), f"应有 C: 占位，实际: {payloads}"
        assert not any(p.startswith("T:") for p in payloads), "僚机不应再发 T:"

    def test_wingman_loiter_beyond_penalty_line(self):
        """僚机盘旋半径按 uid 哈希 300m 或 500m，与长机 100m 圈间距 ≥200m。"""
        agent = SwarmSearchAgent("uav_2")
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 0.0
        agent._dwell_time = 5.0
        agent._track_time = 5.0
        agent._last_det_time = 10.0
        agent._filter = None
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        fly = _find_cmd(cmds, "set_destination")
        assert fly is not None
        assert fly.params["loiter_radius"] >= 300.0


# ── 通信：D: 解析、已摧毁记忆、干扰预警 ──────────────────────────────────


class TestComms:
    def test_destroyed_message_marks_target(self):
        """收到 D: 消息应同步进已摧毁列表（不再判别该目标）。"""
        agent = SwarmSearchAgent("uav_3")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent.decide(
            _make_obs(comm_inbox=(_msg("D:27.010,125.010", "uav_1"),)), dt=0.1
        )
        assert (27.010, 125.010) in agent._known_destroyed

    def test_destroyed_target_not_reverified(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent._known_destroyed = [(27.005, 125.005)]
        obs = _make_obs(detected=True, target_lat=27.0051, target_lon=125.0051)
        for i in range(3):
            agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH"

    def test_jammed_broadcasts_warning(self):
        """obs.self.jammed 翻真时应广播 J:lat,lon 干扰预警。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        cmds = agent.decide(_make_obs(jammed=True), dt=0.1)
        assert any(p.startswith("J:") for p in _payloads(cmds)), "被干扰时应广播 J: 预警"

    def test_jam_warning_rate_limited(self):
        """干扰预警应限频 ~1Hz。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        n_bc = 0
        for i in range(20):  # 2s 连续被干扰
            cmds = agent.decide(_make_obs(jammed=True), dt=0.1)
            n_bc += sum(1 for p in _payloads(cmds) if p.startswith("J:"))
        assert n_bc <= 3, f"干扰预警应限频（~1Hz），实际 2s 内 {n_bc} 次"

    def test_position_heartbeat_broadcast(self):
        """应以 ~1Hz 广播 P:lat,lon 位置心跳（proximity 避让用）。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        n_hb = 0
        for i in range(30):  # 3s
            cmds = agent.decide(_make_obs(), dt=0.1)
            n_hb += sum(1 for p in _payloads(cmds) if p.startswith("P:"))
        assert 1 <= n_hb <= 4, f"3s 内心跳应 1~4 次，实际 {n_hb}"


# ── 命令合法性与递归保护 ──────────────────────────────────────────────────


class TestCommandValidity:
    def test_no_unknown_verbs(self):
        known_verbs = {
            "set_destination",
            "set_heading",
            "set_speed",
            "component.gimbal_tracking.set_orientation",
            "set_fov",
            "comm.broadcast",
            "comm.send",
            "agent.report",
        }
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(15):
            cmds = agent.decide(
                _make_obs(detected=(i > 8), target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
            for cmd in cmds:
                assert cmd.verb in known_verbs, f"未知 verb: {cmd.verb}"

    def test_dispatch_depth_capped(self):
        """限深保护：人为制造状态振荡时 _dispatch 不超过深度上限。"""
        from competition.user_algorithms.adversarial_swarm.agent import State

        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        calls = []

        def oscillate(obs, dt):
            calls.append(1)
            agent._state = State.SEARCH if len(calls) % 2 else State.VERIFY
            return agent._dispatch(obs, dt)

        agent._do_search = oscillate
        agent._do_verify = oscillate
        cmds = agent.decide(_make_obs(), dt=0.1)
        assert isinstance(cmds, list)
        assert len(calls) <= 7  # 顶层层 + 限深 6

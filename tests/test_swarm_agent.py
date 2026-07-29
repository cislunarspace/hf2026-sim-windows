"""tests/test_swarm_agent.py — 赛题三对抗集群 Agent 测试。

使用 mock obs 验证状态机、SAM 绕行、干扰广播与已摧毁记忆。
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
    jammed: bool = False,
    zones: tuple = (),
    comm_inbox: tuple = (),
) -> MagicMock:
    obs = MagicMock()
    obs.self.uid = "uav_1"
    obs.self.lat = lat
    obs.self.lon = lon
    obs.self.alt = alt
    obs.self.heading_deg = 0.0
    obs.self.speed = 30.0
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


def _find_cmd(cmds, verb: str):
    for cmd in cmds:
        if isinstance(cmd, Command) and cmd.verb == verb:
            return cmd
    return None


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

    def test_search_avoids_sam(self):
        """搜索航点落在 SAM bbox 内时，fly_to 目的地应被推出 bbox。"""
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        # 强制航点生成后，把所有航点塞进 SAM bbox 中央
        agent.decide(_make_obs(), dt=0.1)
        zone = _make_zone(bbox=((26.990, 124.990), (27.020, 125.020)))
        for i in range(5):
            cmds = agent.decide(_make_obs(zones=(zone,)), dt=0.1)
            fly = _find_cmd(cmds, "set_destination")
            assert fly is not None
            la, lo = fly.params["latitude"], fly.params["longitude"]
            inside = (26.990 <= la <= 27.020) and (124.990 <= lo <= 125.020)
            assert not inside, f"航点应避开 SAM bbox，实际 ({la}, {lo})"


# ── 通信与协同 ──────────────────────────────────────────────────────────


class TestComms:
    def test_announce_triggers_join(self):
        agent = SwarmSearchAgent("uav_2")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_1"
        cmds = agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._state.value == "JOIN"
        assert _find_cmd(cmds, "set_destination") is not None

    def test_jammed_broadcasts_warning(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        cmds = agent.decide(_make_obs(jammed=True), dt=0.1)
        bc = _find_cmd(cmds, "comm.broadcast")
        assert bc is not None, "被干扰时应广播预警"
        assert bc.params["payload"].startswith("J:")

    def test_jam_warning_rate_limited(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        n_bc = 0
        for i in range(20):  # 2s 连续被干扰
            cmds = agent.decide(_make_obs(jammed=True), dt=0.1)
            bc = _find_cmd(cmds, "comm.broadcast")
            if bc is not None and bc.params["payload"].startswith("J:"):
                n_bc += 1
        assert n_bc <= 3, f"干扰预警应限频（~1Hz），实际 2s 内 {n_bc} 次"


# ── VERIFY 与已摧毁记忆 ────────────────────────────────────────────────


class TestVerifyAndDestroyed:
    def test_detection_triggers_verify(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent.decide(_make_obs(detected=True, target_lat=27.005, target_lon=125.005), dt=0.1)
        assert agent._state.value == "VERIFY"

    def test_dwell_complete_marks_destroyed(self):
        agent = SwarmSearchAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 10.0
        agent._dwell_time = 19.95
        agent._track_time = 19.95
        agent._last_det_time = 10.0
        agent._imm = None

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH"
        assert (27.005, 125.005) in agent._known_destroyed

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


# ── 命令合法性 ──────────────────────────────────────────────────────────


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

"""tests/test_coop_decoy_agent.py — 赛题二协同 Agent 集成测试。

使用 mock obs 验证状态机行为、命令生成和通信。
"""
from typing import Optional, Tuple
from unittest.mock import MagicMock

import pytest

try:
    from competition.user_algorithms.coop_decoy.agent import CoopDecoyAgent
    from competition.sdk.core.commands import Command
except ImportError:
    pytest.skip("coop_decoy agent 尚未实现，跳过测试", allow_module_level=True)


# ── Mock 工厂 ──────────────────────────────────────────────────────────

def _make_obs(
    lat: float = 27.0,
    lon: float = 125.0,
    alt: float = 200.0,
    detected: bool = False,
    target_lat: Optional[float] = None,
    target_lon: Optional[float] = None,
    heading: float = 0.0,
    speed: float = 25.0,
    gimbal_pan: float = 0.0,
    gimbal_tilt: float = -45.0,
    gimbal_fov: float = 70.0,
    fleet_size: int = 3,
    comm_inbox: tuple = (),
) -> MagicMock:
    """创建 mock CoopObs。"""
    obs = MagicMock()
    obs.self.uid = "uav_1"
    obs.self.lat = lat
    obs.self.lon = lon
    obs.self.alt = alt
    obs.self.heading_deg = heading
    obs.self.speed = speed
    obs.self.gimbal_pan = gimbal_pan
    obs.self.gimbal_tilt = gimbal_tilt
    obs.self.gimbal_fov_deg = gimbal_fov
    obs.self.status = "active"
    obs.self.jammed = False

    # Detection mock
    obs.self.detection.detected = detected
    obs.self.detection.confidence = 0.8 if detected else 0.0
    obs.self.detection.target_lat = target_lat if detected else None
    obs.self.detection.target_lon = target_lon if detected else None
    obs.self.detection.target_type = None

    # Briefing mock
    obs.briefing.fleet_size = fleet_size
    obs.briefing.target_initial_pos = None
    obs.briefing.target_count = 5
    obs.briefing.params = {
        "coop_k": 2,
        "sector_center_lat": 27.0,
        "sector_center_lon": 125.0,
    }

    # Comms
    obs.comm_inbox = comm_inbox

    return obs


def _find_cmd(cmds, verb: str):
    """在命令列表中查找指定 verb 的命令。"""
    for cmd in cmds:
        if isinstance(cmd, Command) and cmd.verb == verb:
            return cmd
    return None


# ── 测试 ──────────────────────────────────────────────────────────────

class TestCoopAgentLifecycle:
    """生命周期测试。"""

    def test_reset(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()  # 不应抛异常

    def test_first_decide_returns_commands(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        obs = _make_obs()
        cmds = agent.decide(obs, dt=0.1)
        assert isinstance(cmds, list)
        assert len(cmds) > 0


class TestCoopAgentSearch:
    """搜索阶段测试。"""

    def test_search_emits_fly_to(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        cmds = None
        for i in range(20):
            obs = _make_obs(lat=27.0 + i * 0.0001, lon=125.0 + i * 0.0001)
            cmds = agent.decide(obs, dt=0.1)
        assert cmds is not None
        fly_cmd = _find_cmd(cmds, "set_destination")
        assert fly_cmd is not None, "搜索阶段应有 fly_to 命令"

    def test_search_no_broadcast(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(10):
            obs = _make_obs()
            cmds = agent.decide(obs, dt=0.1)
            bc = _find_cmd(cmds, "comm.broadcast")
            assert bc is None, "搜索阶段不应广播"


class TestCoopAgentVerify:
    """VERIFY 阶段测试。"""

    def test_detection_triggers_verify(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        # 搜索几帧
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 检测到目标
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        # 应有云台命令（锁定目标）
        gimbal_cmd = _find_cmd(cmds, "component.gimbal_tracking.set_orientation")
        assert gimbal_cmd is not None, "VERIFY 阶段应有云台命令"


class TestCoopAgentComms:
    """通信测试。"""

    def test_shared_target_triggers_join(self):
        """收到队友广播后应切换到 JOIN。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        # 搜索几帧
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 收到队友确认目标
        msg = MagicMock()
        msg.payload = "T:27.01000,125.01000"
        msg.sender_uid = "uav_2"
        obs = _make_obs(comm_inbox=(msg,))
        cmds = agent.decide(obs, dt=0.1)
        # 应有飞行命令（飞向共享目标）
        fly_cmd = _find_cmd(cmds, "set_destination")
        assert fly_cmd is not None, "JOIN 状态应有飞行命令"

    def test_broadcast_format(self):
        """长机首次 TRACK 应广播 A:lat,lon（announce）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 手动设置状态到 TRACK（长机）
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        agent._last_report_time = 0.0
        agent._dwell_time = 0.0
        agent._track_time = 0.0
        agent._last_det_time = 10.0

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        bc = _find_cmd(cmds, "comm.broadcast")
        assert bc is not None, "TRACK 阶段应广播"
        payload = bc.params["payload"]
        assert payload.startswith("A:"), f"长机首次应 announce，实际: {payload}"

    def test_announce_triggers_join(self):
        """收到 A: 消息后空闲 UAV 应进入 JOIN。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)

        # 收到 announce 消息
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_1"
        obs = _make_obs(comm_inbox=(msg,))
        cmds = agent.decide(obs, dt=0.1)
        fly_cmd = _find_cmd(cmds, "set_destination")
        assert fly_cmd is not None, "收到 announce 应飞行向目标"

    def test_wingman_uses_larger_loiter(self):
        """僚机应使用更大盘旋半径。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        # 手动设置为僚机 TRACK
        agent._state = agent._state.TRACK
        agent._is_wingman = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        agent._last_report_time = 0.0
        agent._dwell_time = 5.0
        agent._track_time = 5.0
        agent._last_det_time = 10.0
        agent._imm = None

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        fly_cmd = _find_cmd(cmds, "set_destination")
        if fly_cmd and "loiter_radius" in fly_cmd.params:
            assert fly_cmd.params["loiter_radius"] >= 200.0, \
                f"僚机盘旋半径应 >= 200m，实际 {fly_cmd.params['loiter_radius']}"


class TestCoopAgentCommands:
    """命令合法性测试。"""

    def test_commands_are_command_instances(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(10):
            obs = _make_obs(lat=27.0 + i * 0.0001)
            cmds = agent.decide(obs, dt=0.1)
            for cmd in cmds:
                assert isinstance(cmd, Command), f"命令应为 Command 实例: {cmd}"

    def test_no_unknown_verbs(self):
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        known_verbs = {
            "set_destination", "set_heading", "set_speed",
            "component.gimbal_tracking.set_orientation",
            "set_fov",
            "comm.broadcast", "comm.send", "agent.report",
        }
        for i in range(10):
            obs = _make_obs(lat=27.0 + i * 0.0001)
            cmds = agent.decide(obs, dt=0.1)
            for cmd in cmds:
                if isinstance(cmd, Command):
                    assert cmd.verb in known_verbs, f"未知 verb: {cmd.verb}"

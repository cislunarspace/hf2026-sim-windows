"""tests/test_search_track_agent.py — 搜索-跟踪 Agent 集成测试。

使用 mock obs 验证状态机行为和命令生成。
"""

from unittest.mock import MagicMock

import pytest

try:
    from competition.sdk.core.commands import Command
    from competition.user_algorithms.search_track.my_agent import MySearchTrackAgent
except ImportError:
    pytest.skip("my_agent.py 尚未实现，跳过测试", allow_module_level=True)


# ── Mock 工厂 ──────────────────────────────────────────────────────────


def _make_obs(
    lat: float = 27.0,
    lon: float = 125.0,
    alt: float = 300.0,
    detected: bool = False,
    target_lat: float | None = None,
    target_lon: float | None = None,
    target_initial_pos: tuple[float, float] | None = (27.005, 125.005),
    heading: float = 0.0,
    speed: float = 30.0,
    gimbal_pan: float = 0.0,
    gimbal_tilt: float = -45.0,
    gimbal_fov: float = 70.0,
) -> MagicMock:
    """创建 mock SearchTrackObs。"""
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

    # Detection mock
    obs.self.detection.detected = detected
    obs.self.detection.confidence = 0.8 if detected else 0.0
    obs.self.detection.target_lat = target_lat if detected else None
    obs.self.detection.target_lon = target_lon if detected else None
    obs.self.detection.azimuth_error_deg = None
    obs.self.detection.target_type = "ground_vehicle" if detected else ""

    # Briefing
    obs.briefing.self_uid = "uav_1"
    obs.briefing.fleet_size = 1
    obs.briefing.target_initial_pos = target_initial_pos
    obs.briefing.params = {"target_speed": 8.0}

    # Comm
    obs.comm_inbox = ()

    return obs


def _find_cmd(cmds, verb: str) -> Command | None:
    """在命令列表中查找指定 verb 的命令。"""
    for cmd in cmds:
        if cmd.verb == verb:
            return cmd
    return None


# ── 状态机行为测试 ──────────────────────────────────────────────────────


class TestAgentLifecycle:
    """Agent 生命周期测试。"""

    def test_reset(self):
        agent = MySearchTrackAgent("uav_1")
        # 调用 reset 不应抛异常
        agent.reset()

    def test_first_decide_returns_commands(self):
        """第一次 decide 应返回非空命令列表。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        obs = _make_obs()
        cmds = agent.decide(obs, dt=0.1)
        assert isinstance(cmds, list)
        assert len(cmds) > 0, "decide 应返回至少一个命令"


class TestSearchPhase:
    """搜索阶段行为测试。"""

    def test_search_emits_fly_and_gimbal(self):
        """搜索阶段应产生 fly_to 和 point_gimbal 命令。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()

        # 连续调用多次（模拟飞行过程）
        cmds = None
        for i in range(20):
            obs = _make_obs(lat=27.0 + i * 0.0001, lon=125.0 + i * 0.0001)
            cmds = agent.decide(obs, dt=0.1)

        assert cmds is not None
        verbs = {cmd.verb for cmd in cmds}
        # 搜索阶段至少要有飞行和云台命令之一
        assert (
            "set_destination" in verbs
            or "component.gimbal_tracking.set_orientation" in verbs
        ), f"搜索阶段命令集: {verbs}，应含 fly_to 或 point_gimbal"

    def test_search_reports_every_second(self):
        """搜索阶段也必须每秒上报（评分漏报记 0；目标出生点 30s 停驶可得分）。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        # mock 的 target_initial_pos (27.005,125.005) 不匹配任何路线 →
        # 回退上报出生点；首拍即应出现 report_target
        cmds = agent.decide(_make_obs(), dt=0.1)
        assert _find_cmd(cmds, "agent.report") is not None, "搜索首拍应上报出生点"
        # 之后以 1s 节拍持续上报
        reports = 0
        for i in range(30):  # 3s
            cmds = agent.decide(_make_obs(), dt=0.1)
            if _find_cmd(cmds, "agent.report") is not None:
                reports += 1
        assert reports >= 2, f"3s 内上报 {reports} 次，应 ≥2"

    def test_matched_route_reports_predicted_position(self):
        """briefing 出生点匹配上路线时，SEARCH 应沿路线预测位置上报。"""
        from algorithms.search.route_prior import ROUTES

        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        start = ROUTES[3][0]
        obs = _make_obs(target_initial_pos=(start[0], start[1]))
        cmds = agent.decide(obs, dt=0.1)
        report = _find_cmd(cmds, "agent.report")
        assert report is not None
        # t≈0 预测位置即路线 Start（WaitTime=30s 内不动）
        assert abs(report.params["lat"] - start[0]) < 1e-4
        assert abs(report.params["lon"] - start[1]) < 1e-4


class TestTrackPhase:
    """跟踪阶段行为测试。"""

    def test_detection_enters_engage_immediately(self):
        """赛题一无诱饵：检测到目标应当拍进 ENGAGE，不做 VERIFY 空转。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        agent.decide(obs, dt=0.1)
        agent.decide(obs, dt=0.1)
        assert agent._state.value in ("engage", "attack")

    def test_detection_triggers_tracking(self):
        """持续检测到快速移动目标后应进入跟踪模式并上报。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()

        # 先搜索几帧
        for i in range(5):
            obs = _make_obs(lat=27.0 + i * 0.0001)
            agent.decide(obs, dt=0.1)

        # 目标以 ~30 m/s 向东运动；赛题一无 VERIFY，检测到即进 ENGAGE，
        # EKF 收敛 + 1s 上报节拍 → 130 帧内必出现 report
        target_lon_offset = 0.0
        found_report = False
        for i in range(130):
            target_lon_offset += 0.00003  # ~3 m/s east
            uav_lat = 27.0 + i * 0.0002  # UAV 向北运动
            obs = _make_obs(
                lat=uav_lat,
                lon=125.0,
                detected=True,
                target_lat=27.005,
                target_lon=125.005 + target_lon_offset,
            )
            cmds = agent.decide(obs, dt=0.1)
            if _find_cmd(cmds, "agent.report") is not None:
                found_report = True

        # 跟踪阶段应产生过 report_target 命令
        assert found_report, "跟踪阶段应产生 report_target 命令"

    def test_report_target_contains_lat_lon(self):
        """report_target 命令应包含 lat 和 lon。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()

        # 快速进入跟踪（移动目标，避免 VERIFY 拒绝）
        target_east_offset = 0.0
        for i in range(50):
            target_east_offset += 0.00001
            obs = _make_obs(
                detected=True,
                target_lat=27.005,
                target_lon=125.005 + target_east_offset,
            )
            cmds = agent.decide(obs, dt=0.1)

        report_cmd = _find_cmd(cmds, "agent.report")
        if report_cmd is not None:
            assert "lat" in report_cmd.params
            assert "lon" in report_cmd.params
            assert isinstance(report_cmd.params["lat"], float)
            assert isinstance(report_cmd.params["lon"], float)


class TestLostRecovery:
    """LOST 恢复测试（回归：曾在最后位置不放弃，直接 _wp_idx=0 从头螺旋）。"""

    def _enter_tracking(self, agent):
        """喂移动目标帧直到进入 ENGAGE/ATTACK。"""
        for i in range(100):
            obs = _make_obs(
                detected=True, target_lat=27.005, target_lon=125.005 + i * 0.00003
            )
            agent.decide(obs, dt=0.1)
        assert agent._state.value in ("engage", "attack")

    def test_lost_loiters_at_last_estimate(self):
        """丢失后应在最后估计位置盘旋等待重捕获，螺旋进度不重置。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        self._enter_tracking(agent)
        agent._wp_idx = 5

        cmds = None
        for i in range(25):  # 丢失 2.5s（> 2s 容忍 → LOST）
            cmds = agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "lost"
        assert agent._wp_idx == 5, "LOST 不应重置螺旋进度"
        fly = _find_cmd(cmds, "set_destination")
        assert fly is not None, "LOST 应飞向最后估计位置盘旋"

    def test_lost_reacquire_goes_engage(self):
        """LOST 中重新检测到目标应回 ENGAGE（赛题一无诱饵判别，直接恢复跟踪）。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        self._enter_tracking(agent)
        for i in range(25):
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "lost"

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.009)
        agent.decide(obs, dt=0.1)
        # EKF 仍热时可能同帧从 ENGAGE 直接进 ATTACK
        assert agent._state.value in ("engage", "attack")

    def test_lost_timeout_returns_to_search(self):
        """LOST 超过重捕获时限仍无检测 → 回 SEARCH。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()
        self._enter_tracking(agent)
        for i in range(150):  # 15s 无检测（> 10s 重捕获时限）
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "search"


class TestCommandValidity:
    """命令合法性测试。"""

    def test_commands_only_address_self(self):
        """所有命令只影响自身（SDK 约束）。"""
        agent = MySearchTrackAgent("uav_1")
        agent.reset()

        for i in range(10):
            obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
            cmds = agent.decide(obs, dt=0.1)
            for cmd in cmds:
                assert isinstance(cmd, Command), f"非 Command 对象: {type(cmd)}"

    def test_no_unknown_verbs(self):
        """不应产生 SDK 不支持的命令类型。"""
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
        agent = MySearchTrackAgent("uav_1")
        agent.reset()

        for i in range(20):
            obs = _make_obs(detected=(i > 10), target_lat=27.005, target_lon=125.005)
            cmds = agent.decide(obs, dt=0.1)
            for cmd in cmds:
                assert cmd.verb in known_verbs, f"未知命令 verb: {cmd.verb}"

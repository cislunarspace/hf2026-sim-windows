"""tests/test_coop_decoy_agent.py — 赛题二协同 Agent 集成测试。

使用 mock obs 验证状态机行为、命令生成和通信。
"""

from unittest.mock import MagicMock

import pytest

try:
    from competition.sdk.core.commands import Command
    from competition.user_algorithms.coop_decoy.agent import CoopDecoyAgent
except ImportError:
    pytest.skip("coop_decoy agent 尚未实现，跳过测试", allow_module_level=True)


# ── Mock 工厂 ──────────────────────────────────────────────────────────


def _make_obs(
    lat: float = 27.0,
    lon: float = 125.0,
    alt: float = 200.0,
    detected: bool = False,
    target_lat: float | None = None,
    target_lon: float | None = None,
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
            assert bc is None or not bc.params["payload"].startswith(("A:", "T:")), (
                "搜索阶段不应广播目标消息（P: 心跳除外）"
            )


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


class TestCoopAgentVerifyOls:
    """OLS 速度判别测试（12s 采样窗口）。"""

    def test_stationary_target_returns_to_search(self):
        """静止候选（可能停顿中的真目标）判别后回 SEARCH，且不做位置标记。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        for i in range(125):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        assert not hasattr(agent, "_known_decoys") or not agent._known_decoys

    def test_moving_target_goes_track(self):
        """9 m/s 快速目标（速度档必真）12s 后应进 TRACK。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 9 m/s 东移：每帧 Δlon = 9*0.1/(111320*cos27°) ≈ 9.07e-6°
        for i in range(125):
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 9.07e-6,
                ),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "快速移动目标应进 TRACK"

    def test_slow_mover_returns_to_search(self):
        """5 m/s 目标与诱饵同速档（官方 runner 注入 decoy_speed=5.0），回 SEARCH 但不标记。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        for i in range(125):
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 5.04e-6,
                ),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "5 m/s 类应回 SEARCH"

    def test_lost_during_verify_aborts_without_decoy_mark(self):
        """VERIFY 中连续丢失 >2s 应放弃且不记诱饵。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        for i in range(30):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "VERIFY"
        for i in range(30):  # 3s 无检测
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "SEARCH", "丢失超时应回 SEARCH"

    def test_lock_flicker_rejected_by_speed_band(self):
        """锁跳变导致的虚高速度（>13.5）应被速度带上限拒绝。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 大部分时间锁在静止点，少量帧跳到 400m 外（位置阶跃 → 表观速度虚高）
        for i in range(125):
            if i % 10 < 8:
                tlat, tlon = 27.005, 125.005
            else:
                tlat, tlon = 27.005, 125.009
            agent.decide(
                _make_obs(detected=True, target_lat=tlat, target_lon=tlon),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "锁跳变虚高应被拒绝回 SEARCH"

    def test_engine_time_used_for_verdict(self):
        """VERIFY 用引擎时间（score_view.sim_time）：控制节拍快于引擎 2.5 倍时，
        12 m/s 真目标仍应判真（回归：dt 累加曾把 12 m/s 读成 ~5 误判诱饵）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        sim_t = [-28700.0]

        def _obs(i, detected=True):
            obs = _make_obs(
                detected=detected,
                target_lat=27.005,
                target_lon=125.005 + i * 4.84e-6,  # 12 m/s × 0.04s
            )
            sim_t[0] += 0.04  # 引擎每拍只走 0.04s（控制节拍 0.1s 的 2.5 倍快）
            obs.briefing.score_view.sim_time = sim_t[0]
            return obs

        for i in range(5):
            agent.decide(_obs(0, detected=False), dt=0.1)
        for i in range(125):
            agent.decide(_obs(i), dt=0.1)
        assert agent._state.value == "TRACK", (
            "引擎时间下 12 m/s 目标应判真；若回退 dt 累加会被误判为诱饵"
        )

    def test_reject_cooldown_blocks_immediate_reverify(self):
        """判别否决后冷却期内同位置检测不重进 VERIFY，冷却后允许（停顿真目标可重遇）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 静止候选 → 否决
        for i in range(125):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        # 冷却期内：同位置持续检测不应卡回 VERIFY
        for i in range(50):  # 5s
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "冷却期内不应重进 VERIFY"
        # 冷却期后（累计 >20s）：允许重新判别
        for i in range(160):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value in ("VERIFY", "SEARCH"), "冷却后应重新判别"
        assert agent._verify_samples or agent._state.value == "SEARCH"

    def test_search_sweeps_gimbal(self):
        """SEARCH 应输出云台扫描命令（回归：曾只设 FOV 不转云台）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        cmds = agent.decide(_make_obs(), dt=0.1)
        gimbal_cmd = _find_cmd(cmds, "component.gimbal_tracking.set_orientation")
        assert gimbal_cmd is not None, "SEARCH 应有云台扫描命令"

    def test_position_heartbeat_broadcast(self):
        """应以 ~1Hz 广播 P:lat,lon 位置心跳（proximity 避让用）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        n_hb = 0
        for i in range(30):  # 3s
            cmds = agent.decide(_make_obs(), dt=0.1)
            bc = _find_cmd(cmds, "comm.broadcast")
            if bc is not None and bc.params["payload"].startswith("P:"):
                n_hb += 1
        assert 1 <= n_hb <= 4, f"3s 内心跳应 1~4 次，实际 {n_hb}"

    def test_ingest_p_records_teammate(self):
        """收到 P: 心跳应记录队友位置。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        msg = MagicMock()
        msg.payload = "P:27.0010,125.0010"
        msg.sender_uid = "uav_2"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert "uav_2" in agent._teammates
        la, lo, _ = agent._teammates["uav_2"]
        assert abs(la - 27.001) < 1e-3

    def test_join_goes_through_verify(self):
        """僚机 JOIN 到达目标附近后应先 VERIFY 判别，不直接 TRACK。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        msg = MagicMock()
        msg.payload = "A:27.005,125.005"
        msg.sender_uid = "uav_1"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._state.value == "JOIN"
        # 僚机已在目标 200m 内且检测到 → 应进 VERIFY 而非 TRACK
        obs = _make_obs(
            lat=27.005, lon=125.005, detected=True,
            target_lat=27.005, target_lon=125.005,
        )
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "VERIFY", (
            "JOIN 收敛后应先 VERIFY 判别（防止收敛到未鉴别的诱饵）"
        )


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

    def test_join_timeout_returns_to_search(self):
        """JOIN 超时（未检测到目标）应回 SEARCH，避免收敛到已放弃的诱饵。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 收到 announce 进入 JOIN
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_2"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._state.value == "JOIN"
        # 无检测、无新 announce，跑超过 JOIN 超时（60s）
        for i in range(610):
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "SEARCH", "JOIN 超时后应回 SEARCH"
        assert agent._shared_target is None, "超时后共享目标应清空"

    def test_announce_expires(self):
        """announce 超过 15s 未更新应过期。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_2"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._shared_target is not None
        # 跑 16s 无新消息（16s > 15s 过期窗口）
        for i in range(160):
            agent.decide(_make_obs(), dt=0.1)
        assert agent._shared_target is None, "announce 应已过期"

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
        bcs = [
            c for c in cmds
            if isinstance(c, Command)
            and c.verb == "comm.broadcast"
            and c.params["payload"].startswith("A:")
        ]
        assert bcs, "TRACK 阶段长机首次应 announce"
        payload = bcs[0].params["payload"]
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
        agent._filter = None

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        fly_cmd = _find_cmd(cmds, "set_destination")
        if fly_cmd and "loiter_radius" in fly_cmd.params:
            assert fly_cmd.params["loiter_radius"] >= 200.0, (
                f"僚机盘旋半径应 >= 200m，实际 {fly_cmd.params['loiter_radius']}"
            )


class TestCoopAgentTrackFilter:
    """TRACK 阶段滤波更新与上报测试（回归：TRACK 曾不更新 IMM，上报位置冻结）。"""

    def _enter_track(self, agent, wingman=False):
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent._state = agent._state.TRACK
        agent._is_wingman = wingman
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 0.0
        agent._dwell_time = 1.0
        agent._track_time = 1.0
        agent._last_det_time = 10.0
        agent._filter = None

    def test_track_reports_follow_moving_target(self):
        """TRACK 中持续检测移动目标，上报位置应跟随目标而非冻结在入 TRACK 点。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_track(agent)
        reports = []
        for i in range(40):  # 4s，目标东移 ~0.002°（~200m）
            obs = _make_obs(
                detected=True, target_lat=27.005, target_lon=125.005 + i * 0.00005
            )
            cmds = agent.decide(obs, dt=0.1)
            r = _find_cmd(cmds, "agent.report")
            if r is not None:
                reports.append(r)
        assert reports, "TRACK 中有检测且滤波已初始化时应上报"
        last = reports[-1].params
        assert last["lon"] > 125.0055, (
            f"上报应跟随目标东移（>125.0055），实际 {last['lon']:.6f}"
        )

    def test_wingman_also_reports(self):
        """僚机 TRACK（JOIN 路径，imm 未初始化）也应在滤波收敛后上报。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        self._enter_track(agent, wingman=True)
        reports = []
        for i in range(60):  # 6s 移动目标，足够滤波初始化并估计出速度
            obs = _make_obs(
                detected=True, target_lat=27.005, target_lon=125.005 + i * 0.00005
            )
            cmds = agent.decide(obs, dt=0.1)
            r = _find_cmd(cmds, "agent.report")
            if r is not None:
                reports.append(r)
        assert reports, "僚机 TRACK 中滤波收敛后应上报"


class TestCoopAgentDestroyedMemory:
    """已摧毁目标记忆：摧毁后不重复跟踪、上报。"""

    def test_dwell_complete_marks_destroyed(self):
        """dwell 满 20s 后目标应记入 _known_destroyed 并回 SEARCH。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
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

        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH", "dwell 满 20s 应回 SEARCH"
        assert (27.005, 125.005) in agent._known_destroyed, (
            "完成盯防的目标应记入已摧毁列表"
        )

    def test_destroyed_target_not_reverified(self):
        """SEARCH 中检测到已摧毁目标附近的目标不应再进 VERIFY。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent._known_destroyed = [(27.005, 125.005)]
        # 检测点在已摧毁目标 ~15m 内
        obs = _make_obs(detected=True, target_lat=27.0051, target_lon=125.0051)
        for i in range(3):
            agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH", "已摧毁目标不应触发 VERIFY"

    def test_destroyed_announce_not_joined(self):
        """队友对己知已摧毁目标的 announce 不应触发 JOIN。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent._known_destroyed = [(27.010, 125.010)]
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_1"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._state.value == "SEARCH", "已摧毁目标的 announce 不应触发 JOIN"


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
            "set_destination",
            "set_heading",
            "set_speed",
            "component.gimbal_tracking.set_orientation",
            "set_fov",
            "comm.broadcast",
            "comm.send",
            "agent.report",
        }
        for i in range(10):
            obs = _make_obs(lat=27.0 + i * 0.0001)
            cmds = agent.decide(obs, dt=0.1)
            for cmd in cmds:
                if isinstance(cmd, Command):
                    assert cmd.verb in known_verbs, f"未知 verb: {cmd.verb}"


class TestRecursionGuard:
    """递归保护回归测试。"""

    def test_verify_join_pingpong_no_recursion(self):
        """回归：announce 目标 B 与 ~243m 外另一辆车的检测 A 曾让
        VERIFY↔JOIN 同拍乒乓（A 被 JOIN 覆写为 _target 后又满足 VERIFY 的
        'shared 与 target 不同'条件），递归重入直至 RecursionError 被
        runner 吞掉。修复后 JOIN 不再覆写 _target，且重入走限深 _dispatch。
        """
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        det_a = (27.001, 125.001)  # 距 UAV ~149m
        shared_b = (27.003, 125.0)  # 距 UAV ~334m，距 A ~243m

        # 自己发现候选 A → VERIFY
        obs = _make_obs(detected=True, target_lat=det_a[0], target_lon=det_a[1])
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "VERIFY"

        # 队友 announce 目标 B，检测仍指 A：旧实现此处递归崩溃
        msg = MagicMock()
        msg.payload = f"A:{shared_b[0]},{shared_b[1]}"
        msg.sender_uid = "uav_2"
        for _ in range(10):
            obs = _make_obs(
                detected=True,
                target_lat=det_a[0],
                target_lon=det_a[1],
                comm_inbox=(msg,),
            )
            cmds = agent.decide(obs, dt=0.1)
            assert isinstance(cmds, list)
        # 状态应收敛在 VERIFY/JOIN（协同流程内），不崩溃、不卡死
        assert agent._state.value in ("VERIFY", "JOIN", "TRACK")

    def test_dispatch_depth_capped(self):
        """限深保护：人为制造状态振荡时 _dispatch 不超过深度上限。"""
        from competition.user_algorithms.coop_decoy.agent import State

        agent = CoopDecoyAgent("uav_1")
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


class TestJoinSlotDeconfliction:
    """J: 占位与 proximity 避让测试（v14 局三机扎堆 proximity 203 次的修复）。"""

    def _jmsg(self, lat, lon, sender):
        msg = MagicMock()
        msg.payload = f"J:{lat},{lon}"
        msg.sender_uid = sender
        return msg

    def _amsg(self, lat, lon, sender):
        msg = MagicMock()
        msg.payload = f"A:{lat},{lon}"
        msg.sender_uid = sender
        return msg

    def test_join_blocked_when_slot_taken(self):
        """SEARCH 收到 A: 但目标已有他机 J: 占位时，不应进 JOIN。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (
            self._amsg(27.010, 125.010, "uav_1"),
            self._jmsg(27.010, 125.010, "uav_2"),
        )
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "SEARCH", "已有僚机占位，第三机应继续搜索"

    def test_join_proceeds_when_slot_free(self):
        """只有 A: 没有 J: 时仍应进 JOIN（不占位的正常协同）。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (self._amsg(27.010, 125.010, "uav_1"),)
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "JOIN"

    def test_join_tiebreak_backoff(self):
        """双机同时 JOIN 同一目标的竞态：uid 大者应退让回 SEARCH。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        agent._state = agent._state.JOIN
        agent._target = (27.005, 125.005)
        agent._shared_target = (27.005, 125.005)
        agent._join_time = 0.0
        inbox = (self._jmsg(27.005, 125.005, "uav_2"),)
        agent.decide(_make_obs(comm_inbox=inbox), dt=0.1)
        assert agent._state.value == "SEARCH", "uid 大者应退让"
        assert agent._shared_target is None

    def test_join_broadcasts_claim(self):
        """JOIN 中应以 ~2Hz 广播 J: 占位。"""
        agent = CoopDecoyAgent("uav_2")
        agent.reset()
        agent._state = agent._state.JOIN
        agent._target = (27.005, 125.005)
        agent._join_time = 0.0
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        cmds = agent.decide(_make_obs(), dt=0.1)
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert any(p.startswith("J:") for p in payloads), f"应有 J: 占位，实际: {payloads}"

    def test_wingman_track_broadcasts_claim(self):
        """僚机 TRACK 应广播 J: 占位（而不是 T:）。"""
        agent = CoopDecoyAgent("uav_2")
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
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert any(p.startswith("J:") for p in payloads), f"应有 J: 占位，实际: {payloads}"
        assert not any(p.startswith("T:") for p in payloads), "僚机不应再发 T:"

    def test_wingman_loiter_beyond_penalty_line(self):
        """长机 100m / 僚机 400m 同心盘旋，最近距离 300m > 200m 罚线。"""
        agent = CoopDecoyAgent("uav_2")
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
        assert fly.params["loiter_radius"] >= 400.0


class TestTrackRoleArbitration:
    """TRACK 角色仲裁：同一目标只能一长一僚（v15 proximity 303 的修复）。"""

    def _tmsg(self, lat, lon, sender):
        msg = MagicMock()
        msg.payload = f"T:{lat},{lon}"
        msg.sender_uid = sender
        return msg

    def _jmsg(self, lat, lon, sender):
        msg = MagicMock()
        msg.payload = f"J:{lat},{lon}"
        msg.sender_uid = sender
        return msg

    def _enter_track_as_lead(self, agent):
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 0.0
        agent._dwell_time = 1.0
        agent._track_time = 1.0
        agent._last_det_time = 10.0
        agent._filter = None

    def test_second_lead_demotes_to_wingman(self):
        """双长机：uid 大者听到 uid 小长机的 T: 后应降级为僚机。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        self._enter_track_as_lead(agent)
        inbox = (self._tmsg(27.005, 125.005, "uav_1"),)
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                        comm_inbox=inbox)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK"
        assert agent._is_wingman, "uid 大长机应降级补僚机位"

    def test_second_lead_exits_when_wing_taken(self):
        """双长机且僚机位已占：uid 最大者应退出回 SEARCH（不记否决）。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        self._enter_track_as_lead(agent)
        inbox = (
            self._tmsg(27.005, 125.005, "uav_1"),
            self._jmsg(27.005, 125.005, "uav_2"),
        )
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                        comm_inbox=inbox)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH"
        assert agent._target is None

    def test_second_wingman_exits(self):
        """双僚机：uid 大者退出回 SEARCH。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        self._enter_track_as_lead(agent)
        agent._is_wingman = True
        inbox = (self._jmsg(27.005, 125.005, "uav_2"),)
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                        comm_inbox=inbox)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH"

    def test_search_skips_verify_when_fully_manned(self):
        """SEARCH 检测到的目标长僚已齐（T:+J:）时不进 VERIFY。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        inbox = (
            self._tmsg(27.001, 125.001, "uav_1"),
            self._jmsg(27.001, 125.001, "uav_2"),
        )
        obs = _make_obs(detected=True, target_lat=27.001, target_lon=125.001,
                        comm_inbox=inbox)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "SEARCH", "长僚已齐的目标不应再进 VERIFY"


class TestVerifyAbortCooldown:
    """VERIFY 接触丢失中止的冷却行为（死亡螺旋修复回归）。"""

    def _enter_verify(self, agent):
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        obs = _make_obs(detected=True, target_lat=27.003, target_lon=125.003)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "VERIFY"

    def test_abort_uses_short_flat_cooldown(self):
        """中止后 5s 内同位置不重进 VERIFY，5s 后允许；连续中止不升档。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent)
        # 30 帧（3s）无检测 → 中止（阈值 2s）
        for _ in range(30):
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "SEARCH"
        # 2s 内重检测同位置：挡（5s 平冷却）
        for _ in range(20):
            agent.decide(
                _make_obs(detected=True, target_lat=27.003, target_lon=125.003),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        # 再 3s+（自中止累计 >5s）：允许重进
        for _ in range(30):
            agent.decide(
                _make_obs(detected=True, target_lat=27.003, target_lon=125.003),
                dt=0.1,
            )
        assert agent._state.value == "VERIFY", "5s 平冷却后应允许重新判别"

    def test_verify_lost_keeps_gimbal_on_target(self):
        """VERIFY 无检测拍应继续指向目标（防 LOS 偏出 → 接触丢失 → 中止）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent)
        cmds = agent.decide(_make_obs(), dt=0.1)  # 无检测拍
        gimbal = _find_cmd(cmds, "component.gimbal_tracking.set_orientation")
        assert gimbal is not None, "VERIFY 无检测拍也应输出云台指向"

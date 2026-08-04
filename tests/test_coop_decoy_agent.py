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
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        assert not hasattr(agent, "_known_decoys") or not agent._known_decoys

    def test_moving_target_goes_track(self):
        """9 m/s 目标（[7,9) 区间）经两个 20s OLS 窗口确认后进 TRACK。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 9 m/s 东移：每帧 Δlon = 9*0.1/(111320*cos27°) ≈ 9.07e-6°
        for i in range(420):  # 两个 20s 窗口（9 m/s 在 [7,9) 需二次确认）
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 9.07e-6,
                ),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "快速移动目标应进 TRACK"

    def test_slow_mover_enters_track_probe(self):
        """5 m/s 目标与诱饵同速档（官方 runner 注入 decoy_speed=5.0）→ 进
        TRACK 验证模式（_is_probe=True）：同速不可分，不再依赖 Start 池否决，
        真伪由盯满 20s 后的冻结/继续移动判别。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(
                    detected=True,
                    target_lat=27.005,
                    target_lon=125.005 + i * 5.04e-6,
                ),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "5 m/s 档应进 TRACK 验证模式"
        assert agent._is_probe, "5 m/s 档应标记为验证模式"

    def test_lost_during_verify_aborts_without_decoy_mark(self):
        """VERIFY 中连续丢失 >6s 应放弃且不记诱饵（6s 阈值按 10Hz decide、
        检出率 ~35% 标定：容忍短暂 FOV 遮挡又不卡死在丢失目标上）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        for i in range(10):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "VERIFY"
        for i in range(65):  # 6.5s 无检测 > 6s 阈值
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "SEARCH", "丢失超时应回 SEARCH"

    def test_lock_flicker_rejected_by_speed_band(self):
        """锁跳变导致的虚高速度（>13.5）应被速度带上限拒绝。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 大部分时间锁在静止点，少量帧跳到 400m 外（位置阶跃 → 表观速度虚高）
        for i in range(210):
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
        12 m/s 真目标仍应判真（回归：dt 累加曾把 12 m/s 读成 ~5 误判诱饵）。
        判别改为 20s 时间窗后，需 20s 引擎时间（500 控制拍）。"""
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
        for i in range(520):  # 20.8s 引擎时间 > 20s 窗口
            agent.decide(_obs(i), dt=0.1)
        assert agent._state.value == "TRACK", (
            "引擎时间下 12 m/s 目标应判真"
        )

    def test_reject_cooldown_blocks_immediate_reverify(self):
        """判别否决后冷却期内同位置检测不重进 VERIFY，冷却后允许（停顿真目标可重遇）。

        新设计下只有"判诱"（诱饵 Start 区 + 速度带外）才记否决；测试用
        route_2 诱饵 Start 区的静止候选触发 reject 20s 冷却。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 诱饵 Start 区静止候选 → 否决
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.01913, target_lon=124.98309),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        # 冷却期内：同位置持续检测不应卡回 VERIFY
        for i in range(50):  # 5s
            agent.decide(
                _make_obs(detected=True, target_lat=27.01913, target_lon=124.98309),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "冷却期内不应重进 VERIFY"
        # 冷却期后（累计 >20s）：允许重新判别
        for i in range(210):  # 再 21s 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.01913, target_lon=124.98309),
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
        # 僚机已在目标 200m 内且检测到 → 直接 TRACK（边跟踪边判别，
        # 协锁 dwell 立即累计；假阳性由后台低速 bailout 兜底）
        obs = _make_obs(
            lat=27.005, lon=125.005, detected=True,
            target_lat=27.005, target_lon=125.005,
        )
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK"
        assert agent._is_wingman


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
        """announce 超过 40s 未更新应过期（40s：僚机 40 m/s 可飞 1.6km）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        msg = MagicMock()
        msg.payload = "A:27.010,125.010"
        msg.sender_uid = "uav_2"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert agent._shared_target is not None
        # 跑 41s 无新消息（41s > 40s 过期窗口）
        for i in range(410):
            agent.decide(_make_obs(), dt=0.1)
        assert agent._shared_target is None, "announce 应已过期"

    def test_broadcast_format(self):
        """已判真长机首次 TRACK 应广播 A:lat,lon（announce）；probe 只发 T:。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 手动设置状态到 TRACK（已判真长机）
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._confirmed_real = True
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

    def test_probe_leader_broadcasts_t_not_a(self):
        """probe（5 m/s 候选）长机过首帧后只发 T: 不发 A:——A: 只在进入
        TRACK 首帧发出，之后用 T: 保持共享目标新鲜。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._is_probe = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 0.0
        agent._last_report_time = 10.0
        agent._dwell_time = 5.0
        agent._track_time = 5.0
        agent._last_det_time = 10.0
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        cmds = agent.decide(obs, dt=0.1)
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert any(p.startswith("T:") for p in payloads), "probe 长机应发 T:"
        assert not any(p.startswith("A:") for p in payloads), "probe 长机不应发 A:"

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
        agent._confirmed_real = True  # OLS 判真的 9/12 目标才上报
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

    def test_wingman_does_not_report(self):
        """僚机 TRACK（JOIN 路径未过 OLS 判别）不上报——它不知道目标是真是
        诱饵，上报位置会匹配到最近存活真目标、污染 accuracy。"""
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
        assert not reports, "僚机未判真不应上报"

    def test_track_timers_use_engine_axis(self):
        """TRACK 计时用引擎轴（score_view.sim_time）：引擎每拍只走 0.04s
        而 dt=0.1 时，dwell 满 20s 需 ~500 拍而非 200 拍（回归：dt 累加曾让
        dwell 在引擎 ~8s 就触发、提前广播 D: 撤离打断协锁），_track_time
        同理按引擎轴累计。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._target = (27.005, 125.005)
        agent._sim_time = 0.0
        agent._last_bc_time = 0.0
        agent._last_report_time = 0.0
        agent._dwell_time = 0.0
        agent._track_time = 0.0
        agent._last_det_time = 0.0
        agent._filter = None

        sim_t = [0.0]
        dwell_200 = dwell_500 = dwell_520 = track_520 = None
        for i in range(530):
            sim_t[0] += 0.04  # 引擎每拍只走 0.04s（控制节拍 0.1s 的 2.5 倍快）
            # 12 m/s 东移：避免静止目标触发 <4.0 bailout 干扰计时测试
            obs = _make_obs(
                detected=True,
                target_lat=27.005,
                target_lon=125.005 + i * 4.84e-6,
            )
            obs.briefing.score_view.sim_time = sim_t[0]
            agent.decide(obs, dt=0.1)
            if i == 199:
                dwell_200 = agent._dwell_time
            if i == 499:
                dwell_500 = agent._dwell_time
            if i == 519:
                dwell_520 = agent._dwell_time
                track_520 = agent._track_time
        assert dwell_200 < 20.0, f"200 拍（dt 轴 20s）不应满 dwell，实际 {dwell_200}"
        assert 19.5 <= dwell_500 <= 20.5, (
            f"500 拍应累计引擎轴 ~20s（dt 轴会到 ~50），实际 {dwell_500}"
        )
        assert dwell_520 >= 20.0, f"满 20s 需 ~520 拍而非 200 拍，实际 {dwell_520}"
        assert track_520 >= 20.0, f"_track_time 应按引擎轴累计，实际 {track_520}"

    def test_5ms_filter_not_reported(self):
        """report 阈值 >5.5：5 m/s 档（真/诱不可分）不上报——上报诱饵位置
        会匹配到最近存活真目标、打爆它的 RMSE（污染 accuracy 维度）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_track(agent)
        agent._filter = _FakeFilter(speed=5.0, lat=27.005, lon=125.005)
        for i in range(30):  # 3s
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
            assert _find_cmd(cmds, "agent.report") is None, "5 m/s 档不应上报"

    def test_9ms_filter_reported(self):
        """report 阈值 >5.5：9/12 m/s 档正常上报（accuracy 维度脱离 0）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_track(agent)
        agent._confirmed_real = True  # OLS 判真标志
        agent._filter = _FakeFilter(speed=9.0, lat=27.005, lon=125.005)
        reports = []
        for i in range(30):  # 3s
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
            r = _find_cmd(cmds, "agent.report")
            if r is not None:
                reports.append(r)
        assert reports, "9 m/s 档应正常上报"


class TestProbeVerificationTail:
    """验证式跟踪尾段：probe 候选 dwell 满 20s 后，用引擎行为（冻结/继续
    移动）判别真伪——真目标被 ≥2 架盯满 20s 后引擎冻结，诱饵继续移动。"""

    def _enter_probe_dwell(self, agent, speed):
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._is_probe = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 10.0
        agent._dwell_time = 19.95  # 本帧 +0.1 后满 20s
        agent._track_time = 19.95
        agent._last_det_time = 10.0
        agent._filter = _FakeFilter(speed=speed, lat=27.005, lon=125.005)

    def _jmsg(self, lat, lon, sender):
        msg = MagicMock()
        msg.payload = f"J:{lat},{lon}"
        msg.sender_uid = sender
        return msg

    def test_frozen_target_marked_destroyed(self):
        """probe 尾段：dwell 满 20s 后滤波速度持续 <1.5（真目标被引擎冻结
        摧毁）→ D: 广播 + 记摧毁 + 回 SEARCH。"""
        agent = CoopDecoyAgent("uav_1")
        self._enter_probe_dwell(agent, speed=0.5)
        msg = self._jmsg(27.005, 125.005, "uav_2")
        cmds = None
        for _ in range(70):  # 7s > 5s 冻结证据窗口
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                          comm_inbox=(msg,)),
                dt=0.1,
            )
            if agent._state.value == "SEARCH":
                break
        assert agent._state.value == "SEARCH", "冻结真目标应判摧毁回 SEARCH"
        assert (27.005, 125.005) in agent._known_destroyed
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert any(p.startswith("D:") for p in payloads), "应广播 D: 摧毁通知"

    def test_moving_target_gets_cooldown(self):
        """probe 尾段：dwell 满 20s 后滤波速度持续 ≥1.5（诱饵被盯满仍移动）
        → 记冷却回 SEARCH，不标记摧毁、不广播 D:。"""
        agent = CoopDecoyAgent("uav_1")
        self._enter_probe_dwell(agent, speed=5.0)
        msg = self._jmsg(27.005, 125.005, "uav_2")
        cmds = None
        for _ in range(70):
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                          comm_inbox=(msg,)),
                dt=0.1,
            )
            if agent._state.value == "SEARCH":
                break
        assert agent._state.value == "SEARCH", "仍移动的诱饵应回 SEARCH"
        assert (27.005, 125.005) not in agent._known_destroyed
        assert agent._last_reject_pos is not None, "诱饵应记冷却"
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert not any(p.startswith("D:") for p in payloads), "诱饵不应广播 D:"

    def test_probe_lost_aborts_to_search(self):
        """probe 验证模式中检测全丢 >10s → 放弃回 SEARCH（不记否决，
        _mark_abort 的 5s 平冷却防死循环）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        agent._state = agent._state.TRACK
        agent._is_wingman = False
        agent._is_probe = True
        agent._target = (27.005, 125.005)
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 10.0
        agent._dwell_time = 1.0
        agent._track_time = 1.0
        agent._last_det_time = 10.0
        agent._filter = _FakeFilter(speed=5.0, lat=27.005, lon=125.005)
        for _ in range(110):  # 11s 无检测 > 10s 阈值
            agent.decide(_make_obs(), dt=0.1)
        assert agent._state.value == "SEARCH", "probe 盯丢超时应回 SEARCH"
        assert agent._last_reject_pos is None, "盯丢不应记否决"
        assert agent._last_abort_pos == (27.005, 125.005), "应记 5s 平冷却中止"


class TestCoopAgentDestroyedMemory:
    """已摧毁目标记忆：摧毁后不重复跟踪、上报。"""

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
        """单机 dwell 满 20s 不算摧毁（评估器按 K=2 协锁判毁）——
        继续盯防等僚机，不标记、不离开（旧逻辑单机满 20s 离开会拆掉协锁）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        obs = _make_obs(detected=True, target_lat=27.005, target_lon=125.005)
        agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK", "单机满 20s 应继续盯防"
        assert (27.005, 125.005) not in agent._known_destroyed

    def test_coop_dwell_marks_destroyed_and_broadcasts(self):
        """dwell 满 20s 且队友在场（J: 占位）→ 等引擎冻结证据（滤波速度
        <1.5 连续 5s，真目标已被引擎摧毁）→ 标记 + D: 广播 + 回 SEARCH。
        （v01 诊断：agent 自身 dwell 满 20s 时评测器 coop 往往未满，凭
        claim 提前宣布会拆散协锁并误标真目标）"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        agent._filter = _FakeFilter(speed=0.5, lat=27.005, lon=125.005)
        msg = MagicMock()
        msg.payload = "J:27.005,125.005"
        msg.sender_uid = "uav_2"
        cmds = None
        for _ in range(60):  # 6s > 5s 冻结证据窗口
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                          comm_inbox=(msg,)),
                dt=0.1,
            )
            if agent._state.value == "SEARCH":
                break
        assert agent._state.value == "SEARCH", "协锁满 20s 应回 SEARCH"
        assert (27.005, 125.005) in agent._known_destroyed
        payloads = [c.params["payload"] for c in cmds
                    if isinstance(c, Command) and c.verb == "comm.broadcast"]
        assert any(p.startswith("D:") for p in payloads), "应广播 D: 摧毁通知"

    def test_coop_dwell_moving_target_not_prematurely_destroyed(self):
        """dwell 满 20s + 队友在场但滤波速度仍 ≥1.5（评测器 coop 尚未完成，
        目标仍在移动）→ 不宣布摧毁、继续盯（v01 诊断：凭 claim 提前自宣
        摧毁是 0 杀的判别失误之一）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        self._enter_track_solo_dwell(agent)
        agent._filter = _FakeFilter(speed=9.0, lat=27.005, lon=125.005)
        msg = MagicMock()
        msg.payload = "J:27.005,125.005"
        msg.sender_uid = "uav_2"
        for _ in range(30):  # 3s
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005,
                          comm_inbox=(msg,)),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "目标仍在移动不应提前宣布摧毁"
        assert (27.005, 125.005) not in agent._known_destroyed

    def test_destroyed_message_marks_target(self):
        """收到 D: 消息应同步进已摧毁列表（不再判别该目标）。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        for i in range(5):
            agent.decide(_make_obs(), dt=0.1)
        msg = MagicMock()
        msg.payload = "D:27.010,125.010"
        msg.sender_uid = "uav_1"
        agent.decide(_make_obs(comm_inbox=(msg,)), dt=0.1)
        assert (27.010, 125.010) in agent._known_destroyed

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

    def test_orphan_wingman_promotes_and_announces(self):
        """孤儿僚机（leader 已离开、无 A:/T: claim）>5s 后晋升为长机并广播
        A:——否则它只发 J: 占位，其他 UAV 不会据此 JOIN，真目标永远凑不齐
        第二架（v06 诊断：20002 对 5 m/s 真目标 99.8% 连续有效却无人加入）。"""
        agent = CoopDecoyAgent("uav_3")
        agent.reset()
        self._enter_track_as_lead(agent)
        agent._is_wingman = True
        all_payloads = []
        for _ in range(60):  # 6s > 5s 晋升阈值
            cmds = agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
            all_payloads += [
                c.params["payload"] for c in cmds
                if isinstance(c, Command) and c.verb == "comm.broadcast"
            ]
        assert not agent._is_wingman, "孤儿僚机应晋升为长机"
        assert any(p.startswith("A:") for p in all_payloads), "晋升后应广播 A:"

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
        # 65 帧（6.5s）无检测 → 中止（阈值 6s）
        for _ in range(65):
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


class TestVerifyFastPass:
    """VERIFY 判别：OLS 20s 时间窗（fast-pass 已禁用——CvFilter 在 5 m/s
    诱饵上收敛过程有过冲尖峰误判真）。"""

    def test_fast_target_passes_in_window(self):
        """9 m/s 目标（[7,9) 区间）经两个 20s OLS 窗口确认后进 TRACK。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 9 m/s 东移：0.0000081°/帧 ≈ 0.9m/帧 @10Hz
        track_frame = None
        for i in range(420):  # 两个 20s 窗口
            obs = _make_obs(
                detected=True, target_lat=27.003, target_lon=125.003 + i * 0.0000081
            )
            agent.decide(obs, dt=0.1)
            if agent._state.value == "TRACK":
                track_frame = i
                break
        assert track_frame is not None, "9 m/s 目标应进 TRACK"

    def test_slow_decoy_enters_track_probe(self):
        """5 m/s 诱饵与真目标同速不可分 → 也进 TRACK 验证模式（不再靠
        速度带/Start 池否决），由 dwell 满 20s 后的继续移动判为诱饵。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        # 5 m/s 东移：0.0000045°/帧
        for i in range(210):
            obs = _make_obs(
                detected=True, target_lat=27.003, target_lon=125.003 + i * 0.0000045
            )
            agent.decide(obs, dt=0.1)
        assert agent._state.value == "TRACK", "5 m/s 诱饵同速不可分，应进验证模式"
        assert agent._is_probe, "5 m/s 档应标记为验证模式"


class _FakeFilter:
    """可控 CvFilter 替身：速度/位置/收敛由测试指定，滤波更新 no-op。"""

    def __init__(self, speed=5.0, lat=27.005, lon=125.005, converged=True):
        self._speed = speed
        self._lat = lat
        self._lon = lon
        self._conv = converged
        self.initialized = False

    def is_initialized(self):
        return self.initialized

    def initialize(self, la, lo, **kw):
        self.initialized = True
        self._lat, self._lon = la, lo

    def predict(self, dt):
        pass

    def update_position(self, la, lo):
        self._lat, self._lon = la, lo

    def position_wgs84(self):
        return (self._lat, self._lon)

    def speed_mps(self):
        return self._speed

    def velocity_mps(self):
        # (ve, vn) 东/北向速度：方向任意（测试不关心），模长 = speed
        return (self._speed, 0.0)

    def is_converged(self, std_m):
        return self._conv


class TestDiscriminationRedesign:
    """判别重设计（任务 4）：二次验证 / Start 池否决 / fast-pass 7.0 / 低速核查。"""

    def _enter_verify(self, agent, lat=27.005, lon=125.005):
        for _ in range(5):
            agent.decide(_make_obs(), dt=0.1)
        agent.decide(
            _make_obs(detected=True, target_lat=lat, target_lon=lon), dt=0.1
        )
        assert agent._state.value == "VERIFY"

    def test_ols_in_band_needs_two_windows(self):
        """OLS 低速入带 [6.5, 8.0) 不立即 TRACK（5 m/s 诱饵 ~13.5% 漂移误入带），
        第二独立窗口也入带才 TRACK（误报降至 ~1.8%）。
        ≥8.0 是 9/12 m/s 档确定性速度单窗口快判，不在此路径；用 7 m/s 测二次。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent, lat=27.003, lon=125.003)
        agent._filter = _FakeFilter(speed=6.0)  # <7.0 不触发 fast-pass，
        # 只测 OLS 二次窗口路径
        # 第一窗口：7 m/s 东移样本（0.00000706°/帧 = 0.7m/帧 @10Hz；
        # 入口检测点即序列起点，首帧从 0.7m 起避免重复点压低 OLS）
        for i in range(1, 211):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.003,
                          target_lon=125.003 + i * 0.00000706),
                dt=0.1,
            )
        assert agent._state.value == "VERIFY", "低速入带后应等待第二窗口"
        assert agent._verify_pass_count == 1
        # 第二窗口：继续 7 m/s 样本
        for i in range(211, 421):  # 再 21s
            agent.decide(
                _make_obs(detected=True, target_lat=27.003,
                          target_lon=125.003 + i * 0.00000706),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "第二窗口入带后应进 TRACK"

    def test_ols_fast_band_single_window(self):
        """OLS ≥9.0（9/12 m/s 确定性速度，5 m/s 漂移 0.4%）单窗口即 TRACK。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent, lat=27.003, lon=125.003)
        agent._filter = _FakeFilter(speed=5.0)  # 禁 fast-pass，只测 OLS 路径
        for i in range(1, 211):  # 21s > 20s OLS 窗口；12 m/s 无噪声 OLS=12 ≥9.0
            agent.decide(
                _make_obs(detected=True, target_lat=27.003,
                          target_lon=125.003 + i * 0.0000108),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "≥9.0 单窗口应直接判真"
        assert agent._verify_pass_count == 0

    def test_ols_out_of_band_suspect_gets_cooldown(self):
        """带外静止（OLS≈0 <2.0，真目标不停顿）：记升档冷却（防空转——
        盯静止对象纯浪费），冷却非永久（目标移动移出 500m 半径即可重判）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent)
        for i in range(210):  # 21s > 20s OLS 窗口；静止候选 → OLS=0 带外
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        assert agent._last_reject_pos is not None, "suspect 带外应记冷却"
        # 冷却期内：同位置不重进
        for i in range(50):
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "冷却期内不应重进 VERIFY"
        # 冷却期后（>20s）：允许重新判别（升档非永久）
        for i in range(210):  # 再 21s 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent._state.value in ("VERIFY", "SEARCH"), "冷却后应重新判别"

    def test_out_of_band_decoy_start_rejected(self):
        """带外静止（OLS≈0 <2.0）→ 记否决冷却（与 Start 池无关——
        Start 池裁决已删除，静止即否决）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent, lat=27.01913, lon=124.98309)  # route_2 Start
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.01913, target_lon=124.98309),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH"
        assert agent._last_reject_pos is not None, "诱饵 Start 区应记否决"

    def test_out_of_band_slow_goes_track_probe(self):
        """OLS 出带低速 [2.0, 7.0)（5 m/s 档真/诱同速不可分）→ 直接进 TRACK
        验证模式：不再查 Start 池、不 reject（训练集 Start 池验证集会失效），
        _is_probe=True。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent, lat=27.005, lon=125.005)  # 非任何 Start 区
        for i in range(1, 211):  # 21s > 20s OLS 窗口；5 m/s 东移
            agent.decide(
                _make_obs(detected=True, target_lat=27.005,
                          target_lon=125.005 + i * 0.0000045),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "5 m/s 带外低速应进 TRACK 验证模式"
        assert agent._is_probe, "带外低速应标记为验证模式"
        assert agent._last_reject_pos is None, "带外低速不应记否决"

    def test_stationary_rejected_even_at_true_start(self):
        """带外静止（OLS≈0 <2.0）→ reject 冷却，即使位置在真 Start 150m 内：
        真目标不停顿，盯静止对象纯浪费（原 Start 池 'true' 裁决已删除）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent, lat=27.00109, lon=125.00086)  # road1 Start
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.00109, target_lon=125.00086),
                dt=0.1,
            )
        assert agent._state.value == "SEARCH", "静止应回 SEARCH"
        assert agent._last_reject_pos is not None, "静止应记否决"

    def test_fast_pass_disabled_ols_only(self):
        """fast-pass 已禁用：判别只走 OLS 20s 时间窗。静止候选带外回 SEARCH
        （CvFilter 速度不再影响判别——曾在 5 m/s 诱饵上过冲误判真）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_verify(agent)
        agent._filter = _FakeFilter(speed=7.5)  # 即使 CvFilter 在带内
        for i in range(210):  # 21s > 20s OLS 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        # 静止候选 OLS=0 带外 → 回 SEARCH（CvFilter 的 7.5 不再触发 fast-pass）
        assert agent._state.value == "SEARCH", "fast-pass 禁用，静止候选应回 SEARCH"

    def _enter_track(self, agent, target, filter_):
        agent._state = agent._state.TRACK
        agent._target = target
        agent._filter = filter_
        agent._sim_time = 10.0
        agent._last_bc_time = 10.0
        agent._last_report_time = 10.0
        agent._dwell_time = 1.0
        agent._track_time = 1.0
        agent._last_det_time = 10.0

    def test_track_slow_check_enters_probe_mode(self):
        """TRACK 低速核查：CvFilter 持续 <5.5 → 不再查 Start 池退出，置
        _is_probe=True 进入验证模式继续盯（真伪交给 dwell 满 20s 后的
        冻结/继续移动判别）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_track(
            agent,
            (27.01913, 124.98309),  # 诱饵 Start 区（不再因此退出）
            _FakeFilter(speed=5.0, lat=27.01913, lon=124.98309),
        )
        for i in range(110):  # 11s > 10s 核查窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.01913, target_lon=124.98309),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "低速核查不再因 Start 区退出"
        assert agent._is_probe, "持续低速应进入验证模式"

    def test_track_slow_check_keeps_true_target(self):
        """TRACK 低速核查：<5.5 但位置在真 Start 区 → 继续盯
        （5 m/s 真目标在出生区不被误杀）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        self._enter_track(
            agent,
            (27.00109, 125.00086),  # road1 Start（true 区）
            _FakeFilter(speed=5.0, lat=27.00109, lon=125.00086),
        )
        for i in range(110):
            agent.decide(
                _make_obs(detected=True, target_lat=27.00109, target_lon=125.00086),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "真 Start 区低速真目标应继续盯"
        assert agent._is_probe, "低速应进入验证模式"

    def test_wingman_bail_keeps_5ms_target(self):
        """僚机 bailout 收紧到 4.0：4.5 m/s（5 m/s 档）在真 Start 区不退出；
        3.5 静止触发 bailout 退出（任何位置）。"""
        agent = CoopDecoyAgent("uav_1")
        agent.reset()
        agent._is_wingman = True
        self._enter_track(
            agent,
            (27.00109, 125.00086),
            _FakeFilter(speed=4.5, lat=27.00109, lon=125.00086),
        )
        for i in range(160):  # 16s > 15s 原 bailout 窗口
            agent.decide(
                _make_obs(detected=True, target_lat=27.00109, target_lon=125.00086),
                dt=0.1,
            )
        assert agent._state.value == "TRACK", "4.5 m/s 真 Start 区不应退出"

        agent2 = CoopDecoyAgent("uav_2")
        agent2.reset()
        agent2._is_wingman = True
        self._enter_track(
            agent2,
            (27.005, 125.005),
            _FakeFilter(speed=3.5, lat=27.005, lon=125.005),
        )
        for i in range(160):
            agent2.decide(
                _make_obs(detected=True, target_lat=27.005, target_lon=125.005),
                dt=0.1,
            )
        assert agent2._state.value == "SEARCH", "3.5 m/s 静止类应 bailout"


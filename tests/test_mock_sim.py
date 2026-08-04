"""MockSimClient 引擎测试：运动/检测/comm/注入链路的行为验证。

被测对象是"环境模拟层"（competition/sdk/core/mock_client.py），
评分/评测/隔离链路已在别处有测试覆盖。测行为不测实现。
"""
import math

import pytest

from competition.sdk.core.commands import (
    broadcast, fly_to, point_gimbal, set_gimbal_fov,
)
from competition.sdk.core.mock_client import MockSimClient

SCENARIO = "competition/scenarios/coop_decoy/scenario.json"
UAVS = ("20001", "20002", "20003")
TARGETS = ("10001", "10002", "10003")


@pytest.fixture()
def client():
    c = MockSimClient(scenario_path=SCENARIO, detect_loss=0.0)  # 无丢失：测几何
    c.connect()
    yield c
    c.close()


def _uav(client, uid=UAVS[0]):
    return client._entities[uid]


def test_world_init(client):
    """初始化：3 UAV + 3 真目标 + 15 诱饵，位置来自 scenario.json。"""
    kinds = {e["kind"] for e in client._entities.values()}
    assert kinds == {"uav", "ground_vehicle", "decoy_vehicle"}
    assert len(client._entities) == 21
    ws = client.wait_first_state()
    assert set(ws.uavs) == set(UAVS)
    assert set(ws.targets) == set(TARGETS)
    assert len(ws.decoys) == 15
    assert ws.sim_time == 0.0


def test_poll_advances_sim_time(client):
    """每拍 poll_latest 前进一个控制周期（0.1s），帧被正确解析。"""
    ws = client.poll_latest()
    assert abs(ws.sim_time - 0.1) < 1e-9
    ws2 = client.poll_latest()
    assert abs(ws2.sim_time - 0.2) < 1e-9
    # 帧含 gimbal_tracking 与 comm 结构（isolation 投影需要）
    raw = ws.uavs[UAVS[0]].raw
    assert "gimbal_tracking" in raw and "comm" in raw


def test_fly_to_moves_uav(client):
    """set_destination 后 UAV 朝目标移动，航向指向目标。"""
    uav = _uav(client)
    start = (uav["lat"], uav["lon"])
    # 向东 1km
    target = (start[0], start[1] + 0.01)
    client.publish(UAVS[0], fly_to(target[0], target[1], speed=40.0))
    client.poll_latest()
    uav = _uav(client)
    dist = math.hypot(uav["lat"] - start[0], uav["lon"] - start[1])
    assert dist > 0  # 已移动
    assert 0.0 < uav["heading"] < 180.0  # 大致向东


def test_loiter_keeps_uav_near_dest(client):
    """到点后绕圈（loiter），不远离目标点。"""
    uav = _uav(client)
    target = (uav["lat"], uav["lon"] + 0.001)  # ~110m 东
    client.publish(UAVS[0], fly_to(target[0], target[1], speed=40.0,
                                   loiter_radius=200.0))
    for _ in range(50):
        client.poll_latest()
    uav = _uav(client)
    dlat = abs(uav["lat"] - target[0]) * 111320.0
    dlon = abs(uav["lon"] - target[1]) * 111320.0 * math.cos(math.radians(target[0]))
    assert math.hypot(dlat, dlon) < 400.0  # 在 loiter 半径附近


def test_gimbal_approach_is_rate_limited(client):
    """云台按 pan_rate/tilt_rate 逼近指令角，非瞬移。"""
    uav = _uav(client)
    g = uav["gimbal"]
    assert g["pan_rate"] == 60.0 and g["tilt_rate"] == 45.0
    client.publish(UAVS[0], point_gimbal(120.0, -30.0))
    client.poll_latest()
    g = _uav(client)["gimbal"]
    assert abs(g["pan"] - 6.0) < 1e-6      # 60dps × 0.1s
    assert abs(g["tilt"] - (-4.5)) < 1e-6  # 45dps × 0.1s


def test_fov_applies_immediately(client):
    """set_fov 实时生效（探针证实的引擎行为），并钳制 [5,120]。"""
    client.publish(UAVS[0], set_gimbal_fov(30.0))
    client.publish(UAVS[0], set_gimbal_fov(200.0))
    assert _uav(client)["gimbal"]["fov"] == 120.0
    client.publish(UAVS[0], set_gimbal_fov(1.0))
    assert _uav(client)["gimbal"]["fov"] == 5.0


def test_detection_locks_nearest_in_cone(client):
    """云台指向某目标时检测真值锁"离光轴最近"，含角偏差与置信度。"""
    uav = _uav(client)
    # 把 UAV 移到目标 10001 正上方附近，云台向下
    uav["lat"], uav["lon"] = 27.005, 124.998
    uav["heading"] = 0.0
    client.publish(UAVS[0], point_gimbal(0.0, -90.0))
    ws = client.poll_latest()
    # 云台速率 45dps：从 0° 转到 -90° 需 2s，先转到位再检测
    for _ in range(25):
        ws = client.poll_latest()
    det = ws.uavs[UAVS[0]].raw["gimbal_tracking"]["detection"]
    assert det["detected"] is True
    assert det["target_type"] == "ground_vehicle"
    assert abs(det["target_position"]["latitude"] - 27.005) < 1e-6
    assert det["confidence"] > 0.9


def test_detection_false_when_pointing_away(client):
    """云台指向天空时无检测。"""
    client.publish(UAVS[0], point_gimbal(0.0, 0.0))
    ws = client.poll_latest()
    det = ws.uavs[UAVS[0]].raw["gimbal_tracking"]["detection"]
    assert det["detected"] is False


def test_detection_marks_decoy_as_misid(client):
    """锁到诱饵时 misid_flag=True 且 target_type=decoy_vehicle。"""
    client.publish(UAVS[0], set_gimbal_fov(120.0))  # 宽锥更容易吞诱饵
    client.publish(UAVS[0], point_gimbal(0.0, -90.0))  # 指向正下方，排除远处真目标
    # 找最近的诱饵：把所有真目标挪远，只留诱饵在锥内
    for t in TARGETS:
        e = client._entities[t]
        e["lat"], e["lon"] = e["lat"] + 0.5, e["lon"] + 0.5
    for _ in range(25):
        ws = client.poll_latest()  # 等云台转到位
    det = ws.uavs[UAVS[0]].raw["gimbal_tracking"]["detection"]
    assert det["detected"] is True
    assert det["misid_flag"] is True
    assert det["target_type"] == "decoy_vehicle"


def test_comm_delivery_within_range(client):
    """broadcast 在 1000m 内送达 inbox，发送方 sent/delivered 计数。"""
    client.publish(UAVS[0], broadcast("P:27.0,125.0"))
    client.poll_latest()
    # 三机初始相距 <2km（20001 与 20002 约 1.1km、与 20003 约 0.9km）
    for tuid in UAVS[1:]:
        inbox = client._entities[tuid]["comm"]["inbox"]
        assert any(m["sender"] == UAVS[0] and m["payload"].startswith("P:")
                   for m in inbox)
    s = client._entities[UAVS[0]]["comm"]
    assert s["sent"] == 1 and s["delivered"] == len(UAVS) - 1


def test_comm_rejected_out_of_range(client):
    """超过 1000m 的接收方被拒绝（rejected_range 计数）。"""
    client._entities[UAVS[0]]["lat"] += 0.05  # ~5.5km 远
    client.publish(UAVS[0], broadcast("A:hello"))
    client.poll_latest()
    s = client._entities[UAVS[0]]["comm"]
    assert s["sent"] == 1 and s["delivered"] == 0
    assert s["rejected_range"] == len(UAVS) - 1
    for tuid in UAVS[1:]:
        assert client._entities[tuid]["comm"]["inbox"] == []


def test_astar_plan_returns_road_polyline(client):
    """astar_plan 返回路网折线近似（真实沿路网规划，mock 校准项）。"""
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "astar_plan",
        "params": {"start_lat": 27.00109, "start_lon": 125.00086,
                   "end_lat": 27.01, "end_lon": 125.01},
    })
    frame = client._frames[-1]
    res = frame[TARGETS[0]]["astar_plan_result"]
    assert res["success"] is True and res["count"] >= 2
    # 首点是请求起点
    assert abs(res["waypoints"][0]["lat"] - 27.00109) < 1e-6
    # 路网折线：中间点沿最近路网（非直线）
    from competition.sdk.core.mock_client import _road_path_between
    wps = _road_path_between(27.00109, 125.00086, 27.01, 125.01)
    assert len(wps) >= 2
    assert abs(wps[0]["lat"] - 27.00109) < 1e-6
    # 路网折线应含路网节点（road1 Start 附近 → 沿途节点）
    assert any(27.007 <= w["lat"] <= 27.013 for w in wps)


def test_set_trajectory_moves_target(client):
    """set_trajectory 后目标沿航点匀速移动，速度来自 set_speed。"""
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "set_speed", "params": {"speed": 9.0}})
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "set_trajectory",
        "params": {"waypoints": [
            {"lat": 27.005, "lon": 124.998},
            {"lat": 27.006, "lon": 124.999},
        ]}})
    client.poll_latest()
    e = client._entities[TARGETS[0]]
    moved = math.hypot(e["lat"] - 27.005, e["lon"] - 124.998)
    assert moved > 0
    # 9 m/s × 0.1s ≈ 0.9m（大圆距离）
    from competition.sdk._vendored.geometry import haversine_m
    dist = haversine_m(27.005, 124.998, e["lat"], e["lon"])
    assert abs(dist - 0.9) < 1e-6


def test_set_position_halts_target(client):
    """set_position 冻结目标（摧毁语义：位置不再变化）。"""
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "set_speed", "params": {"speed": 9.0}})
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "set_trajectory",
        "params": {"waypoints": [
            {"lat": 27.005, "lon": 124.998},
            {"lat": 27.006, "lon": 124.999},
        ]}})
    client.poll_latest()
    client.publish_raw({
        "unique_id": TARGETS[0], "cmd": "set_position",
        "params": {"latitude": 27.0055, "longitude": 124.9985}})
    client.poll_latest()
    e = client._entities[TARGETS[0]]
    assert e["halted"] is True
    assert e["lat"] == 27.0055 and e["lon"] == 124.9985


def test_end_engine_sets_status(client):
    """publish_engine('end') → 帧 status=ended。"""
    client.publish_engine("end")
    ws = client.poll_latest()
    assert ws.status == "ended"


def test_decoy_lock_steal_probability():
    """FOV 内同时有真目标和诱饵时，诱饵抢锁概率模型生效。"""
    from competition.sdk.core.mock_client import _hash01

    # 创建精简 scenario：1 UAV + 1 真目标 + 1 诱饵，都在 FOV 内
    import json, tempfile, os
    scenario = {
        "entities": [
            {
                "id": "U1", "name": "U1", "type": "fixed_wing_uav",
                "params": {
                    "initial_latitude": 27.0, "initial_longitude": 125.0,
                    "initial_altitude": 500.0, "initial_heading": 0.0,
                },
                "components": {
                    "kinematics": {"params": {"min_speed": 15, "max_speed": 40}},
                    "gimbal_tracking": {"params": {"fov": 60.0}},
                },
            },
            {
                "id": "T1", "name": "T1", "type": "ground_vehicle",
                "params": {
                    "initial_latitude": 27.001, "initial_longitude": 125.0,
                    "initial_altitude": 0.0,
                },
            },
            {
                "id": "D1", "name": "D1", "type": "decoy_vehicle",
                "params": {
                    "initial_latitude": 27.002, "initial_longitude": 125.0,
                    "initial_altitude": 0.0,
                },
            },
        ]
    }
    fd, path = tempfile.mkstemp(suffix=".json")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(scenario, f)

        # ---- decoy_lock_rate=1.0：必锁诱饵 ----
        c = MockSimClient(scenario_path=path, detect_loss=0.0,
                          decoy_lock_rate=1.0)
        c.connect()
        c.publish("U1", set_gimbal_fov(60.0))
        c.publish("U1", point_gimbal(0.0, -90.0))  # 指向正下方
        for _ in range(25):
            ws = c.poll_latest()  # 等云台转到位
        det = ws.uavs["U1"].raw["gimbal_tracking"]["detection"]
        assert det["detected"] is True, "应检测到目标"
        assert det["misid_flag"] is True, "rate=1.0 时必锁诱饵"
        assert det["target_type"] == "decoy_vehicle"
        c.close()

        # ---- decoy_lock_rate=0.0：必锁真目标 ----
        c = MockSimClient(scenario_path=path, detect_loss=0.0,
                          decoy_lock_rate=0.0)
        c.connect()
        c.publish("U1", set_gimbal_fov(60.0))
        c.publish("U1", point_gimbal(0.0, -90.0))
        for _ in range(25):
            ws = c.poll_latest()  # 等云台转到位
        det = ws.uavs["U1"].raw["gimbal_tracking"]["detection"]
        assert det["detected"] is True, "应检测到目标"
        assert det["misid_flag"] is False, "rate=0.0 时必锁真目标"
        assert det["target_type"] == "ground_vehicle"
        c.close()
    finally:
        os.unlink(path)

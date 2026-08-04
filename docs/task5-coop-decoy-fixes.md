# 赛题二修复任务书：时间轴 / report / 验证式跟踪 / 区域覆盖

> 本文档供 codex 独立执行。所有行号基于 2026-07-31 master 分支，实现时以实际代码为准。
> 改动只涉及两个文件：`competition/user_algorithms/coop_decoy/agent.py`、`algorithms/search/start_pools.py`，外加测试 `tests/test_coop_decoy_agent.py`。

## 0. 背景与目标

v35 局（600s，coop_decoy）首次达成协锁（`coop_ticks=256`），但：`dwell_accumulated` 从未达到 20s（`resets=1`），`n_reports=0`，`n_destroyed=0`。诊断结论：

1. **TRACK/JOIN 计时用错时间轴**（agent 轴 `dt` 累加，比引擎时间快 ~2.5 倍）→ dwell 20s 判定在引擎 8s 就触发，UAV 提前广播 D: 撤离，评测器侧 coop 累计到 ~8s 被自己的撤离打断清零。这是协锁不连续的头号嫌疑。
2. **report 阈值 `speed_mps() > 3.0` 依赖滤波速度收敛**，收敛慢 + TRACK 存活短 → 整局零上报，accuracy 维度 = 0。
3. **5 m/s 档判别依赖训练集 Start 池**；验证集"目标小车路线会采取新路线"，Start 池大概率失效 → 5 m/s 真目标丢失 → 无法全歼 → base 必然 <70 不达标（见 §1 达标数学）。
4. **SEARCH 路线先验**（训练集航点）同样依赖训练集，需退化方案。

修复目标：时间轴对齐、report 恢复、5 m/s 档改"验证式跟踪"、SEARCH 改区域覆盖。

## 1. 关键事实（执行前必读）

### 1.1 评测器有两条独立评分管道

```
管道 A（kill/协锁）：引擎检测(相机几何+accuracy骰子) → 最近邻匹配真目标(≤120m)
                    → 非 misid → ≥K=2 架同时 → 累计 20s（中断 ≤2s 回补，>2s 清零）→ 摧毁
管道 B（accuracy）：agent 的 report_target 发射 → 最近邻存活真目标 → RMSE（120m 容差）
```

- 管道 A **不吃 report**，只吃"相机有没有对准"（runner.py `_observe_scoring` 用引擎 `gimbal_tracking.detection` 喂评测器）。agent 与 kill 的唯一接口是 gimbal 指向。
- 管道 B 只吃 report。`n_reports=0` 时 accuracy 维度 = 0。report 只要发出且最近存活目标不是"尸体"就被计入，无距离门槛，每目标限 1/s（`coop_eval.py::record_report`）。
- 摧毁后引擎把真目标 `set_position` **冻结**（不再移动）；诱饵被盯满 20s 只是 identified，**继续移动**——这是"验证式跟踪"的依据。

### 1.2 达标数学（为什么 5 m/s 档必须杀）

达标线 = `kill_rate ≥ 2/3` **且** `base_score ≥ 70`。base = 0.5×kill + 0.3×accuracy + 0.2×mission_time。

不杀满 3 个时 kill ≤ 33.3 分；未全歼时 mission_time = 0；accuracy 上限 30 分 → base 上限 63.3 < 70。**不达标**。所以必须全歼 3 个真目标（含 5 m/s 那台，每局 3 真目标速度档为 5/9/12 m/s 各一），且全歼要快（≤240s 满分，240–420s 线性归零），且至少部分目标有上报。

### 1.3 时间轴语义

`_sync_time`（agent.py L267-291）从 `obs.briefing.score_view.sim_time` 读引擎时间，并维护 `self._sim_dt`（本拍引擎时间增量）。实测控制节拍比引擎时间快 ~2.5 倍：用 `dt` 累加会让所有时间基准失真 2.5 倍。VERIFY 内计时（`_verify_lost_s`、`_wing_bail_s`、`_slow_check_s`）已用 `_sim_dt`；**TRACK/JOIN 的 `_track_time`/`_dwell_time`/`_join_time` 仍是 `dt` 累加——本次修复对象**。

## 2. 改动一：时间轴统一（agent.py）

| 位置 | 现状 | 改为 |
|---|---|---|
| L759 `_do_track` | `self._track_time += dt` | `self._track_time += self._sim_dt` |
| L869 `_do_track` | `self._dwell_time += dt` | `self._dwell_time += self._sim_dt` |
| L867 `_do_track`（同块 grace 检查） | `gap <= _TRACK_GRACE_S + dt` | `gap <= _TRACK_GRACE_S + self._sim_dt`（gap 是引擎轴 `_sim_time` 差值，右端必须同轴） |
| L961 `_do_join` | `self._join_time += dt` | `self._join_time += self._sim_dt` |

不要动：L550 `_gimbal_phase += dt * 0.5`（扇扫相位，与评分无关）；低速核查 L804/L818 已是 `_sim_dt`。

效果：dwell 20s 判定与评测器对齐（引擎 20s 才触发 D: 撤离）；`_TRACK_TIMEOUT_S=90` 恢复 90 引擎秒（此前 ~36s 就超时放弃）。

## 3. 改动二：report 发射阈值（agent.py L943-952）

现状：

```python
if (
    self._sim_time - self._last_report_time >= _REPORT_INTERVAL
    and self._filter
    and self._filter.is_initialized()
    and self._filter.speed_mps() > 3.0
):
```

目标：5 m/s 候选（真/诱不可分）**不报**（报诱饵位置会匹配到最近存活真目标、把它的 RMSE 打爆，污染 accuracy 维度）；9/12 m/s 档正常报。

改法：`speed_mps() > 3.0` → `speed_mps() > _SLOW_CHECK_SPEED`（常量已存在 = 5.5，与改动三的验证阈值一致）。滤波初始化条件保留；不做事先"检测点兜底上报"（RMSE 是全程累计，几秒缺口影响 <1%，不值当的复杂度）。

## 4. 改动三：5 m/s 档验证式跟踪（agent.py）

### 4.1 原理

5 m/s 真目标与 15 台 5 m/s 诱饵运动学完全不可分（同速同路网同伪装），唯一判别通道是引擎行为差异：真目标被 ≥2 架盯满 20s 后**冻结**（不再移动），诱饵被盯满 20s 后**继续移动**。所以"2 架盯住 5 m/s 候选 20s 不撒手"本身就是判别器：盯满后目标速度骤降 → 真目标已摧毁；仍在 5 m/s 移动 → 诱饵，放弃（误跟踪不扣分，只耗时间）。

### 4.2 VERIFY OLS 出带分支（agent.py L710-733）

现状：速度出带（含 5 m/s）→ `match_start_pool` 裁决，非 'true' → reject 冷却。

改为（`_do_verify` 内出带分支）：

- OLS 速度在 `[2.0, 7.0)`（新增常量 `_PROBE_SPEED_MIN = 2.0`）→ **直接进 TRACK（验证模式）**，不再查 Start 池、不再 reject。进 TRACK 时重置 `_dwell_time/_track_time/_last_det_time/_slow_check_s`，并置 `self._is_probe = True`。
- OLS 速度 <2.0（静止）→ 维持 reject（真目标不停顿，盯静止对象纯浪费）。
- OLS 速度 ≥14.5 → 维持 reject + 冷却（锁跳变虚高）。
- Start 池 `match_start_pool` 的 'true' 裁决（L720-728）**删除**——训练集烘焙的 Start 池在验证集可能失效，5 m/s 真目标不再依赖它；5 m/s 目标无论在哪出生都走验证模式。
- jump 否决（L658-668）保留，先于速度带拦截。

### 4.3 TRACK 低速核查语义（agent.py L789-832）

现状：`v<4.0` 持续 15s → bailout 退出；`v<5.5` 持续 10s → 查 Start 池，非 'true' 退出。

改为：

- `v<4.0` 持续 15s → bailout 退出（保留，5 m/s 档不触发）。
- `v<5.5` 持续 10s → **不再查 Start 池退出**，置 `self._is_probe = True`（进入验证模式，继续盯）；`v≥5.5` 时置 `self._is_probe = False`。
- 新增字段 `self._is_probe: bool = False`（`reset()` 中初始化）。

注意：JOIN→TRACK 直入路径（L1001-1010）不改，僚机 `_is_probe` 从 False 起步，由滤波速度自动收敛到正确语义（9/12 档 → False；5 档 → True）。

### 4.4 dwell 满 20s 分叉（agent.py L879-891）

现状：`_dwell_time >= 20 and 队友在场 → 广播 D:、记 _known_destroyed、回 SEARCH`。

改为（保持"队友在场"条件，K=2 才能摧毁）：

- 非 probe：原逻辑不变（D: 广播 + 记摧毁 + 回 SEARCH）。
- probe：进入"验证尾段"（新增 `_probe_check_s` 计时，引擎轴）：
  - 滤波速度连续 5s（`_sim_dt` 累计）< 1.5 → 真目标已被引擎冻结摧毁 → D: 广播 + 记 `_known_destroyed` + 回 SEARCH；
  - 滤波速度连续 5s ≥ 1.5 → 诱饵（identified）→ 记冷却（复用 `_mark_reject` 的冷却机制，位置为 `_target`）回 SEARCH；
  - 期间检测持续丢失（`_sim_time - _last_det_time > 10s`）→ 直接放弃回 SEARCH（不记否决）。
- 已知局限（接受）：验证尾段内队友恰好离开（K<2）导致目标未冻结、被误判诱饵——概率低，20s 都守住了 5s 尾段走的概率小；误判只丢一个 5 档候选，回收 UAV 后可重新发现。

### 4.5 probe 盯丢退出（agent.py `_do_track` 内）

验证模式（`_is_probe=True`）中，若 `_sim_time - _last_det_time > 10s`（检测全丢、dwell 无法累计）→ 放弃回 SEARCH（不记否决，5s 平冷却防死循环）。非 probe 不加（保持最小改动，靠 90s 超时兜底）。

## 5. 改动四：SEARCH 区域覆盖

### 5.1 新增 `algorithms/search/start_pools.py` 函数

```python
def coverage_waypoints_for_uid(uid: str, n_shares: int = 3) -> list[tuple[float, float]]:
    """区域覆盖航点：TARGET_STARTS(26) + DECOY_STARTS(18) 共 44 点混合，
    按 uid 数字后缀取模 n_shares 分片（步长 n_shares 取点，首点按 uid 序号偏移
    错开），三架 UAV 覆盖不相交、合取为全集。验证集换路线后新 Start 仍在
    A* 路网节点附近（环境要素不变=路网不变），44 点覆盖路网关键交汇处。"""
```

- `TARGET_STARTS`（26 点，L11-40）、`DECOY_STARTS`（18 点，L41-63）常量已存在，直接混合。
- uid 后缀数字取法：`int(uid.rsplit("_", 1)[-1]) % n_shares` 之类，与 `route_waypoints_for_uid` 现有分片风格保持一致（先读 `algorithms/search/target_roads.py:319` 参考其写法，风格对齐）。

### 5.2 agent.py L460-465

`self._search_waypoints = route_waypoints_for_uid(self.my_uid, n_shares=3)` → `coverage_waypoints_for_uid(self.my_uid, n_shares=3)`；import 改为 `from algorithms.search.start_pools import match_start_pool, coverage_waypoints_for_uid`（若 `route_waypoints_for_uid` 全文件仅此一处使用，同步删除其 import；`target_roads.py` 文件本身不动）。

其余 SEARCH 逻辑（到达 200m 内切下一点、proximity 避让、扇扫）不动。

权衡（接受）：覆盖点 26→44，遍历一轮变长（~600s 量级），训练集表现可能略降；换来验证集不失效。

## 6. 测试计划（tests/test_coop_decoy_agent.py）

先读测试文件现有的 `_make_obs` / `_enter_track` / `_FakeFilter` 基建，沿用其风格。以下用例按需新增/更新：

1. **时间轴**：`_make_obs` 若支持 `briefing.score_view.sim_time` 则新增用例——模拟引擎时间每拍只走 0.04 而 `dt=0.1`，断言 `_dwell_time` 按引擎轴累计（满 20s 需要 ~500 拍而非 200 拍）、`_track_time` 同理。若 `_make_obs` 不支持 sim_time，先给它加参数（注意 `_sync_time` 读不到时 fallback `dt`，此时两轴相等，现有测试行为不变）。
2. **report 阈值**：`_FakeFilter(speed=5.0)` → 不发 report；`_FakeFilter(speed=9.0)` → 发 report。
3. **VERIFY 出带低速放行**：OLS 判出 ~5 m/s（带外低速）→ 进 TRACK 且 `_is_probe=True`，不再 reject。替换/更新现有 `test_out_of_band_suspect_gets_cooldown`（语义变了：低速带外不再冷却，静止/高速带外才冷却）。
4. **低速核查**：probe 目标（v=5.0）持续 10s+ → 不退出、`_is_probe=True`。更新 `test_track_slow_check_exits_on_decoy_start`（原"非 true 区退出"语义删除）。
5. **dwell 20s 分叉**：probe + 满 20s 后滤波速度 0.5 → D: 广播 + 回 SEARCH；probe + 满 20s 后速度 5.0 → 冷却回 SEARCH。
6. **probe 盯丢退出**：probe 中检测中断 >10s → 回 SEARCH。
7. **SEARCH 分片**：`coverage_waypoints_for_uid` 三 uid 分片不相交、并集 = 44 点全集、首点互不相同。
8. 全量跑 `pytest tests/test_coop_decoy_agent.py tests/test_start_pools.py`，涉及 5 档 reject/Start 池裁决语义的既有用例逐个核对更新。

## 7. 验证与验收

1. 单测全绿。
2. 连跑 2 局 600s（`run/run_fast.ps1 -Scenario coop_decoy -Agent competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent -Duration 600 -Output output/coop_600_vNN`，每局墙钟 ~14 分钟，用后台运行等待）。验收：
   - `n_destroyed ≥ 2`（9/12 档必须稳定杀，向全歼 3 个靠拢）；
   - `n_reports > 0`（accuracy 维度脱离 0）；
   - `dwell_accumulated_s` 达到 20s 量级、`resets` 明显下降（协锁连续）；
   - 对照 v35（coop_ticks=256 / n_reports=0 / dwell 未满 20s）。
3. 若首局不达标，按"先看 dwell 断点（哪一秒中断、中断时 UAV 在干什么）→ 再看判别失误"顺序诊断，用输出目录的 evaluation.json + agent debug 日志（`_DEBUG` 打印）定位。

## 8. 范围外（不要做）

- 不改检测几何（FOV、飞行速度、loiter 半径、gimbal 策略）——下一轮；
- 不加地形边界软约束——下一轮；
- 不改通信报文格式（P:/A:/T:/J:/D:）；
- 不改 JOIN 逻辑、VERIFY 的 OLS 窗口/快判逻辑；
- 不 commit（等用户许可）。

## 9. 涉及文件

| 文件 | 动作 |
|---|---|
| `competition/user_algorithms/coop_decoy/agent.py` | 改动一~四（§2-5） |
| `algorithms/search/start_pools.py` | 新增 `coverage_waypoints_for_uid`（§5.1） |
| `tests/test_coop_decoy_agent.py` | 新增/更新用例（§6） |
| `tests/test_start_pools.py` | 可选：coverage 分片测试放这里 |

参考阅读（动笔前）：`agent.py` 全文（重点 L247-291 `_sync_time`、L456-555 `_do_search`、L658-733 VERIFY 判别、L756-954 `_do_track`、L958-1038 `_do_join`）；`algorithms/search/start_pools.py`；`competition/sdk/_vendored/coop_eval.py`（L399-508 状态机、L577-625 record_report）；`tests/test_coop_decoy_agent.py` 测试基建。

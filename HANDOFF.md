# HANDOFF — 红枫2026 仿真加速结论与快速迭代环境（最终状态交接）

## 仓库

- 路径：`D:/codes/hf2026-sim-windows/`
- 当前分支：`master`
- 最新三批提交（按顺序）：
  - `b3e947d`  fix(coop_decoy): 环境根因修复与搜索判别重构（仿真诊断驱动）
  - `649170f`  perf(coop_decoy): 高飞扫描+路线先验+心跳避让+平滑云台
  - `9116948`  fix(search_track): ACQUIRE+搜索中心改用自身位置（避开随机化坐标陷阱）
- 测试：`pytest tests/ -q` → **170 全过**

---

## 1. 环境修复（一次性）

| 问题 | 现象 | 修复 |
|---|---|---|
| `config/HeightSample.csv`（786MB）本地缺失 | 引擎 `csv_unavailable` 拒掉全部地面车辆轨迹命令，**所有目标/诱饵冻结在原地**（此前所有"判别异常"都由此衍生） | `git show 46250a8:config/HeightSample.csv > config/HeightSample.csv`（从 git 历史对象恢复），加入 `/config/HeightSample.csv` 到 `.gitignore`（786MB 不入库） |
| `run/run_fast.ps1` 缺失 | 无官方 headless 快速迭代入口 | 已提交（headless CLI、自动启 Redis 6382、`-ReuseEngine` 复用引擎） |
| scenario.json 端口被改（6381→6382） | 跑/停不能即用 | `run_fast.ps1` 自动在 `output/` 下生成 `scenario.local.json`（redis_port=6382），官方 scenario.json 保持原样 |

复现 786MB 文件：

```bash
git show 46250a8:config/HeightSample.csv > config/HeightSample.csv
# 该 commit 也是 pdb 等 UE 调试符号的清理提交（305a063）
```

---

## 2. 五个根因 + 修复（每行有仿真证据）

| 根因 | 现象 | 修复 | 提交 |
|---|---|---|---|
| **地形缺失** | 上文 1 | 上文 1 | (环境) |
| **agent 时间基准失真** | runner 控制节拍比引擎快 ~2.5×；`self._sim_time += dt` 让 OLS 速度低估 2.5×（12 m/s 真目标读成 ~5）、1Hz 上报节拍失真 | 改用 `obs.briefing.score_view.sim_time`（引擎真时间），mock obs 回退 dt 累加保证测试 | `b3e947d` |
| **判别前提失效** | 官方 runner `inject_astar_decoy decoy_speed=5.0` 给 15 个诱饵也注入 5 m/s 路线，"静止即诱饵"失效；>12 m/s 虚高来自锁跳变/UAV 过顶 | OLS 速度带 [6.5, 13.5]，放弃 T1（5 m/s 类速度不可分）| `b3e947d` |
| **永久诱饵标记陷阱** | 移动诱饵逃出 150m 标记；真目标 WaitTime ≤30s 停顿被判"静止诱饵"永久误标 | 改为 20s 否定冷却 + 同位置连续否决指数升档（20→40→80→160s）| `b3e947d` |
| **搜索效率/覆盖** | 螺旋半径 700m 覆盖图幅 ~15%，9/12 m/s 快目标从未进入搜索环；诱饵反复触发 12s 鉴别空转 | 高飞（40 m/s + 500m alt，相机脚印 ~570m，扫描率 ×4）+ 沿 `points.json` 26 条路网路线先验扫描（路线数据离线烘到 `algorithms/search/target_roads.py`，decide 禁读文件）| `649170f` |

附带修复（每条都有独立诊断 + TDD 测试）：

| 子问题 | 修复 |
|---|---|
| TRACK 状态从不更新 IMM → 上报位置冻结、accuracy 必 0 | TRACK 中持续 predict/update_bearing/update_range |
| SEARCH 无云台扫描（云台停在上一方向） | `_do_search` 加 pan ±90° 扇扫 |
| 僚机 JOIN 后直接进 TRACK 不判别 → 诱饵被白飞 60s | JOIN→VERIFY 统一所有 TRACK 入口过 OLS |
| 提前 announce（VERIFY 入口）→ 全队围着同一诱饵空转 | 移到 TRACK 入口 announce |
| K=2 协同：长机 35s 超时放弃 → 僚机来不及（跨区 50s+VERIFY 12s）| 长机 90s 超时 + 僚机 350m 入 VERIFY |
| <200m proximity 扣分 8×2=15 把 accuracy 清零 | 位置心跳 P:lat,lon（1Hz）+ 队友<300m 时沿远离方向退 300m |
| K=2 dwell 中断 >2s 清零 | TRACK 云台瞄准改用 IMM 滤波位置（不是逐帧检测，平滑减少锁中断）|
| 赛题一 `briefing.target_initial_pos` 是随机化前坐标 → agent 飞向错位点 | ACQUIRE 直接进 SEARCH，螺旋中心改 `obs.self.lat/lon` |

---

## 3. 当前得分（600s 单局，seed=-1 每次随机）

| 赛题 | 基线 | 当前（多次单局观察值） | 备注 |
|---|---|---|---|
| 赛题一 | 1.24 / 100 | **66.0 / 62.7（600s 局，连续 passed）** | 机体系 pan + CvFilter + 每秒必报：resets=0、dwell 全程连续、RMSE 13~14m |
| 赛题二 | 0 / 100 | 0~17.9，零稳定击杀；v25 accuracy 26.9 | 已修：递归崩溃、ImmFilter 发散、残留引擎污染、VERIFY 死亡螺旋、三机扎堆仲裁、单机误判摧毁离场、VERIFY 吞吐（fast-pass+僚机直入）。当前瓶颈：双机 20s 连续协锁脆弱——检测中断 >2s 即清零重来（v25：201 coop_ticks + 2 resets） |
| 赛题三 | 未跑 | 未跑 | 骨架代码完成（`competition/user_algorithms/adversarial_swarm/agent.py`），注意 **K=3**（runner DEFAULT_K=3），骨架按 K=1 设计需改 |

赛题一演进（600s 单局）：v3 0.05（链路通但只报 10 次）→ v4 1.1（每秒必报 506 次但 ImmFilter 发散 RMSE 1039m）→ v5 66.0 / v6 62.7（CvFilter + 机体系 pan，两条不同随机路线，连续 passed，均零丢锁、dwell 全程连续、RMSE 13~14m）。

机制验证证据（v8 局）：accuracy 27.9，n_reports=170；K=2 首次协锁（10001 coop_ticks=5, resets=1）；proximity 8→1（v9 修复后）；n_reports=10 RMSE=102m（赛题一修复后）。

---

## 4. 关键文件位置

```
competition/user_algorithms/
  search_track/my_agent.py          # 赛题一：ACQUIRE/SEARCH/ENGAGE/ATTACK/LOST 五态（路线先验截获+每秒必报）
  coop_decoy/agent.py              # 赛题二：SEARCH/VERIFY/TRACK/JOIN 四态
  adversarial_swarm/agent.py        # 赛题三：5×2 扇区 + OLS + SAM 水平绕行 + 干扰广播

algorithms/
  estimation/cv_kalman.py           # CvFilter：双轴匀速卡尔曼（位置量测，赛题一主力）
  estimation/ekf.py                 # ImmFilter: rust 优先，纯 Python 后备
  estimation/imm_py.py              # ImmFilter 纯 Python 移植（700 行，rust 缺失时回退）
  estimation/motion.py              # ols_speed_mps（12s 窗口最小二乘速度判别）
  estimation/geometry.py            # bearing_rad, haversine_m
  search/route_prior.py             # 26 条路线有序烘焙 + match_route/predict_position/predict_velocity
  search/spiral.py                  # 阿基米德螺旋航点（无先验回退用）
  search/lawnmower.py               # 往返式覆盖航点（割草机，备选）
  search/target_roads.py            # 沿 points.json 26 条路线烘焙航点（赛题二用）
  tracking/gimbal.py                # compute_gimbal_angles（机体系 pan，必传 uav_heading_deg）

tests/
  test_coop_decoy_agent.py          # 18 个：状态机/通信/VERIFY/已摧毁记忆/心跳/避让等
  test_search_track_agent.py        # 6 个：生命周期/搜索/跟踪/LOST 重捕获/引擎时间
  test_swarm_agent.py               # 14 个：赛题三骨架
  test_imm_py.py                    # 3 个：纯 Python 后备数值/回归测试
  test_imm.py, test_geometry.py     # 原有基础测试
  test_lawnmower.py                 # 5 个：割草机覆盖/对称/sector 拆分
  test_coverage.py                  # 原有 Voronoi 测试
  test_protocol.py, test_ekf.py, test_gimbal.py, test_spiral.py, test_follow.py  # 原有
```

---

## 5. 运行指令

```bash
# 测试
.venv/Scripts/python -m pytest tests/ -q          # 170 全过

# 跑赛题（默认新引擎、干净状态）
powershell -NoProfile -File run/run_fast.ps1 -Duration 600 -Output output/myrun
# 指定赛题/Agent
powershell -NoProfile -File run/run_fast.ps1 \
  -Scenario coop_decoy -Agent competition.user_algorithms.coop_decoy.agent:CoopDecoyAgent \
  -Duration 600 -Output output/coop_600
powershell -NoProfile -File run/run_fast.ps1 \
  -Scenario search_track -Agent competition.user_algorithms.search_track.my_agent:MySearchTrackAgent \
  -Duration 600 -Output output/st_600

# 复用已运行引擎（最快迭代，前提是已在跑）
powershell -NoProfile -File run/run_fast.ps1 -Duration 180 -ReuseEngine

# 清残留引擎（关键！复用测试局前）
taskkill //IM opensim-sim.exe //F

# 评估 json 在 output/<runname>/<scenario>_<unix>.evaluation.json
```

---

## 6. 未解决 & 下一步

| 优先级 | 任务 | 备注 |
|---|---|---|
| 高 | 赛题二协锁 dwell 稳健性 | 判毁逻辑修好后链路可达协锁（v25 201 coop_ticks），但检测中断 >2s 即 reset。方向：TRACK 云台瞄准/盘旋几何降低锁中断（统计中断相对 2s grace 的分布）；协锁同时性验证（僚机 400m 圈与长机 100m 圈）；本地 dwell gap 容忍与评估器 grace 对齐 |
| 高 | 赛题二 5 m/s 档判别 | 1 真 + 15 诱饵同速档，速度带不可用。方向：停顿模式（真目标有 WaitTime 停顿，诱饵恒 5 m/s 无停顿）；验证集目标还有躲避能力（官方描述） |
| 高 | 赛题三仿真验证 + 判别策略 | **K=3（非 1）**：`adversarial_swarm/runner.py:36 DEFAULT_K=3`，需 3 架同时盯防同一目标 20s 才摧毁（官方赛题描述一致；guide-v2 的"协同阈值 1"已过时）。骨架 agent 按 K=1 设计，需改协同汇聚逻辑。目标 4-8 m/s vs 诱饵 5 m/s 速度档完全重叠，速度带不可用；需走"路线/停顿模式" |
| 高 | 验证集路线泛化 | 手册 §5「目标小车路线会采取新路线」；`points.json` 烘焙失效；需程序化提取 `GridDataAll_18.csv`（41MB）的路网中心线，或退回割草机 + 高频扫描。赛题一路线匹配失败时已回退螺旋（单测覆盖，仿真未实测） |
| 高 | 验证集天气变化 | 官方：验证环节目标轨迹**和天气**随机变化。训练局全是 Rain（scenario 默认）；WEATHER_FACTORS 按天气缩放检出率和噪声（`default_detectors.py`），CvFilter 的 R=50² 在好天气偏保守（可接受），坏天气噪声放大需关注 |
| 中 | 赛题二多种子均值 | 单局方差极大（0~17.9），任何策略改动需 5+ 局均值才有结论；跑前确认无残留引擎（run_fast 已自动清理） |
| 中 | 赛题三心跳/任务重分配 | 心跳（同赛题二 P:）+ 失联判定 + 任务拍卖/重分配 |
| 中 | ImmFilter 纯 Python 后备实战验证 | 赛题一/二已改用 CvFilter（纯 Python），不再依赖 ImmFilter；rust_core 缺失时 CvFilter 同样可用 |

---

## 7. 踩过的坑（避免再踩）

1. **残留引擎会污染后续所有仿真局（测量无效化的头号环境坑）**。runner 异常退出（如 v13 夭折）后其引擎不死，继续向同一 Redis 发布 sim:state；新 runner 收到两个场景的交替状态流——`sim_time` 来回跳变（调试日志里 t 在 0.0 与真实值间振荡）、冷却/计时全乱、proximity 边沿计数爆炸（v14~v18 的 200~300 次）、dwell 永不累计。**v11~v18 的零分/异常大多是测量假象，不是 agent 行为**。`run_fast.ps1` 已在非 `-ReuseEngine` 模式启动时自动 `Stop-Process` 清理残留引擎；手工跑前也应 `taskkill //IM opensim-sim.exe //F`。
2. **`point_gimbal` 的 pan 是机体系相对方位（机头=0），不是绝对方位**。官方 baseline（`swarm_coordinated.py:446`）都扣航向：`pan = ((brg - heading + 180) % 360) - 180`。`compute_gimbal_angles` 此前返回绝对 bearing，三个赛题的 Agent 云台全部指偏一个航向角——UAV 静止朝北时碰巧能对，一转向目标就出 FOV，表现为"检测 2 秒后长期丢锁"（跟踪反复中断的头号根因）。已修：`compute_gimbal_angles(..., uav_heading_deg=)` 返回机体系 pan。
2. **赛题一评分只看 1Hz 上报精度（D_max=30m），漏报记 0**。报错和漏报同为 0，所以有任何位置假设就该每秒上报——目标出生后在路线 Start 停 30s（points.json 全部路线 Start.WaitTime=30），开局报出生点约 30s 满分窗口。VERIFY 判别（赛题一无诱饵）和"等 EKF 收敛再上报"都是纯丢分。
3. **`briefing.target_initial_pos` 恒为目标真实出生点**：`prepare_scenario` 在场景随机化**之后**把所选路线 Start 写进实体初始位，引擎按 prepared scenario 生成目标（seed=0 无平移；seed>0 时 UAV 被平移、目标仍在原坐标系路线 Start）。且可匹配 points.json 26 条路线 Start → 目标全程位置可预测（`algorithms/search/route_prior.py`）。此前"坐标陷阱"的结论不成立（当时是地形缺失冻结局的误诊）。
4. **bearing-only ImmFilter 在本场景发散**：检测直接给经纬度（完整位置量测 ±50m），bearing+range 更新的 ImmFilter 实测两帧跳离量测 ~200m。位置量测下双轴匀速卡尔曼（`algorithms/estimation/cv_kalman.py`）才是最小正确工具；速度初值方差取 25 防过冲，有路线先验时带先验速度初值消除斜坡滞后。
5. **agent 时间基准必须用 `briefing.score_view.sim_time`（引擎真时间）**。runner 控制节拍远快于引擎（实测 2.5×），所有时间相关计算（OLS 速度、报告节拍、协同超时、冷却、滤波 predict 的 dt）**必须**用 `score_view.sim_time`，否则 `dt` 累加会让一切失真。
6. **本地时钟 ≠ 评估器口径，按本地状态"宣布胜利"会拆掉协同**。赛题二旧逻辑：长机本地 dwell 满 20s 就把目标标记"已摧毁"并离场，但评估器按 **K=2 同时**协锁 20s 判毁——长机一离开，协锁永远凑不齐，且 nd=True（已摧毁记忆）让它永久拒绝返回（debug5 局实测）。凡涉及评估口径的状态（摧毁/完成），必须以评估器同口径的条件为准（队友在场的新鲜占位为凭），不能只看本地计时。
7. **VERIFY 接触丢失中止 ≠ 判别否决，混用升档冷却会形成死亡螺旋**。中止是"没看清"（无结论），否决是"判了诱饵"（有结论）。原实现中止也走 20→40→80→160s × 500m 升档冷却，密集车场里几次中止就把全场永久锁死（debug 局实测：三机 240s 零 VERIFY，rcd 恒真）。修法：中止 5s 平冷却 + 300m 半径（`_mark_abort`），否决保持升档（`_mark_reject`）。接触丢失的直接原因是 VERIFY 无检测拍不下发云台指令——UAV 40 m/s 接近中 LOS 快速变化，相机偏出 FOV；无检测拍必须持续指向滤波/最后已知位置。
7. **诱饵有 5 m/s 路线**。官方 `inject_astar_decoy decoy_speed=5.0`（`competition/sdk/scenarios/coop_decoy/runner.py:218`），"静止即诱饵"是错的。必须用最小二乘速度判别，且诱饵能跨过 [3.5, 6.5) 阈值。
7. **目标路线有 WaitTime 停顿**（`config/points.json` road1 start=30s）。停顿中速度=0，对应"永久诱饵"在判别读数上不可分。永远不要用位置标记跳过候选——必须用时间冷却。
8. **检测位置 ±50m 高斯噪声**（`config/scenarios/coop_decoy/algorithm.yaml: noise_sigma_m: 50.0`）。短窗口（<8s）的速度估计失真大；12s 窗口 OLS σ_ols ≈ 1.3 m/s。
9. **announce 时机决定全队行为**。VERIFY 入口 announce 让 3 架机围着同一诱饵空转（验证了 50 次判决 49 次判同一车）；应只在 TRACK 入口 announce。
10. **proximity 扣分是清的**（15 分上限）。`<200m` 每次扣 2 分，600s 局 8 次=清零。心跳避让 + 僚机 350m 入 VERIFY 后降至 1 次。
11. **锁跳变虚高**会假阳。OLS 上限 13.5 m/s（地面极速 12）过滤掉大多数跳变/UAV 过顶虚高。
12. **RUN_RNG_SEED=-1** 是开（不可复现）。改用 seed=N（≥0）可复现。性能/质量评估需要多种子均值。
13. **decide 禁文件 IO**（手册 §6.1），但 `briefing` 字段、`message.payload`、`obs.self.detection` 之外的位置 (`briefing.score_view`) 是合法信息。

---

## 8. 环境快照

- `.venv/` Python（含 redis 客户端、pytest）
- `bin/redis-server.exe`、`bin/redis-cli.exe` 6382 / 6379 端口
- `opensim-sim.exe` 引擎二进制（fixed-wing C++ 仿真）
- 评分 `output/<run>/<scenario>_<unix>.evaluation.json`：`n_destroyed`、`n_reports`、`targeting_rmse_m`、`dimension_scores {kill, accuracy, mission_time}`、`base_score`、`penalty`、`per_target {coop_ticks, dwell_accumulated_s, resets}`
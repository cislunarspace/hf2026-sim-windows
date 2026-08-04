# 交接：coop_decoy 杀敌率优化（mock 快速仿真 + 编队打击 + 上报精度）

## 当前状态

目标：赛题二 coop_decoy 拿高分（3/3 全杀、base≥70）。全部改动在 dev 仓库（`D:\codes\hf2026-sim-windows`），未 commit（用户此前要求不 commit）；stand 仓库（`D:\codes\hf2026-sim-stand-windows`）的 agent.py 已同步（未跟踪文件）。

**当前代码版本 v14**（`competition/user_algorithms/coop_decoy/agent.py`）：在 v12（同心圈 250/500、编队打击、上报三道闸门）基础上：
- **僚机云台自身判真优先**：僚机 TRACK 时自身滤波判真（速度落 7-14.5 带 + 检测新鲜 ≤1s + 收敛 10s）→ 瞄自身滤波；否则瞄 leader 报告。原逻辑只在"未确认 announce"时信自身，确认时纯 leader 报告（0.5s 滞后 + 30-50m 误差 → 僚机 500m 外瞄偏 5-8° → 诱饵抢锁）。
- **长机广播预测位置**：T: 广播用滤波位置 + 速度外推 0.5s（`velocity_mps()`，减 6m 滞后）。

**mock 验证（v14，共享可见性模型）**：2/8 局有杀（seed4/7，10003 dwell 20s 击杀），**协锁连续性提升（resets=0，此前 1-2 次中断清零）**——双机锁定时段重叠的直接证据。全量 `pytest tests/ -q` 267 全绿（含 `_FakeFilter.velocity_mps` 补丁）。

**真实局历史（8 局全 0 杀，校准逐层逼近）**：
- 独立搜索版（v7）：真目标被锁 0 tick；
- v11 编队版：真目标被锁 320 tick；
- v12（A* 相邻段校准）：真目标被锁 714 tick（20003 TRACK 10002 达 10.7s）、base 8.38；
- **v13 同侧编队已证伪回退**：僚机绕 leader 视线外推点 → 真目标被锁 714→0、penalty 15（外推 400m 让僚机距目标 650m，相关性收益被距离/滞后抵消）；
- **v14 真实局未验证**（后台任务被中断，需重跑）。

**关键工具（本次会话核心交付）**：
- **mock 快速仿真引擎**：`competition/sdk/core/mock_client.py`，`--mock` 参数启用（`run/run_mock.ps1`）。600s 局墙钟 1.5s（≈160 倍提速）。已校准：检测丢失模型（`detect_loss=0.50` 默认，真实 TRACK 状态 det0≈49%）、**同目标共享可见性**（时间+目标 uid 确定性哈希，同目标同拍所有机共享同一判定——丢失相关性可被 mock 体现）、A* 相邻段路网折线（真实 astar_plan 是相邻 waypoint 间的路网短段）。
- **探针**：`COOP_EVAL_DEBUG=1` 产出 `output/eval_debug.csv`（评测器状态）/`eval_debug2.csv`（fov/云台/锁定/FSM，列序：U2,t,uid,fov,pan,tilt,det,conf,misid,matched,eff,is_decoy,sep,rng,n_true,n_true_in,n_decoy_in,nt_sep,nt_rng,nd_sep,nd_rng,lat,lon,alt,heading,state,wingman,confirmed,...）。真实局前删旧文件（追加模式）。

## 下一步建议（按优先级）

1. **重跑真实 v14 确认局**（最重要——v14 未验证）。命令：`powershell -File run/run_fast.ps1 -Duration 600 -Seed 7 -Output output/coop_real_600_v14_s7`（墙钟 ~17 分钟，后台跑）。**验收**：`coop_ticks > 0`（双机同锁真目标时段出现）即方向正确；`n_destroyed ≥ 1` 为达标。若 0 杀，用 `eval_debug2.csv` 分析：真目标被锁 tick（对照 v12 的 714）与双机同锁重叠。
2. **若 v14 真实仍 0 杀**：分析"双机从不 TRACK 同一目标"（v12 局 20003 锁真目标时 20002 在 TRACK 但锁 None）——僚机 JOIN 到达后的锁定失败。候选（mock 先行，每局 1.5s）：
   - JOIN 圈 500→450m（与长机 250 圈差 200m 罚线边缘，需 `_avoid_teammates` 兜底）；
   - 僚机 TRACK 圈 500→600m（更远更稳？需对照）；
   - 长机 announce 加"自身锁真目标"确认位（现 confirmed 只表达 OLS 判真，不表达"当前仍锁着"）。
3. **5 m/s 档（10001）**：8 seed 里 10001 从未被杀。验证式跟踪依赖 dwell 满 20s（50% 丢失下断续难满，尾段永不触发）。候选：dwell 判定改"累计 ≥15s 且最近 2s 无 >2s 间隙"（评测器 grace 语义对齐）；probe 验证尾段触发条件改时间驱动（track_time ≥25s 且最近 5s 无 >2s 间隙）。**注意：probe 时间驱动"放弃"通道已证伪（30s/45s 均把双杀局打回 0 杀）——只做"确认"不做"放弃"**。
4. **收尾**：mock/真实达标后全量 `pytest tests/ -q`（当前 267 全绿），按用户指示决定是否 commit；同步 stand agent.py（`cp competition/user_algorithms/coop_decoy/agent.py D:\codes\hf2026-sim-stand-windows\competition\user_algorithms\coop_decoy\agent.py`）。

## 依赖关系

- mock 引擎/探针（独立，已完成）——所有策略迭代的前置。
- v14 真实确认（独立，最先做）→ 结果决定下一步方向（0 杀则挖配对/锁定，有杀则冲 3/3）。
- 5 m/s 档候选依赖 mock（每局 1.5s）；真实 600s 确认（17 分钟/局）只做最终验证。
- 跑真实局前删 `output/eval_debug*.csv`（追加模式，避免混合）。

## 关联 issue

- 无 issue tracker 条目；原始任务文档：`docs/task4-discrimination-redesign.md`（判别重设计）、`docs/task5-coop-decoy-fixes.md`（时间轴/report/验证式跟踪/区域覆盖）、`docs/handoff-coop-decoy-engine-tuning.md`（本交接归档）。

## 可参考技能

- `diagnosing-bugs`：探针数据分析（eval_debug2.csv 列序见上）；真实局异常提前退出时先查残留进程（`tasklist` 清理 opensim-sim/python）。
- `implement`：按上述候选改 `agent.py` 时使用。

## 关键文件

- dev agent：`D:\codes\hf2026-sim-windows\competition\user_algorithms\coop_decoy\agent.py`（v14）
- mock 引擎：`D:\codes\hf2026-sim-windows\competition\sdk\core\mock_client.py`（`--mock` 参数，`run/run_mock.ps1`）
- 探针：`D:\codes\hf2026-sim-windows\competition\sdk\core\_coop_lock_debug.py` + runner.py 调试块
- 快速跑局：`run/run_fast.ps1`（真实引擎）、`run/run_mock.ps1`（mock）
- 最近产物：`output/mock_v14_s*`（mock 2/8 有杀）、`output/coop_real_600_v12b_s8`（真实 v12 基线：真目标被锁 714 tick）

## 会话日志 2026-08-01：真实 v14 确认 + 候选 mock 验证（结论：v14 方向正确、真实 0 杀；EXP-A/B/H mock 占优但真实回归 → 保留 v14）

### 真实局结果（seed 7，600s，全部 0 杀）
| 配置 | n_destroyed | coop_ticks | reports | 真目标 eff 锁行 | 备注 |
|---|---|---|---|---|---|
| **v14（确认局）** | 0 | **89（10003）** | 107 | 2185（20003:10002 1822 等） | 方向正确（coop>0）；双机同锁 10003 曾到 dwell 6s 后被 >2s 间隙清零；僚机随后锁到附近诱饵 30013（锁模型=离光轴最近） |
| EXP-A+B | 0 | 0 | 0 | 766 | 68% 时间在 TRACK（多为诱饵/过期位置），从未有效协锁——EXP-A 诱饵追逐在真实引擎明显恶化 |
| EXP-A+B+H | 0 | 0 | 25 | ~0 | 同上；rmse 273.8 |

**关键教训**：mock 对 EXP-A 系过度乐观（真实引擎诱饵抢锁/脱锁比 mock 的"离光轴最近"角度模型更苛刻）；同 seed 因诱饵未种子化随机（`prepare_scenario` 的 `decoy_rng = _random.Random()`）每局场景不同，单局对比不可靠，mock 需多 seed 多轮看分布。

### mock 候选实验（每 batch = 每 seed 一局，decoy 随机 → 组间仅可比分布）
| 配置 | 样本 | 击杀/局 | 双杀局 | 10001（5 m/s）击杀 |
|---|---|---|---|---|
| v14 基线 | 16 局 | 4/16 | 0 | 0 |
| EXP-A（probe announce 僚机直入 TRACK，不再 VERIFY） | 16 局 | 3/16 | 0 | 0 |
| **EXP-A+B（+probe 尾段时间驱动确认：track_time≥25s 且最近 5s 无 >2s 间隙，只做确认不做放弃）** | 24 局 | 7/24 | 3 | 0 |
| EXP-A+B+C（僚机 TRACK 圈 500→600m） | 8 局 | 0/8 | 0 | 0（**证伪**，与 v13 教训一致：更远=更差） |
| **EXP-A+B+H（+announce 第 4 段"长机当前仍锁真目标"位；长机脱锁时僚机改信自身滤波）** | 20 局 | **8/20** | 0 | **2（s3@41s、s16@527s）** |

mock 内 EXP-A+B(+H) 明显更优（双杀、偶杀 10001），但真实 3 局全 0 且 EXP-A 系 0 coop → **决定回退保留 v14**（真实最稳：有 coop 有上报）。全部改动未 commit（沿用用户此前要求）。

### 会话交付物
- dev/stand `agent.py` 均已回退为 v14（stand 与 dev 字节一致，无需再 cp）。
- 实验工具留存：`output/agent_v14_backup.py`（v14 基准）、`output/patch_agent.py`（`baseline` / `exp_a` / `exp_b` / `exp_a_b` / `exp_a_b_h` / `exp_c` 可复现，`python output/patch_agent.py <op>`）、`run/run_mock_batch.ps1 -Tag <tag> -Seeds "3;4;..."`（批量 mock+汇总）。
- 全量 `pytest tests/ -q`：267 全绿（v14 与各 EXP 组合均验证）。
- 真实局产物：`output/coop_real_600_v14_s7`（0 杀，coop 89）、`output/coop_real_600_expab_s7`、`output/coop_real_600_abh_s7`。
- 跑真实局前须删 `output/eval_debug*.csv`（追加模式）；当前保留最后真实局探针文件可作参考。

### 下一步候选（若继续冲 3/3，需先解决"真实引擎双机同锁 20s 不可持续"）
1. 真实引擎里双机配对后在 ~80-100s 内保持 ≥2 架同时有效锁（50% 单机检出下双机同拍 ≈25%，20s 累计 dwell 需要近 100s 不断档）——当前最缺的是"配对不散"（v14 局：20001 JOIN 到达后 20002 锁到诱饵即拆对）。
2. 方向：wingman 云台不再锚定 leader 报告而是"检测点 + 目标判真一致性校验"（检测点距 leader 目标 <150m 且 leader 仍锁 → 瞄检测点，0 延迟）；或验证式跟踪的配对专用 announce 重发策略（mock 已证 EXP-A 直入 TRACK 有效但真实回归，需真实引擎下先复现 mock 的双杀机制再上）。
3. 10001（5 m/s）在 mock 仅 EXP-A+B+H 击杀过 2 次——真实引擎该档可能需先验证式跟踪配对才有戏；风险是诱饵死盯（probe 90s 超时兜底）。

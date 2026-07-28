# 文献知识库 — 红枫 2026 无人集群竞赛

## 概述

本知识库收录 33 篇文献的提取摘要，按主题分类整理，为算法选型和技术报告提供文献支撑。

**知识库结构**：
- `01-perception/`：感知与视觉检测
- `02-estimation/`：状态估计与滤波
- `03-search/`：搜索与覆盖规划
- `04-coordination/`：多机协同与任务分配
- `05-adversarial/`：对抗、干扰与博弈
- `06-path-planning/`：路径规划与威胁规避
- `07-tracking/`：目标跟踪（单目标与集群）
- `summaries/`：主题汇总综述

**使用方式**：
1. 先阅读 `summaries/` 下的主题综述，了解各方向的研究现状
2. 根据需要查阅具体文献的提取摘要
3. 在算法选型讨论中引用相关文献

---

## 主题索引

### 一、感知与视觉检测（01-perception/）

**核心问题**：航拍场景下的目标检测与跟踪识别

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 1 | jiangbo-2021-dl-aerial-detection.md | 基于深度学习的无人机航拍目标检测研究综述 | 2021 | YOLOv5/v7、CenterNet、小目标检测 |
| 29 | liuyan-2025-cooperative-perception-survey.md | 战场环境智能无人集群协同感知关键技术综述 | 2025 | 分布式感知、目标识别、协同处理 |
| 30 | zheng-2026-distributed-cross-domain.md | 跨域无人集群分布式自组织协同多目标跟踪 | 2026 | 海空协同、分布式一致性、Voronoi覆盖 |
| 31 | zhangzhongmin-2026-bytetrack-yolov10.md | 基于改进ByteTrack与YOLOv10的无人机多目标跟踪算法 | 2026 | ByteTrack+YOLOv10、小目标优化 |

**主题综述**：[summaries/05-perception.md](summaries/05-perception.md)

---

### 二、状态估计与滤波（02-estimation/）

**核心问题**：Bearing-Only Tracking、EKF/UKF/粒子滤波、IMM 交互多模型

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 2 | 2026-dl-single-target-tracking-survey.md | 基于深度学习的无人机单目标跟踪综述 | 2026 | SiamRPN++、TransT、OSTrack、DeepSORT |
| 3 | zhuoli-2021-uav-single-tracking-survey.md | 无人机影像单目标跟踪综述 | 2021 | SiamFC、ECO、KCF、相关滤波 |
| 4 | mai-2025-uav-visual-tracking.md | UAV视觉跟踪：挑战与进展 | 2025 | 深度学习跟踪器、移动端轻量化 |
| 5 | jiaosongming-2023-siamrpn-tracking.md | 基于SiamRPN的无人机目标跟踪及控制算法 | 2023 | SiamRPN+PID控制、57.3FPS |
| 27 | huangyuhang-2025-multiscale-tracking.md | 融合多尺度全局-局部特征的无人机目标跟踪算法 | 2025 | 多尺度特征融合、T16指标提升 |

**主题综述**：[summaries/01-estimation.md](summaries/01-estimation.md)

---

### 三、搜索与覆盖规划（03-search/）

**核心问题**：协同覆盖、概率搜索、通信受限下的路径规划

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 6 | chenyang-2024-coverage-limited-comm.md | 通信受限条件下多无人机协同覆盖规划 | 2024 | Voronoi划分+Lloyd优化、matroid约束 |
| 7 | fanruitao-2026-game-path-planning.md | 博弈环境下的多无人机协同路径规划 | 2026 | Stackelberg博弈、CPP-Stackelberg混合算法 |
| 8 | wangziquan-2023-weak-info-decision.md | 弱信息交互条件下的无人机集群决策方法 | 2023 | 共享目标列表策略、分布式马尔可夫决策 |

**主题综述**：[summaries/02-search.md](summaries/02-search.md)

---

### 四、多机协同与任务分配（04-coordination/）

**核心问题**：多机协同目标跟踪、分布式任务分配、CBBA

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 9 | du-2025-uav-swarm-survey.md | UAV自主智能集群综述 | 2025 | 强化学习、群体智能、混合架构 |
| 10 | page-2025-coop-target-tracking.md | 无人机集群协同目标跟踪 | 2025 | 分布式自适应控制、置信度评估 |
| 11 | jiao-2025-marl-evolution-tracking.md | 进化多智能体RL用于UAV多目标跟踪 | 2025 | MARL+进化算法、MARL-EVO框架 |
| 12 | sun-2026-two-stage-swarm-tracking.md | 两阶段框架：监督+迁移学习用于UAV集群多目标跟踪 | 2026 | 轨迹预测+分布式追踪、11.3%RMSE改善 |
| 13 | wang-2026-joint-search-attack.md | 联合搜索-攻击决策方法 | 2026 | 状态转移机理、搜索-攻击决策融合 |
| 14 | panzishuang-2024-dynamic-coalition.md | 基于动态一致性联盟算法的异构集群协同作战联盟组建 | 2024 | 信息素机制、动态联盟重构 |
| 15 | hushengrong-2026-urban-strike-allocation.md | 城市环境多无人机协同打击任务分配与路径规划 | 2026 | 改进匈牙利算法、MQGA路径规划 |

**主题综述**：[summaries/03-coordination.md](summaries/03-coordination.md)

---

### 五、对抗、干扰与博弈（05-adversarial/）

**核心问题**：通信干扰、威胁规避、多机对抗决策

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 16 | jadeli-2026-emi-resilience.md | 电磁干扰下UAV集群韧性 | 2026 | 模拟退火、10架UAV韧性评估 |
| 17 | xue-2025-antijam-formation.md | 反干扰编队跟踪混合策略 | 2025 | Stackelberg博弈、MADDPG混合策略 |
| 18 | yuxu-2025-pigeon-marl-navigation.md | 鸽群启发MARL用于对抗环境UAV集群导航 | 2025 | Pigeon-MARL、势函数最优编队 |
| 19 | xuanshuzhe-2021-swarm-game-survey.md | 无人机集群对抗博弈综述 | 2021 | 博弈论、多智能体强化学习 |
| 20 | xuejian-2024-incomplete-info-survey.md | 非完全信息下无人机集群对抗研究综述 | 2024 | 元博弈框架、数据驱动建模 |
| 21 | liwei-2024-adversarial-decision-survey.md | 无人机集群对抗决策算法研究综述 | 2024 | 动态对抗网络、对抗性任务分配 |
| 22 | gaoxianzhong-2023-rule-ai-training.md | 规则与智能耦合约束训练方法 | 2023 | 规则与RL耦合、对抗策略生成 |
| 23 | lirong-2022-heterogeneous-adversarial.md | 异构无人机集群对抗决策研究 | 2022 | 多层级异构建模、攻防对抗动态 |
| 24 | limengmeng-2024-drl-adversarial.md | 基于深度强化学习的无人机集群对抗算法研究 | 2024 | QPLEX+平均场、多层级对抗架构 |

**主题综述**：[summaries/04-adversarial.md](summaries/04-adversarial.md)

---

### 六、路径规划与威胁规避（06-path-planning/）

**核心问题**：威胁规避、轨迹优化、协同路径规划

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 25 | zhengjuhong-2026-elpio-path-planning.md | 超低空电磁威胁域无人机群ELPIO协同路径规划 | 2026 | 电子战组合、ELPIO框架 |
| 26 | song-2026-dmp-trajectory.md | 政策搜索优化DMP的多UAV协同目标跟踪轨迹规划 | 2026 | 动态运动基元(DMP)、策略搜索优化 |

**主题综述**：[summaries/06-path-planning.md](summaries/06-path-planning.md)

---

### 七、目标跟踪：单目标与集群（07-tracking/）

**核心问题**：视觉目标跟踪、集群协同跟踪、跟踪与避障融合

| # | 文件名 | 标题 | 年份 | 主要内容 |
|---|--------|------|------|----------|
| 28 | xuexirui-2025-swarm-tracking-survey.md | 无人机集群目标跟踪方法研究综述 | 2025 | 通信拓扑、相对定位、协同控制律 |
| 32 | lizhiwei-2026-drl-tracking-obstacle.md | 基于深度强化学习的无人机目标跟踪和障碍规避融合控制 | 2026 | PPO+ANNs、USV/UAV协同 |
| 33 | lirong-2022-heterogeneous-adversarial.md | 异构无人机集群对抗决策研究 | 2022 | 多层级异构建模、攻防对抗动态 |

---

## 文献质量说明

### 需要人工核实的内容

- **字符丢失问题**（薛锡瑞 2025 等）：部分文献存在括号内容、年份、数字缺失，需要与原文 PDF 对比核实
- **格式问题**：英文连字符断词需人工合并，图表引用格式不一致需标准化
- **分类说明**：第 33 篇文献同时归入 05-adversarial 和 07-tracking，反映其跨学科特性

### 后续生成

| 综述文件 | 对应主题 | 状态 |
|----------|----------|------|
| [01-estimation.md](summaries/01-estimation.md) | 状态估计与滤波 | ✅ 已完成 |
| [02-search.md](summaries/02-search.md) | 搜索与覆盖规划 | ✅ 已完成 |
| [03-coordination.md](summaries/03-coordination.md) | 多机协同与任务分配 | ✅ 已完成 |
| [04-adversarial.md](summaries/04-adversarial.md) | 对抗与干扰 | ✅ 已完成 |
| [05-perception.md](summaries/05-perception.md) | 感知与检测 | ✅ 已完成 |
| [06-path-planning.md](summaries/06-path-planning.md) | 路径规划与威胁规避 | ✅ 已完成 |

---

## 知识库统计

- **文献总数**：33 篇
- **提取摘要**：33 份
- **主题综述**：6 份
- **覆盖年份**：2021-2026
- **来源**：MinerU 转换的 markdown

---

## 使用指南

1. **快速了解**：先阅读 `summaries/` 下的主题综述
2. **深入研究**：根据需要查阅具体文献的提取摘要
3. **算法选型**：在讨论中引用相关文献和数据
4. **技术报告**：引用文献支持算法选择和参数设置

---

**知识库更新时间**：2026-07-28
**维护者**：@ouyangjiahong
**项目仓库**：[hf2026-sim-windows](https://github.com/cislunarspace/hf2026-sim-windows)

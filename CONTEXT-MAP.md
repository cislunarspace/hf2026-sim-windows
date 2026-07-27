# Context Map — 红枫 2026 无人集群竞赛

## 子上下文索引

| 子上下文 | 路径 | 说明 |
|---|---|---|
| 仿真平台 SDK | `competition/docs/` | 平台 API、评分规则、感知指南 |
| 算法模块 | `algorithms/` | 搜索/跟踪/鉴别/协同/规避/估计 |
| Rust 计算服务 | `rust_core/` | PyO3 加速：滤波、路径规划、任务分配 |
| 参赛提交 | `competition/user_algorithms/` | 最终提交的 Agent 入口文件 |
| 技术报告 | `report/技术报告/` | LaTeX 技术报告 |

## 关键决策（`docs/adr/`）

| ADR | 标题 | 状态 |
|---|---|---|
| ADR-001 | 语言选择：Rust 核心 + Python 集成层 | 已决 |
| ADR-002 | 赛道策略：基于赛题三设计，自底向上实现 | 已决 |
| ADR-003 | 代码按功能划分，多文件打包 | 已决 |
| ADR-004 | 分层架构：Agent→状态机→算法模块→计算服务 | 已决 |

## 辅助文档

| 文档 | 路径 | 说明 |
|---|---|---|
| 术语表 | `docs/glossary.md` | 核心术语定义 |
| 环境验证 | `docs/env-verification.md` | 环境检查报告 |
| 文献关键词 | `docs/literature-keywords.md` | 感知/决策文献检索指南 |

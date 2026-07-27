# ADR-003: 代码按功能划分（非按语言）

## 状态

**已决** — 多文件打包，按功能划分模块

## 背景

参赛者要求最终代码按功能划分文件夹，不按语言类别划分。同时提到核心算法可能用 Rust。

## 决策

代码目录结构按**功能模块**组织，而非按 `python/` 和 `rust/` 划分。

```
D:\codes\hf2026-sim-windows\
├── competition/
│   └── user_algorithms/         # 平台要求的算法提交位置
│       ├── search_track/my_agent.py
│       ├── coop_decoy/my_agent.py
│       └── adversarial_swarm/my_agent.py
│
├── algorithms/                  # 按功能划分的共享模块
│   ├── search/                  # 搜索算法
│   │   ├── spiral.py            # 螺旋路径生成
│   │   ├── grid.py              # 网格分配
│   │   └── coverage.py          # 覆盖路径规划
│   ├── tracking/                # 跟踪算法
│   │   ├── gimbal.py            # 云台控制（LOS、扫描）
│   │   ├── follow.py            # 飞行跟随（前瞻、盘旋）
│   │   └── report.py            # 目标上报策略
│   ├── discrimination/          # 诱饵鉴别
│   │   ├── kinematics.py        # 运动学分析（具体算法待定）
│   │   ├── filter.py            # 位置滤波（具体算法待定）
│   │   └── decision.py          # 鉴别决策（具体算法待定）
│   ├── coordination/            # 多机协同
│   │   ├── protocol.py          # 通信协议编解码
│   │   ├── allocation.py        # 任务分配（具体算法待定）
│   │   └── consensus.py         # 分布式共识（具体算法待定）
│   ├── threat/                  # 威胁规避
│   │   ├── zone.py              # 威胁区判断
│   │   ├── avoidance.py         # 规避路径（具体算法待定）
│   │   └── jamming.py           # 干扰处理
│   └── estimation/              # 状态估计
│       ├── kalman.py            # 滤波器（具体方案待定）
│       ├── particles.py         # 粒子滤波器（具体方案待定）
│       └── geometry.py          # 几何计算
│
├── rust_core/                   # Rust 计算服务
│   ├── src/
│   │   ├── lib.rs               # PyO3 模块入口
│   │   ├── kalman.rs            # 卡尔曼滤波器
│   │   ├── particles.rs         # 粒子滤波器
│   │   ├── pathfinding.rs       # A* 路径规划
│   │   ├── assignment.rs        # 任务分配算法
│   │   └── geometry.rs          # 几何计算
│   ├── Cargo.toml
│   └── pyproject.toml
│
└── report/技术报告/              # LaTeX 技术报告
    ├── book.tex
    ├── title.tex
    └── package.tex
```

## 设计理由

1. 功能模块可跨赛题复用（filter.py 三道赛题都用）
2. Rust 代码集中在 rust_core/ 但通过 Python 接口暴露给 algorithms/ 使用
3. 入口文件放在平台要求的 user_algorithms/ 下，薄壳调用 algorithms/ 模块
4. 渐进实现：Phase 1（赛题一）只实现 search + tracking + 基础 estimation

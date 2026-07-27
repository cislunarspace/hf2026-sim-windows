# ADR-004: 框架架构（基于赛题三，自顶向下设计）

## 状态

**已决** — 基于赛题三设计通用框架，逐步实现

## 背景

三条赛道有递进关系：
- 赛题一：单机搜索+跟踪 → 基础模块
- 赛题二：+多机协同+诱饵鉴别 → 扩展模块
- 赛题三：+威胁规避+干扰处理+K=3 协同 → 完整框架

应从赛题三的完整需求出发设计架构，然后逐步实现。

## 架构设计

### 分层架构

```
┌─────────────────────────────────────────────────┐
│  Agent 层（Python，每赛题一个入口文件）           │
│  SearchTrackAgent / CoopAgent / SwarmAgent      │
├─────────────────────────────────────────────────┤
│  状态机层（Python）                               │
│  ACQUIRE → VERIFY → TRACK → SEARCH              │
├─────────────────────────────────────────────────┤
│  算法模块层（Python，按功能划分）                 │
│  搜索 / 跟踪 / 鉴别 / 协同 / 规避 / 通信        │
├─────────────────────────────────────────────────┤
│  计算服务层（Rust，PyO3 暴露给 Python）           │
│  滤波 / 路径规划 / 任务分配 / 几何计算（方案待定）│
├─────────────────────────────────────────────────┤
│  SDK 层（平台提供，不修改）                       │
│  Obs / Commands / Runner                        │
└─────────────────────────────────────────────────┘
```

### 功能模块划分

```
algorithms/
├── search/              # 搜索算法
│   ├── spiral.py        # 螺旋搜索路径生成
│   ├── grid.py          # 网格分配搜索
│   └── coverage.py      # 覆盖路径规划
├── tracking/            # 跟踪算法
│   ├── gimbal.py        # 云台控制（LOS 瞄准、扫描）
│   ├── follow.py        # 飞行跟随（前瞻补偿、盘旋）
│   └── report.py        # 目标上报策略
├── discrimination/      # 诱饵鉴别
│   ├── kinematics.py    # 运动学分析（方案待定）
│   ├── filter.py        # 位置滤波（方案待定）
│   └── decision.py      # 鉴别决策（方案待定）
├── coordination/        # 多机协同
│   ├── protocol.py      # 通信协议编解码
│   ├── allocation.py    # 任务分配（Greedy、拍卖）
│   └── consensus.py     # 分布式共识（K=3 认领）
├── threat/              # 威胁规避
│   ├── zone.py          # 威胁区判断
│   ├── avoidance.py     # 规避路径（爬升、绕行）
│   └── jamming.py       # 干扰处理
└── estimation/          # 状态估计
    ├── kalman.py        # 滤波器 Python 接口（方案待定）
    ├── particles.py     # 粒子滤波器 Python 接口（方案待定）
    └── geometry.py      # 几何计算（haversine、bearing）
```

### Rust 计算服务（具体算法待文献调研后决定）

```
rust_core/               # Rust 源码目录
├── src/
│   ├── lib.rs           # PyO3 模块入口
│   ├── filter.rs        # 滤波器（方案待定）
│   ├── pathfinding.rs   # 路径规划（方案待定）
│   ├── assignment.rs    # 任务分配（方案待定）
│   └── geometry.rs      # 几何计算（haversine、bearing）
├── Cargo.toml
└── pyproject.toml       # Maturin 构建配置
```

### 渐进实现路径（具体算法选型待文献调研后确定）

```
Phase 1: 赛题一（单机搜索+跟踪）
  ├── search/spiral.py
  ├── tracking/gimbal.py + follow.py
  ├── discrimination/filter.py（方案待定）
  ├── estimation/geometry.py（Rust）
  └── entry: search_track/my_agent.py

Phase 2: 赛题二（+诱饵鉴别+K=2 协同）
  ├── discrimination/kinematics.py + decision.py
  ├── coordination/protocol.py + allocation.py
  ├── estimation/滤波器（Rust 加速，方案待定）
  └── entry: coop_decoy/my_agent.py

Phase 3: 赛题三（+威胁规避+K=3 协同+干扰处理）
  ├── threat/zone.py + avoidance.py + jamming.py
  ├── coordination/consensus.py（K=3 认领）
  ├── estimation/路径规划（Rust，方案待定）
  └── entry: adversarial_swarm/my_agent.py
```

## 理由

1. 自顶向下设计保证架构一致性，避免赛题二/三推翻赛题一的设计
2. 功能模块可跨赛题复用（滤波器、几何计算、通信协议）
3. Rust 专注计算密集层，Python 负责逻辑编排
4. 渐进实现降低风险，每阶段都有可运行的交付物

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

### Rust 计算服务

```
rust_core/               # Rust 源码目录
├── src/
│   ├── lib.rs           # PyO3 模块入口
│   ├── ekf.rs           # 单模型 EKF（CV/CA/CT）
│   ├── imm.rs           # IMM 多模型交互滤波器
│   ├── pathfinding.rs   # 路径规划（Phase 3）
│   ├── assignment.rs    # 任务分配（Phase 2）
│   └── geometry.rs      # 几何计算（haversine、bearing、坐标转换）
├── Cargo.toml
└── pyproject.toml       # Maturin 构建配置
```

**estimation 模块选型（文献支撑：Mai 2025）**：

| 模型 | 状态向量 | 维度 | 运动假设 |
|------|----------|------|----------|
| CV (恒速) | [east, north, v_east, v_north] | 4D | 加速度=0 |
| CA (匀加速) | [east, north, v_east, v_north, a_east, a_north] | 6D | 加速度恒定 |
| CT (协调转弯) | [east, north, v_east, v_north, omega] | 5D | 转弯率未知 |

- IMM 将三个模型并行运行，通过交互和加权融合处理机动目标
- 坐标系：EKF 工作在局部切平面（米制），原点取 UAV 初始位置
- 计算开销：3×EKF ≈ 0.03ms/帧，10Hz 下完全可接受

## 状态机设计

### 六状态 FSM（文献支撑：Wang 2026 SECA + 基线优化）

```
ACQUIRE → SEARCH → VERIFY → ENGAGE → COORDINATE → ATTACK → (目标摧毁) → SEARCH
                                                ↓
                                              LOST → SEARCH
```

| 状态 | 含义 | 触发条件 |
|------|------|----------|
| ACQUIRE | 飞向目标初始位置 | 启动时 |
| SEARCH | 螺旋搜索 + 云台扫描 | 到达初始位置 / 丢失恢复 |
| VERIFY | 诱饵鉴别（多帧观察） | 检测到目标 |

**VERIFY 状态设计**（文献支撑：基线 coop_distributed）：
- 观察窗口：3 秒（~30 样本 @ 10Hz）
- 鉴别方法：EMA + 线性回归拟合速度
- 速度阈值：confirm ≥ X m/s，reject < Y m/s（具体值待测试确定）
- 超时处理：标记为"不确定"，分配给低优先级 UAV 继续观察
- 通信：广播确认的真目标/诱饵位置（D:lat,lon 消息）
| ENGAGE | EKF 初始化 + 收敛确认 | 鉴别为真目标 |
| COORDINATE | 跟踪 + 通信协调 | EKF 收敛 + 发送认领消息 |
| ATTACK | K 架同时盯防，累计 20s | K 架 UAV 确认同一目标 |
| LOST | 丢失恢复 | 检测丢失 > 2s |

### 任务分配方案

| 赛题 | 方案 | 理由 |
|------|------|------|
| 赛题一（单机） | 无需分配 | 单 UAV |
| 赛题二（3 机，K=2） | 简单规则 + Voronoi 划分 | CBBA 过重 |
| 赛题三（10 机，K=3） | CBBA + Voronoi 划分 | 分布式、可扩展 |

### 搜索分区方案（文献支撑：陈洋 2024）

| 赛题 | 方案 | 说明 |
|------|------|------|
| 赛题一 | 阿基米德螺旋 | 单机，从初始位置展开 |
| 赛题二 | Voronoi 划分 | 3 架 UAV，按位置自适应分区 |
| 赛题三 | Voronoi 划分 | 10 架 UAV，负载均衡 |

### 通信协议编码（50 字节限制）

| 字段 | 大小 | 说明 |
|------|------|------|
| 消息类型 | 1 byte | 'T'=真目标, 'D'=诱饵, 'A'=认领, 'J'=干扰 |
| 目标 ID | 1 byte | 0-255 |
| 纬度 | 4 bytes | float32 |
| 经度 | 4 bytes | float32 |
| 状态 | 1 byte | 0=发现, 1=确认, 2=认领 |
| **合计** | 11 bytes | 单条消息 |

50 bytes 内可放 4 条目标信息，满足赛题三 10 目标分批广播需求。

### 渐进实现路径

```
Phase 1: 赛题一（单机搜索+跟踪）
  ├── search/spiral.py                  # 阿基米德螺旋搜索
  ├── tracking/gimbal.py + follow.py    # LOS 瞄准 + 前馈飞行
  ├── estimation/ekf.py + imm.py        # IMM(CV+CA+CT) 滤波器（Rust）
  ├── estimation/geometry.py            # 几何计算（Rust）
  └── entry: search_track/my_agent.py

Phase 2: 赛题二（+诱饵鉴别+K=2 协同）
  ├── discrimination/kinematics.py + decision.py  # 诱饵鉴别
  ├── coordination/protocol.py + allocation.py    # CBBA 任务分配
  └── entry: coop_decoy/my_agent.py

Phase 3: 赛题三（+威胁规避+K=3 协同+干扰处理）
  ├── threat/zone.py + avoidance.py + jamming.py  # 威胁规避
  ├── coordination/consensus.py                    # K=3 认领
  └── entry: adversarial_swarm/my_agent.py
```

### 威胁规避策略（文献支撑：Xue 2025, Jade Li 2026）

| 威胁类型 | 方案 | 状态 |
|----------|------|------|
| SAM 防空区 | 待定（根据赛题具体情况） | TBD |
| 动态干扰区 | 优化飞离方向（非固定 600m） | 已决 |
| 多重威胁 | 按距离/威胁等级动态判断优先级 | 已决 |

**干扰飞离方向优化**：
- 基线方案：固定飞离 600m（方向随机）
- 优化方案：估计干扰源方向，向威胁最小方向飞离
- 通信：广播 J:lat,lon 消息通知队友

## 理由

1. 自顶向下设计保证架构一致性，避免赛题二/三推翻赛题一的设计
2. 功能模块可跨赛题复用（滤波器、几何计算、通信协议）
3. Rust 专注计算密集层，Python 负责逻辑编排
4. 渐进实现降低风险，每阶段都有可运行的交付物

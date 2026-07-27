# 文献调研关键词 — 感知与决策算法

## 一、目标检测与感知（Perception）

### 1.1 基础视觉目标检测

| 主题 | 英文关键词 | 中文关键词 |
|---|---|---|
| 实时目标检测 | `YOLO object detection`, `real-time detection`, `anchor-free detector` | 实时目标检测，无锚框检测 |
| 航拍目标检测 | `aerial object detection`, `UAV imagery detection`, `drone-based detection` | 航拍目标检测，无人机图像检测 |
| 小目标检测 | `small object detection`, `feature pyramid network`, `multi-scale detection` | 小目标检测，特征金字塔 |
| 车辆检测 | `vehicle detection in remote sensing`, `ground vehicle detection` | 地面车辆检测 |

### 1.2 多目标跟踪（MOT）

| 主题 | 英文关键词 |
|---|---|
| 多目标跟踪 | `multi-object tracking`, `MOT`, `tracking-by-detection` |
| 深度外观模型 | `deep appearance model`, `Re-ID`, `appearance feature` |
| 关联算法 | `Hungarian algorithm`, `data association`, `gated nearest neighbor` |
| 航拍跟踪 | `UAV tracking`, `aerial multi-object tracking` |

### 1.3 不确定性下的目标识别（核心——诱饵鉴别）

| 主题 | 英文关键词 |
|---|---|
| 运动学一致性检验 | `kinematic consistency check`, `trajectory validation`, `motion pattern analysis` |
| 虚假目标鉴别 | `decoy discrimination`, `false target rejection`, `target validation` |
| 运动模式分类 | `motion pattern classification`, `velocity-based classification`, `movement analysis` |
| 目标可信度评估 | `target credibility assessment`, `belief function`, `Dempster-Shafer theory` |

## 二、状态估计与滤波（Estimation）

### 2.1 基础滤波

| 主题 | 英文关键词 |
|---|---|
| 卡尔曼滤波 | `Kalman filter`, `extended Kalman filter (EKF)`, `unscented Kalman filter (UKF)` |
| 粒子滤波 | `particle filter`, `sequential Monte Carlo`, `bootstrap filter` |
| 航迹平滑 | `track smoothing`, `trajectory estimation`, `state estimation` |
| 非线性滤波 | `nonlinear filtering`, `bearing-only tracking`, `angle-only tracking` |

### 2.2 仅角度/仅方位跟踪（Bearing-Only Tracking — 核心难点）

| 主题 | 英文关键词 |
|---|---|
| 仅方位跟踪 | `bearing-only tracking`, `angle-only tracking`, `BOTT` |
| 被动定位 | `passive localization`, `passive target motion analysis (TMA)` |
| 可观测性分析 | `observability analysis`, `observability bearing-only` |
| 多传感器融合 | `multi-sensor fusion`, `sensor management` |

> **背景**：比赛中 UAV 只有相机（无雷达/测距仪），观测本质是 bearing-only（方位角+俯仰角），没有直接距离信息。这是核心难点。

### 2.3 目标运动建模

| 主题 | 英文关键词 |
|---|---|
| 匀速运动模型 | `constant velocity (CV) model`, `nearly constant velocity` |
| 匀加速模型 | `constant acceleration (CA) model` |
| 转弯模型 | `coordinated turn (CT) model`, `turning rate estimation` |
| 交互多模型 | `interacting multiple model (IMM)`, `model switching` |

## 三、搜索与覆盖规划（Search）

### 3.1 区域覆盖搜索

| 主题 | 英文关键词 |
|---|---|
| 覆盖路径规划 | `coverage path planning (CPP)`, `lawnmower pattern` |
| 基于网格的搜索 | `grid-based search`, `cell decomposition` |
| 信息增益搜索 | `information gain search`, `information-theoretic search` |
| 概率图搜索 | `probability map search`, `occupancy grid`, `Bayesian search` |

### 3.2 多 UAV 协同搜索

| 主题 | 英文关键词 |
|---|---|
| 协同搜索 | `cooperative search`, `multi-UAV search`, `distributed search` |
| 区域划分 | `area partitioning`, `Voronoi partition`, `balanced partition` |
| 搜索效率优化 | `search efficiency optimization`, `coverage maximization` |
| 不确定性下的搜索 | `search under uncertainty`, `probabilistic search` |

## 四、路径规划与避障（Path Planning）

### 4.1 基础路径规划

| 主题 | 英文关键词 |
|---|---|
| A* 算法 | `A* algorithm`, `heuristic search`, `Dijkstra shortest path` |
| RRT 系列 | `RRT`, `RRT*`, `rapidly-exploring random tree`, `informed RRT*` |
| 势场法 | `potential field method`, `artificial potential field` |
| 路径平滑 | `path smoothing`, `B-spline`, `minimum snap trajectory` |

### 4.2 威胁规避

| 主题 | 英文关键词 |
|---|---|
| 禁飞区规避 | `no-fly zone avoidance`, `keep-out zone`, `constraint-based planning` |
| 威胁规避路径 | `threat avoidance path planning`, `survivability routing` |
| 动态障碍规避 | `dynamic obstacle avoidance`, `real-time replanning` |
| UAV 路径规划 | `UAV path planning`, `drone route planning`, `mission planning` |

## 五、任务分配与协同（Coordination）

### 5.1 多 UAV 任务分配

| 主题 | 英文关键词 |
|---|---|
| 任务分配 | `task allocation`, `target assignment`, `multi-agent task allocation` |
| 拍卖算法 | `auction algorithm`, `consensus-based bundle algorithm (CBBA)`, `sequential auction` |
| 匈牙利算法 | `Hungarian algorithm`, `assignment problem`, `bipartite matching` |
| 分布式优化 | `distributed optimization`, `decentralized coordination` |

### 5.2 分布式协同

| 主题 | 英文关键词 |
|---|---|
| 一致性算法 | `consensus algorithm`, `average consensus`, `distributed consensus` |
| 编队控制 | `formation control`, `flocking`, `virtual structure` |
| 分布式决策 | `distributed decision making`, `multi-agent decision`, `game theory` |
| 通信约束下协同 | `communication-constrained coordination`, `limited communication` |

## 六、推荐综述论文检索

### Google Scholar 检索字符串

```
# UAV ISR 任务综述
"unmanned aerial vehicle" AND "intelligence surveillance reconnaissance" AND survey

# 多 UAV 协同搜索
"multi-UAV" AND "cooperative search" AND "target detection" AND survey

# 仅方位跟踪（核心难点）
"bearing-only tracking" AND "UAV" AND ("Kalman" OR "particle filter")

# 诱饵/虚假目标鉴别
"false target" AND "discrimination" AND ("kinematic" OR "motion analysis")

# 分布式任务分配（最新综述）
"consensus-based bundle algorithm" OR "CBBA" AND "multi-UAV" AND survey

# 航拍目标检测（最新综述）
"aerial object detection" AND "deep learning" AND survey AND 2023..2026
```

### 推荐期刊/会议

- IEEE Transactions on Aerospace and Electronic Systems (TAES)
- IEEE Transactions on Robotics (T-RO)
- Journal of Field Robotics (JFR)
- Autonomous Robots
- IEEE International Conference on Robotics and Automation (ICRA)
- IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)
- AIAA Journal of Guidance, Control, and Dynamics

# 文献提取：Anti-Jamming Attack Mixed Strategy for Formation Tracking Control via Game-Theoretical Reinforcement Learning

- **作者**：Lei Xue et al.
- **年份**：2025
- **来源**：IEEE Transactions on Intelligent Vehicles
- **原文路径**：`C:\Users\ouyangjiahong\Downloads\hf比赛\output\Xue 等 - 2025 - Anti-Jamming Attack Mixed Strategy for Formation Tracking Control via Game-Theoretical Reinforcement\hybrid_auto\Xue 等 - 2025 - Anti-Jamming Attack Mixed Strategy for Formation Tracking Control via Game-Theoretical Reinforcement.md`

## 一句话概括

针对多无人机编队跟踪中遭遇智能干扰攻击的场景，提出了一种基于三层 Stackelberg 博弈的混合策略（机动策略+通信频率切换策略），并设计了三层 Actor-Critic（Tri-AC）强化学习算法来求解 Stackelberg-Nash 均衡。

## 核心方法/模型

### 系统设定

系统中有三类无人机：领导者（Leader）、跟随者（Follower）、干扰者（Jammer），集合表示为 P = {L, N, J}。领导者和跟随者采用**混合策略**（机动策略 v + 通信频率切换策略 p），干扰者仅采用通信频率策略。

### 博弈模型：三层 Stackelberg 博弈

将三类无人机的交互建模为三层 Stackelberg 博弈 G = (P, A, U)：

- **上层**：领导者先行动，宣布策略
- **中层**：跟随者观察领导者策略后同时做出最优响应（合作）
- **下层**：干扰者根据领导者和跟随者的策略选择最优攻击策略（非合作）

**效用函数设计**：
- 领导者/跟随者效用：U_i = -e_i - A_i - G_i（最小化跟踪误差 + 攻击损失 + 频率冲突损失）
- 干扰者效用：U_J = A_L + sum(A_i)（最大化对所有无人机的攻击损失）

**攻击损失模型**：基于自由空间路径损耗，A = C_l / d^2，其中 C_l 为信道功率增益常数，d 为干扰者与被干扰无人机的距离。干扰仅在频率匹配时生效（p_J = p_i 时攻击有效）。

**目标函数（三层优化问题）**：
- 领导者 min f_i，跟随者 argmin f_j，干扰者 argmax f_k

论文证明了 Stackelberg-Nash 均衡的存在性（Theorem 1，详见 Appendix A）。

### 求解算法：Tri-AC（Tri-Level Actor-Critic）

扩展 Bi-AC [17] 到三层，采用**集中训练-分散执行**（CTDE）架构：

- **训练阶段**：2n+3 个模型集中训练（每层每智能体一个 actor + 一个 critic）
- **执行阶段**：训练好的模型分配给具体智能体分散使用

**网络结构**：使用 ReLU 激活的单隐层神经网络，f(s,a;W,b) = (1/sqrt(m)) * sum(b_r * ReLU((s,a)^T W_r))，其中 b_r 固定为 Uniform({-1,1}) 随机特征，仅更新 W。

**更新规则**：
- Critic 更新：TD 学习，theta <- theta + alpha * delta * nabla_theta Q_hat（Theorem 2 证明收敛性）
- Actor 更新：策略梯度，phi <- phi + beta * nabla_phi log(pi_phi) * Q_hat（Theorem 3 证明收敛性）
- 从上到下依次确定动作，下层智能体观察上层动作后做出响应

**收敛性**：论文给出了 Critic 收敛（Theorem 2）和 Actor 收敛（Theorem 3）的完整理论证明，误差上界为 O(B^{3/2} * m^{-1/4})。

## 关键参数/实验数据

### 实验设置
- 场景：1L-1F-1J、1L-2F-1J、1L-4F-1J
- 训练轮次：100-500 episodes，每轮步数约 437-16858 steps
- 频率选择：离散频率点集合

### 混合策略 vs 固定策略对比（10 组实验）

**固定策略**（领导/跟随者仅机动不切换频率）：
- 领导者被攻击次数：37-2004 次
- 跟随者被攻击次数：9-6870 次
- 干扰者被攻击次数：22-412 次

**混合策略**（领导/跟随者同时机动+切换频率）：
- 领导者被攻击次数：26-1005 次（显著下降）
- 跟随者被攻击次数：8-765 次（显著下降）
- 干扰者被攻击次数：47-246 次（显著上升）

### 关键对比结论
- 混合策略下，干扰者对领导者的攻击率 r_l 显著降低
- 混合策略下，干扰者对跟随者的攻击率 r_f 显著降低
- 混合策略下，无人机对干扰者的攻击率 r_j 显著升高
- 随训练轮次增加，奖励曲线收敛，验证了 Tri-AC 达到 Stackelberg-Nash 均衡

### 频率选择演化规律（1L-2F-1J）
- 早期：领导/跟随者频率冲突多，干扰者攻击概率低
- 中期：干扰者智能提升，对领导者的攻击概率上升
- 后期：无人机学会避免领导被攻击，干扰者转而攻击跟随者

## 结论

论文设计了一种面向多无人机编队跟踪的抗干扰混合策略，通过三层 Stackelberg 博弈建模，利用 Tri-AC 强化学习算法求解均衡。相比固定策略，混合策略显著降低了领导/跟随者被干扰的概率，提升了干扰者被反击的概率。增加跟随者数量后算法仍能收敛。

**局限性**：
- 仿真实验环境相对简化，未考虑实际通信损耗（频率差带来的损失未建模）
- 干扰者仅采用单一通信策略，若干扰者也采用混合策略则更复杂
- 未在实际机器人系统上验证
- 未来工作：实际机器人系统部署、异构系统博弈控制

## 对本项目的可用点

### 直接可用
1. **对抗场景建模框架**：红枫竞赛中的对抗场景可借鉴其三层 Stackelberg 博弈建模思路——将我方无人机视为 Leader/Follower，将对手视为 Jammer，构建层次化决策模型
2. **混合策略思想**：机动策略+通信策略的混合设计，对竞赛中需要同时考虑运动规划和通信/感知资源分配的场景有参考价值
3. **攻击损失模型**：A = C_l / d^2 的距离衰减模型可用于评估干扰或对抗条件下的通信质量退化

### 参数参考
- 信道功率增益常数 C_l 的建模方式
- 频率冲突损失常数 C_gi 的设计思路
- 采样时间 T 与速度 v 的关系 x(k+1) = x(k) + T * v(k)

### 限制条件
- 论文假设干扰仅在频率完全匹配时生效，实际竞赛中对抗场景可能更复杂
- Tri-AC 算法需要大量训练轮次（数百 episodes），实时性可能不足
- 论文的 2D 简化模型与实际 3D 无人机运动有差距
- 需要已知干扰者位置信息，实际场景中可能需要配合估计模块

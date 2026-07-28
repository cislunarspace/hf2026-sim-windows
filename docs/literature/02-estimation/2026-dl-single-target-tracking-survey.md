# 文献提取：基于深度学习的无人机单目标跟踪综述

- **作者**：陈泷 et al.
- **年份**：2025
- **来源**：航空学报（中图分类号 TP391.4，文献标志码 A）
- **原文路径**：`output/2026 - 基于深度学习的无人机单目标跟踪综述/hybrid_auto/2026 - 基于深度学习的无人机单目标跟踪综述.md`

## 一句话概括
对2022-2025年基于深度学习的无人机单目标跟踪方法进行系统综述，将其归纳为传统Siamese网络、CNN-Transformer混合架构、全Transformer三大技术路线，揭示了从追求精度向精度-效率协同优化的演进趋势。

## 核心方法/模型

### 任务形式化

无人机单目标跟踪任务定义为：

$$
f: (z, x_t) \longrightarrow (\hat{\mathcal{B}}_t, s_t)
$$

其中 z 为模板，x_t 为搜索区域，输出为预测边界框和跟踪置信度。

### 三大技术路线

**1. 传统Siamese网络（第2节）**

参数共享双分支CNN结构，通过交叉相关操作进行模板匹配。沿四个方向演进：
- 注意力机制深化（SiamAPN++ -> SiamRAKPN）
- 轻量化设计（LightTrack用NAS自动搜索、P-SiamFC++用Fisher/秩剪枝、LightFC用重参数化）
- 专用场景优化（ABDNet处理运动模糊、SmallTrack用小波变换处理小目标）
- 时序信息利用（MT-Track多步时序建模）

根本局限：交叉相关的局部线性匹配机制无法建模复杂非线性变化；逐帧独立处理导致时序利用不充分。

**2. CNN-Transformer混合架构（第3节）**

创新性提出三分类体系：

- **模块替代**：用Transformer替代CNN中的特定组件（如TransT用注意力替代相关操作、HCAT用分层交叉注意力）
- **特征后融合**：CNN独立提取特征，Transformer后期融合（如TCTrack设计时间自适应卷积、HFPT引入池化注意力处理小目标）
- **协同建模**：CNN和Transformer深度耦合、双向信息交互（如HiFT分层特征Transformer、STHFT时空分层特征Transformer、PRL-Track渐进式学习）

**3. 全Transformer方法（第4节）**

分为多流和单流两种，单流为主流趋势。进一步按计算策略细分：

- **静态计算**：固定计算路径（OSTrack、SimTrack、MixFormer、SeqTrack将跟踪重新定义为序列生成任务）
- **混合机制**：部分组件引入动态调整（ARTrackV2联合轨迹-外观自回归、AVTrack激活模块按需计算，beta阈值在(0.5,1.0)间）
- **动态计算**：根据输入复杂度智能调整（DDCTrack动态token采样、SGLATrack相似度引导层自适应禁用冗余层、Aba-ViTrack背景感知token丢弃）

### 关键评估指标

- 重叠精度 IoU = |A_pred intersect A_gt| / |A_pred union A_gt|
- 中心位置精度 CLP = 欧氏距离
- 成功率曲线AUC为综合排名指标
- 评估协议：OPE（一次通过评估）

## 关键参数/实验数据

### 数据集规模
| 数据集 | 年份 | 序列数 | 总帧数 | 分辨率 | 模态 |
|--------|------|--------|--------|--------|------|
| UAV123 | 2016 | 123 | 11.3万 | 1280x720 | RGB |
| UAV20L | 2016 | 20 | 5.9万 | 1280x720 | RGB |
| DTB70 | 2017 | 70 | 1.6万 | 1280x720 | RGB |
| UAVDT | 2018 | 50 | 3.7万 | 1080x540 | RGB |
| VisDrone | 2021 | 167 | 26.5万 | - | RGB |
| VTUAV | 2022 | 500 | 170万 | 1920x1080 | RGB-T |
| WebUAV-3M | 2022 | 4500 | 330万 | - | RGB |
| MUST | 2025 | 250 | 14.3万 | 1200x900 | 多光谱(8波段) |

### 代表性算法性能（AUC %）
**UAV123数据集：**
- 传统Siamese最佳：SiamRAKPN 66.4%，LightFC 65.5%
- CNN-Transformer混合：HiFT 58.9%（表6数据质量较差，仅HiFT可见）
- 全Transformer最佳：SimTrack 69.2%，CLTrack 68.9%，SeqTrack 68.6%，CGTrack 67.2%

**DTB70数据集：**
- 传统Siamese最佳：MT-Track 66.3%
- 全Transformer最佳：CLTrack 68.2%

**VisDrone2018数据集：**
- 全Transformer最佳：CLTrack 67.1%，TATrack 66.9%

### 速度指标
- HCAT：GPU 195 FPS，边缘设备 55 FPS
- P-SiamFC++(v2)：CPU 76.4 FPS，参数从7.49M压缩至3.05M
- HiT：边缘设备 61 FPS
- SiamLT：UAV123/UAV20L上 124 FPS
- 李华耀等GhostNet方法：87 FPS
- CTIFTrack：71.98 FPS
- SeqTrack比MixFormer快1.4倍
- AVTrack-MD比AVTrack快17%

### 模型压缩参数
- P-SiamFC++(v2)：参数 3.05M，CPU 76.4 FPS
- LightTrack(NAS)：三个版本分别针对不同计算约束

## 结论

### 主要结论
1. **技术演进趋势明确**：从传统Siamese（效率优先）-> CNN-Transformer混合（平衡策略）-> 全Transformer（精度优先，正在向精度-效率协同优化转变）
2. **全Transformer在精度上显著领先**：在所有基准数据集上性能最优，CLTrack在DTB70达到68.2% AUC
3. **传统Siamese在资源受限场景仍有竞争力**：P-SiamFC++(v2)在CPU上达76.4 FPS
4. **混合架构是当前实用部署的平衡选择**：FLOPs相对较低，适合精度和效率都有要求的场景

### 五大核心挑战
1. Transformer二次计算复杂度与实时性矛盾
2. 背景token稀释导致目标特征退化（小目标场景尤为严重）
3. 多模态异构传感器融合的语义对齐困难
4. 极端视角变化下的视角不变性学习缺失
5. 长序列时空依赖建模中的记忆容量瓶颈

### 未来发展方向
1. 高效注意力机制与自适应计算（线性复杂度注意力、层自适应、NAS）
2. 目标感知特征增强与背景抑制（互信息最大化、token丢弃策略）
3. 统一多模态表示学习与自适应融合
4. 几何约束的视角鲁棒学习
5. 可扩展的长序列记忆架构与在线学习

### 工程部署关键点
- 主流硬件平台：ARM处理器（轻量级方案）和NVIDIA Jetson系列（GPU加速）
- 轻量化技术：模型剪枝、量化、知识蒸馏
- 边缘-云协同架构：机载轻量化处理 + 云端高精度分析

## 对本项目的可用点

本论文为红枫2026竞赛的"估计"（Estimation）模块提供了直接的技术参考：

1. **跟踪器选型依据**：竞赛需要在嵌入式无人机平台上实时跟踪目标，论文的速度-精度对比数据可直接用于选型。推荐优先考虑CNN-Transformer混合方法（如TCTrack/TCTrack++，专门针对无人机时序建模）或动态计算类单流方法（如Aba-ViTrack，首次将高效ViT用于实时无人机跟踪）

2. **时序建模参考**：竞赛涉及轨迹估计，MT-Track的多步时序建模公式（T_t = T_0 + beta(alpha_t * T_{t-1})）和TCTrack的时间自适应卷积权重调整（W_t = W_b * alpha_t^w）可直接借鉴用于目标状态预测和轨迹平滑

3. **轻量化部署参数**：P-SiamFC++(v2)提供3.05M参数量、CPU 76.4 FPS的参考基准；HCAT在边缘设备55 FPS的实测数据可作为嵌入式部署的可行性验证

4. **小目标处理技术**：SmallTrack的小波变换特征保留和图学习增强、HFPT的特征校正层，适用于竞赛中可能出现的远距离小目标场景

5. **运动模糊应对**：ABDNet的对抗性模糊-去模糊框架（去模糊器、模糊生成器、跟踪器三组件）可用于处理无人机高速运动时的图像退化

6. **评估体系**：论文梳理的IoU、CLP、AUC评估指标和OPE评估协议可直接用于竞赛算法的定量评估

7. **限制条件**：全Transformer方法精度最高但计算量大（自注意力二次复杂度），在嵌入式平台上需要配合token采样或层剪枝策略；传统Siamese方法精度有天花板（UAV123上最高66.4% vs 全Transformer 69.2%）

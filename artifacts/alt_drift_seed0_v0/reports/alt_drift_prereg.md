# Pre-Registration: alt_drift Channel-Transfer Confirmation

状态：CONFIRMATORY。本实验确证一个由 xy_velocity seed 0/1/2 探索性数据形成的重构假设——
"控制分配预测活跃通道 → channel-directed search 直接、且每触发更低成本地合成通道纯、回中触发"
是否为可迁移的方法，而非 roll/pitch 单属性观察。下列所有阈值在看到任何 alt_drift 数据前固定。

## 0. 冻结的实验地基（与 xy_velocity 实验完全一致，不改）

- 参数化：D=40，10 窗 × 0.5s × 4 通道 {roll,pitch,yaw,throttle}，5s 时域 + 8s 回中尾段。
- 可行集：|u_t|≤L（stick-limit=1.0，与先前 probe 口径一致，公开披露非调参）、|u_t−u_{t-1}|≤R。
- 属性：post_neutral_alt_drift，ρ = 1.0m − max|高度漂移|，回中后测量；ρ>0 安全，ρ<0 违例。
- 噪声纪律：每个 ρ 跑 J=5，鲁棒违例 = ρ_mean+2σ<0，鲁棒安全 = ρ_mean−2σ>0。
- 分类阈值：INTERIOR_MAX_ABS=0.5，SATURATED_MIN_ABS=0.9，support 阈值 |θ|>0.1。
- 方向探针步长 δ=0.2（抗噪，与 H1/先前 probe 一致）。
- 每臂 80 个 J=5 点，匹配预算。
- 场景：px4_position（POSCTL）。throttle 通道在参数化中以 0 为中位（=悬停保持），回中=throttle 回中位。

## 1. 控制分配推导（出结果前承诺的预测）

多旋翼机体轴层级，竖直推力↔throttle（collective）是直接分配；roll/pitch 倾转对高度只有
二阶耦合（倾转→竖直推力分量 cosθ 损失，且 POSCTL 会补偿）。因此：

- **承诺预测 A_alt = {throttle}（主通道）。**
- 预期 roll/pitch 仅为二阶次要敏感度。

这是一个由系统推导、可被探针证伪的预测，不是事后拟合。derive_A_phi(property) 必须把该推导
实现为可复用判据（xy_velocity→{roll,pitch}，alt_drift→{throttle}），而非写死常量。

## 2. 预注册假设与事前阈值

### H-alt-1（判据迁移，主假设）

δ=0.2 边界法向探针下，throttle 为敏感度质量最大的通道，且 throttle 质量占比 ≥ 0.50。

- 确证：throttle 为最大通道 且 占比 ≥ 0.50。
- 收窄：throttle 最大但占比 ∈ [0.30,0.50)；或 {throttle + 较大的一个倾转通道} 合计 ≥ 0.70 且 throttle 最大
  → 部分迁移，按此把 A_alt 重定义为推导出的多通道集，继续后续，并标注"部分迁移"。
- 证伪：throttle 不是最大通道（如 roll/pitch 占主导）→ 控制分配预测对该属性不成立，判据不迁移，
  作为负面结果如实报告，停止 Phase 2。

（0.50 的设定相对 xy_velocity 双通道 86% 的先例是保守的单通道占主导门槛。）

### H-alt-2（直接合成 + 薄目标）

- Arm A（均匀随机）：alt_drift 内部鲁棒违例 = 0。
- Arm B（通道无关内部包夹）：到达内部仅为稠密多通道输入，通道纯（⊆A_alt）占比 ≤ 0.10。
- Arm C（通道导向，仅在 A_alt 上有支撑）：内部鲁棒违例 > 0，且通道纯占比 ≥ 0.90。
- 确证：以上三条同时成立。

### H-alt-3（每 distinct 机动成本；不主张数量占优）

distinct 机动签名 = (active channel set, 活跃窗带 [min..max], 各通道符号)，幅值无关。
端到端成本 = 该方法总 J=5 点 ÷ distinct 通道纯机动数；ddmin 端到端含其起点（Arm B 80 点）。

- 确证：Arm C 每 distinct 机动成本 < ddmin 端到端每 distinct 机动成本（报告倍数与跨 seed 衰减）。
- 明确不预注册"Arm C distinct 数 ≥ ddmin"——该主张在 xy_velocity 已被证伪，本实验不主张。

## 3. Arm C 搜索设计（相对 xy_velocity 的改进）

Arm C 用**系统 sweep 时长**替代随机采时长：d ∈ {1..10}、起始窗扫过、每条包络做幅值二分定位
最温和违例幅值；每个评估点记录其 distinct 签名。目的：让 distinct 计数/成本不随 seed 抖，
并直接产出 时长-幅值 frontier 作为刻画。

## 4. 分阶段（避免在判据不迁移时浪费算力）

- Phase 0（廉价 sanity）：确认 alt_drift 在可行 throttle 输入下**可被违例**（饱和 throttle 探针）。
  若不可违例 → 该属性/场景需换设定（如 ALTCTL），作为前置发现报告，暂停。
- Phase 1（seed 0）：δ=0.2 探针（判 H-alt-1）+ 三臂（判 H-alt-2）。据 §2 阈值做 go/no-go。
  证伪→停并报负面；收窄→按规则改 A_alt 继续；确证→进 Phase 2。
- Phase 2（seed 1、2 + ddmin 基线，仅在 Phase 1 确证后）：三臂跨 seed + ddmin 基线；
  算每 distinct 机动成本与通道纯占比（判 H-alt-3）。

## 5. 纪律声明

- 上述阈值在任何 alt_drift 数据前固定，记录于本文件；运行中不得调整。
- 非零 seed 经显式 --allow-nonzero-seed 解冻，其余预注册常量与 seed-0 完全一致。
- 报告每处结论标注：confirmatory（针对重构假设）、PX4、属性 alt_drift；跨平台仍需 ArduPilot。

# OverDraw —— 研究叙事、论文骨架与剩余实验计划

> 一份自洽的项目蓝本。涵盖：当前叙事（含重心转移的来龙去脉）、已完成的经验证据账本、能扛审稿的论文骨架、剩余必需/可选实验的规格、写作顺序、以及贯穿全程必须保持的纪律。
>
> 仓库 `github.com/JacksonW1025/uav_sparse` · 工作区 `/mnt/nvme/px4_work/uav_sparse` · 分支 `origin/explore/bbs-probes`

---

## 0. 现状速览（TL;DR）

- **发现阶段已结束。** 六个探针把一个"契约干净的不安全区"（OverDraw）在 ArduCopter 控制权限预算上钉死了：飞控照规约把每条限制/failsafe 正确执行（命令 ≤ ANGLE_MAX、零 failsafe 误触发），飞机却仍因合法输入进入外部不安全。
- **核心强结果**：49 个硬后果 run 全部 `clean_unsafe`、`bug_side=0`；跨 540 格的湍流 × 姿态包络网格里契约/failsafe/clamp 触发**全为零**；不安全区由两种机制构成、2D 边界可学（AUC 0.912）；可被现实侧风持续 bank 机动够到；**收紧 ANGLE_MAX 也关不掉**。
- **叙事重心已从"BBS 算法 + 标度律外推"转到"问题形式化 + dual-oracle 检测方法 + 演示"**。这不是又一次拍脑袋换故事——是数据把那个真实的贡献逼了出来（见 §1.3）。
- **下一步是 build + write，不是再探**。剩余必需实验三项：第二块表（能量逆风版，做对版）、PX4 第二栈、契约测试基线对跑（§4）。
- **目标会议**：ICSE / FSE 为主（问题 + 方法学 + 演示）。

---

## 1. 叙事（The Narrative）

### 1.1 核心问题与定义

**OverDraw / "透支"**：一类飞控规约缺口。在一个**固定的合法配置 P** 下、在**操作者输入 M** 空间里（以**环境 E** 为条件），存在合法输入使系统进入**外部不安全**状态，而**每条 failsafe/限制都按其配置阈值正确触发、没有任何契约被违反**。统一缺口类型是**"阈值不足"**：一个对名义条件够、对合法工况不够的配置阈值或安全包络。

**契约干净不安全（contract-clean unsafe）**：一个状态同时满足
- **Oracle A（世界 oracle）= 不安全**：真实后果（撞地/接触、非指令掉高超阈、未恢复的姿态发散），**不是构造的分析线**；
- **Oracle B（契约 oracle）= 干净**：命令在准入范围内（≤ ANGLE_MAX）、无预防性 failsafe 触发、无策略不变量违反。

OverDraw 区 = `A ∧ ¬B`。

### 1.2 概念框架：FuSA vs SOTIF

- **功能安全（ISO 26262 / FuSA）** 管"实现对不对"：组件是否按规约工作。契约测试（PGFUZZ、RVFuzzer 族）查的是这条轴——**规则有没有被违反**。
- **预期功能安全（ISO 21448 / SOTIF）** 管"规约够不够"：在名义实现完全正确的前提下，规约本身在合法工况下是否充分。
- **OverDraw 是把 SOTIF 视角在飞控 failsafe 阈值/安全包络上操作化**：用 dual oracle 把"照规矩却不安全"从"违反规矩"里干净切出来。这正是契约测试结构性看不见的那一层。

### 1.3 重心的转移（重要：这不是换叙事，是证据选择）

> 这一节专门记录"为什么现在是这个叙事"，因为它直接回应"是不是又在换故事"的疑虑。

**最初的赌注**是算法侧：一个 **Budget-Boundary Search (BBS)** 算法，配两个目标命题——命题 1（边界搜索 `O(log)` 效率）、命题 2（沿"margin"抽象的标度律**外推**）。卖点是"方法不是工程"，押在"需求必须学、算不出"的预算上。

**数据逐步否决了这个赌注**，信号一致且清楚：
1. `stage0 v2`：权限余量信号在 r 上**非单调**。
2. `oracleA v1`：硬后果下 `P(unsafe)` 在 r 上**非单调**（W 形：r=180 峰、r=360 谷、r=540 峰）。
3. `demand v1`：**没有任何标量需求量 Φ** 能把结果单调排序（最优 `actual_rate_peak` AUC 仅 0.797）。

**这是一个从一开始就埋着的结构性死结**：
- 边界**干净**的预算 ⊂ **可算**预算（能量/时间——对"学习"是平凡的）；
- **必须学**的预算 ⊂ 边界**凌乱**（控制权限——非单调、多机制）。

所以"BBS 漂亮地学/外推一个必须学预算的边界"在结构上从第一天起就难，数据只是确认了它。**命题 2（标度律外推 + O(log) 保证）在必须学预算上没有数据支撑，正式放弃。**

**与此同时，另一个候选贡献每一轮都在变强**：契约干净不安全区**存在**、且对契约测试**不可见**。于是新的重心是：

> **不是** "BBS 算法 + 外推保证"（数据不支持）；
> **是** "OverDraw：一类对契约测试结构性不可见的契约干净不安全区 + dual-oracle 检测方法 + 在真实飞控上证明安全包络（ANGLE_MAX）在合法输入下不充分 + 严格的 artifact 排除"。

`stage0/island/oracleA/demand/ep` 五轮共同给出的，是一个**问题形式化 + 检测方法学 + 严格演示**的论文，外加一个**诚实、modest 的算法次贡献**（2D 可学边界，AUC 0.912）。核心卖点几乎白送：**契约测试在整片区域 `B=0`——我们看见的，它结构性看不见。**

### 1.4 与现有工作的定位

| 工作 | 检测的轴 | 形态 | 与 OverDraw 的切分 |
|---|---|---|---|
| PGFUZZ / RVFuzzer 族 | 规则/策略**违反**（FuSA） | 契约 fuzzing | 结构性看不见 `A∧¬B`；本工作正是它们的盲区 |
| RouthSearch | 参数诱导失稳 | **闭式**判据、**参数**空间 | 闭式 vs 观测标定；参数空间 vs 输入空间；贴 bug 一侧 vs 资源耗尽 |
| LGDFUZZER | range 规约 bug | 学习引导搜索 | 同属契约一侧；OverDraw 在契约干净一侧 |
| SOTIF/ADS 测试（STEAM、MoSAFE 等） | 规约充分性（SOTIF） | 自动驾驶域 | 框架同源，但 OverDraw 把它在 RV failsafe 阈值/姿态包络上用 dual oracle + 真实后果操作化 |

**结构性新意**：把 SOTIF 的"规约不充分"在飞控上落成一个**可检测、契约测试看不见**的不安全区类型，并给出 dual-oracle 检测方法 + 在真实栈上的演示。

---

## 2. 经验证据账本（已完成）

> 六个探针的链条本身是可信度论证：每一步都排掉一类"其实是假信号"的解释。

### 2.1 探针时间线

| 探针 / tag | 裁决 | 关键结果 | 抓到/排掉的 artifact |
|---|---|---|---|
| 能量 probe（早期，含于分析） | 退化沙盒 | `margin ≈ 122 − D` 恒等式；`BATT_LOW_MAH` 在 ~219 mAh、**悬停**触发（地速 0.02），与 D 无关 | 证实能量"太可算"，不能当算法主场 |
| 控制权限 v1（早期，含于分析） | 未回答 | 命令 slew 150→4800，实际 RCIN p95 仅 60→1921（**衰减到 ~40%**）；ALT_HOLD 锁死后果 | **artifact ① 输入保真**：RC 滤波削猛度 |
| `ctrlauth-stage0-v2-20260615` | GO | 通路保真过线（180→0.998、300→1.006）；P3 `overdraw=8/bug_side=25/safe=1`；权限可压穿（P2 max err 223.72°）；但呈"岛"、B 非单调；P4 margin 非单调 | 修复 ①；改用 GUIDED_NOGPS SET_ATTITUDE_TARGET 通路 |
| `ctrlauth-island-v1-20260615` | ISLAND-FLAKY | **25/25 `bug_side` 实为 harness `GCS_COMMAND LAND`**（`MODE.Rsn=2`），非飞控；8 个 `overdraw` 真干净（B=0）；修正后 `p_overdraw=1.00` 全格 → "岛"是 harness 伪影，区域其实**全干净** | **artifact ② harness 污染**：清理动作被计成 failsafe |
| `ctrlauth-oracleA-v1-20260615` | **REAL-GAP** | 65 run，**49 hard-A 全 `clean_unsafe`、`bug_side=0`**（44 撞地 + 4 掉高>15m + 1 发散）；`B_preventive=0`；命令峰 44° < 45°（留 1° margin）；结果在 r 上 W 形非单调 | **artifact ③ 软后果**：oracle A 锚到真实后果（撞地/掉高/不恢复发散），排除"瞬态越界后恢复" |
| `ctrlauth-demand-v1-20260615` | COMPLEX-BOUNDARY | 无标量 Φ 达单调阈；最优 `actual_rate_peak` AUC 0.797（全 oracle 窗会到 0.957 但混入撞地后反弹角速率 → 后果污染，修正后 0.797）；两机制签名 | **artifact ④ 后果泄漏**：需求特征必须 pre-consequence |
| `ctrlauth-ep-v1-20260615` | 完成 | 机制 `altitude_bleed=21 / tumble=11 / both=17`；**2D AUC 0.912**（`actual_rate_peak × high_bank_dwell_impulse_gt_80deg`）；**E/P 网格 540/540、B=0 全格、契约/failsafe/clamp 全 0**；`r50=120` 在所有 E/P 格饱和、前移单调但面积非严格单调；现实侧风持续 bank witness | 跨条件确认契约干净性；2D 边界可学；现实可达 |

### 2.2 已被牢固确立的事实

- **存在**：契约干净不安全区在控制权限上存在（49 hard-A、B=0），扛过四轮 artifact。
- **契约不可见**：跨整个 E/P 网格（540 run）契约/failsafe/clamp = 0。
- **不可调修**：`r50` 跨所有 ANGLE_MAX 饱和——收紧包络关不掉 gap。
- **有结构**：两种机制（速率驱动 LOC + 持续 bank 掉高），2D 可学（AUC 0.912）。
- **现实可达**：侧风 + 持续 bank 的现实输入可达。

### 2.3 必须诚实标注的边界（写作时别越界）

- **跨条件预测弱**：`r50` 被钉在地板、面积非单调 → **不要**把它当"边界随条件可预测移动"卖；强读法是"gap 对包络收紧鲁棒、调参修不掉"。
- **算法贡献 modest**：2D 经验边界（AUC 0.912），**不是**解析标度律，**没有** O(log) 保证。
- **单栈、单必须学预算**：目前只有 ArduCopter + 控制权限 → **广度是 CCF-A 的主要缺口**（见 §4）。
- **样本效率未测**：没跑搜索基线，**不得**声称"比基线快 X 倍"（除非补 §4.4）。

---

## 3. 论文骨架（Paper Skeleton）

### 3.1 定位与目标会议

- **定位**：OverDraw —— 发现一类规约/契约测试结构性看不见的契约干净不安全区。
- **主会 ICSE / FSE**：问题形式化 + 方法学 + 演示，契合其口味。
- **不投 ISSTA（暂）**：它会要"算法保证"，而保证恰是数据不给的；除非补足 §4.4 把算法子贡献做硬。

### 3.2 能扛审稿的 claim（措辞已校准到数据）

- **C1 存在性**：存在"契约干净不安全"状态——飞机进入外部不安全（撞地 / >15 m 非指令掉高 / 未恢复发散），而命令全程 ≤ ANGLE_MAX、无任何 failsafe/限幅介入。
- **C2 契约不可见**：这类状态对契约测试（PGFUZZ/RVFuzzer 族，只检规则违反）**结构性不可见**——跨湍流（0/低/高）× 姿态包络（30°/45°/上限）的 540 格，契约/failsafe/clamp 事件为零。
- **C3 包络不充分且不可调修**：文档化的姿态安全包络 ANGLE_MAX 不充分——留在界内不保证安全；且**收紧它也关不掉 gap**（最保守 P 下 clean-unsafe 区仍在）。故这是**规约缺口，非调参错误**。
- **C4 检测方法学 + 标签可信**：dual oracle（世界 oracle A + 契约 oracle B）标注 `A∧¬B` 区；经**迭代 oracle 硬化**排除四类 artifact（输入保真 / harness 污染 / 软后果 / 后果泄漏），证明标签为真。
- **C5 结构 + modest 算法**：不安全区由两种可区分机制构成（速率驱动 LOC + 持续 bank 掉高），边界 2D 可学（AUC 0.912），且可被现实侧风持续 bank 机动够到。

### 3.3 必须避开的 claim

- 标量 margin / 标度律 / `O(log)` 外推（`demand v1` 已杀）。
- 干净的面积扩张律 / "边界随条件可预测移动"（`ep v1` 否，`r50` 钉地板）。
- "我们的搜索比基线快 X 倍"（未跑基线）。

### 3.4 Threats to Validity（四个 artifact = 卖点）

把硬化序列写成可信度论证，正面回应"你这 gap 是不是测量噪声"：

| # | artifact | 若不排除会造成的假结果 | 如何检出/排除 | 现行纪律 |
|---|---|---|---|---|
| ① | 输入保真 | 猛度被 RC 滤波削到 40%，"需求"名存实亡 | 核对 achieved≈commanded（RATE desired/actual vs cmd ≥ 0.9） | 用 GUIDED_NOGPS 角速率设定点绕开 RC 整形 |
| ② | harness 污染 | 清理用的 `GCS_COMMAND LAND` 被计成 failsafe，把干净区涂成 bug 侧 | B 分解 + `MODE.Rsn` 审计 | `MODE.Rsn=GCS_COMMAND` 排除出 oracle B；清理放窗后 disarm |
| ③ | 软后果 | 姿态误差瞬态越界后恢复被当"不安全" | 硬化 oracle A 到真实后果 + 区分 recovered/diverged | oracle A 锚撞地/掉高/不恢复发散；瞬态恢复不计 |
| ④ | 后果泄漏 | 撞地后反弹角速率混进需求特征，AUC 虚高（0.957） | 限定特征到 `active_start..maneuver_end` | 需求量一律 pre-consequence 计算 |

### 3.5 Claim → 证据映射

| Claim | 证据来源 | 关键数字/图 |
|---|---|---|
| C1 存在 | `oracleA v1` | 49 hard-A `clean_unsafe`、B=0；44 撞地/4 掉高/1 发散；`oracleA_v1_outcomes_vs_r.png`、`*_trajectory.png` |
| C2 契约不可见 | `ep v1` | 540 格契约/failsafe/clamp = 0；契约盲区对照表 |
| C3 不充分+不可调修 | `ep v1` | `r50=120` 跨 ANGLE_MAX 30/45/上限饱和；clean-unsafe 在紧 P 下持续 |
| C4 方法学+可信 | `stage0/island/oracleA/demand` | 四 artifact 排除序列；§3.4 表 |
| C5 结构+2D+现实 | `ep v1` + `demand v1` | 机制 21/11/17；2D AUC 0.912；`mechanisms_result.json`；现实 witness `epmovev1_reach_sidewind...` |

### 3.6 建议的论文结构

1. Intro：照规矩却不安全的 RV 失效；契约测试的盲区；OverDraw。
2. 背景与定位：FuSA vs SOTIF；dual oracle；vs PGFUZZ/RouthSearch/LGDFuzzer（§1.4）。
3. 方法：dual oracle 形式化；硬化的 oracle A；harness 卫生；预注册裁决。
4. 实验设置：ArduCopter SITL、栈、P/E/M、网格。
5. 结果：C1–C5，逐条配图（§3.5）。
6. Threats to validity：四 artifact 排除序列（§3.4）——本节是可信度卖点。
7. 讨论：不可调修的含义；contract-clean 一类对认证/测试实践的意义。
8. 相关工作、结论。

---

## 4. 剩余实验（Remaining Experiments）

> 优先级：①②③ 为 CCF-A 竞争力**必需**，④ 为**可选**（决定算法子贡献有没有牙）。所有新实验**必须沿用 §6 的纪律**，否则会回归到已排除的 artifact。

### 4.1 [必需] 第二块表：能量 / RTL 逆风版（做对版）—— 支撑 C1/C2 的广度

**为什么**：早期能量 probe 是**退化的**——`margin ≈ 122 − D` 恒等式、`BATT_LOW_MAH` 在悬停（地速 0.02）触发，风没调制任何东西。要把它重做成一个**真正的 clean-unsafe 演示**：逆风让 RTL 的容量阈值**真的不够**（回不了家 → 干净撞地/迫降），证明 OverDraw **不是一次性现象**。

**规格**
- **P（固定合法）**：`BATT_LOW_MAH` / `BATT_CRT_MAH` 设在文档推荐合法值；机架温和。
- **E（条件轴）**：逆风 `SIM_WIND_SPD` × 方向（顶风回程），分档。
- **M（操作者输入轴）**：外飞距离 D / 任务航点，使 RTL 触发时**仍在远处且顶风**。
- **触发机制**：容量阈值在名义（无风/近距）够、在逆风远距**不够**。
- **Oracle A（硬后果）**：未能返航至 home 半径内 / 中途迫降 / 撞地——**二值大后果**（回没回到家），碾压噪声。
- **Oracle B（干净）**：RTL 在配置阈值**正确触发**、无其它预防 failsafe 误触发、无策略违反 → 确认 `B=0`。
- **裁决/产出**：存在合法 (D, 逆风) 使 A 内 B 外（干净没回到家）；逐格 clean-unsafe 概率；契约触发数（应为 0）。
- **纪律要点**：oracle A 锚"回没回到家"这种真实后果，**不要**用"剩余 mAh < 构造线"；清理放窗后 disarm；`MODE.Rsn=GCS_COMMAND` 排除。
- **tag**：`planc/energy-headwind-v1-<YYYYMMDD>`

### 4.2 [必需] PX4 第二栈复现 —— 泛化性最大杠杆，支撑 C1/C2/C3

**为什么**：单栈（ArduCopter）会被"这是 ArduPilot 特有怪癖"按住。在 PX4 上复现控制权限 OverDraw 是**最大的泛化性杠杆**。

**规格**
- 在 **PX4 SITL** 上重建控制权限场景：等价的姿态/角速率设定点通路（offboard attitude/rate setpoint）、等价的硬化 oracle A、等价的契约 oracle B（PX4 的姿态限幅 + 预防 failsafe 集合）。
- 复现 `oracleA v1` + `ep v1` 的核心：合法激进姿态命令 → 硬后果（撞地/掉高/发散），命令在 PX4 的姿态包络内、**无预防 failsafe**（PX4 侧 `B=0`）。
- 跑一个**缩小版 E/P 网格**确认契约干净性与 2D 边界在 PX4 上同样成立。
- **裁决/产出**：PX4 上 clean-unsafe 存在 + 契约触发为 0；与 ArduCopter 结果并列。
- **纪律要点**：PX4 的 harness 同样会发清理 mode——**先做一次 B 分解审计**（artifact ② 会一模一样地复现），再信任 bug_side 标签。
- **tag**：`planc/px4-ctrlauth-v1-<YYYYMMDD>`

### 4.3 [必需] 契约测试基线对跑 —— 把 C2 子弹打实

**为什么**：现在 C2 是"**我们的** oracle 测得 B=0"。升级成"**独立的契约测试器**在同一批 trace 上什么都找不到"，让 C2 无懈可击。

**规格**
- 在 `oracleA v1` + `ep v1` 的同一批 trace（或同一批输入）上，跑一个 **PGFUZZ 式策略/不变量检查器**（复用或实现其策略集：姿态界、模式不变量、failsafe 应触发条件等）。
- 统计它标记的违反数——预期为 **0**（这正是契约干净的定义），作为对照表/图。
- 可选：同时在**这些输入会触发的"普通 bug 侧"对照点**（如早期 `bug_side` 真违反点，若有）上验证该检查器**能**正常报警，以证明它不是哑的。
- **产出**：契约盲区对照表（OverDraw 区：契约检查器 0 命中；对照 bug 点：正常命中）。
- **tag**：`planc/contract-baseline-v1-<YYYYMMDD>`

### 4.4 [可选] 算法子贡献：主动学习 vs 网格的样本效率 —— 给 C5 的算法面加牙

**何时做**：只有当你想把 C5 的算法主张从"边界 2D 可学"升到"边界**高效**可学"时才做；否则把算法 claim 收到 AUC 0.912 为止。

**规格**
- 在 `ep v1` 的 2D 特征空间里，比较**主动学习/级集估计**与**网格/随机采样**逼近 clean-unsafe 边界所需的**样本数**（达到同等边界精度/AUC）。
- 带多种子、报方差。**诚实**：若优势不显著就如实写，别硬凑"X 倍"。
- **产出**：样本效率曲线（主动学习 vs 网格/随机）。
- **tag**：`planc/active-learning-v1-<YYYYMMDD>`

### 4.5 优先级与依赖

```
必需（CCF-A 竞争力）：
  4.1 能量逆风版 ──┐
  4.2 PX4 复现 ────┼──> ≥2 块表 + 2 栈 + 契约盲区 = 论文广度达标
  4.3 契约基线 ───┘
可选（算法面增强）：
  4.4 样本效率 ──> 决定 C5 算法 claim 的强度；不做则收口到"2D 可学"
依赖：
  4.2 PX4 必须先做 B 分解审计（artifact ② 会复现）
  4.1/4.2/4.3 都沿用 §6 纪律
```

---

## 5. 写作与构建顺序（Build Order）

1. **先固化已有结果的论文资产**：把 `oracleA v1` + `ep v1` 的图表整理成 C1–C5 的成稿图；起草 §3.4 threats 一节（四 artifact 序列，现成）。
2. **跑 4.3 契约基线**（最便宜、直接把 C2 打实）。
3. **跑 4.1 能量逆风版**（凑够 ≥2 块表的广度）。
4. **跑 4.2 PX4 复现**（泛化性，工程量最大，建议留足时间）。
5. **（可选）跑 4.4 样本效率**，决定 C5 算法面强度。
6. **成文**：按 §3.6 结构写；claim 严格按 §3.2/§3.3 措辞，不越界。

---

## 6. 贯穿全程的纪律（从探针方法学继承）

> 这些是六个探针用真金白银换来的；任何新实验偏离其一，就会回归到已排除的 artifact。

1. **P 固定**：单次实验内不搜索、不变 P；P 只作分层对照。生态效度（飞行中改不了参数）+ "这是软件不是纯物理"的锚。
2. **真实后果 oracle ≫ 构造阈值**：oracle A 锚撞地/掉高/不恢复发散/回没回到家这类**二值大后果**；信号必须远大于噪声底。
3. **输入必须真被施加**：核对 achieved ≈ commanded（≥ 0.9）；绕开会削输入的滤波/整形（用角速率设定点而非 RC override）。
4. **harness 卫生**：清理动作（LAND/RTL/disarm）**必须排除出 oracle B**（`MODE.Rsn=GCS_COMMAND`），且最好放在 oracle 窗关闭之后用 disarm。**新栈先做 B 分解审计再信 bug_side 标签。**
5. **需求/特征 pre-consequence**：任何需求量 Φ 只用机动窗内、撞地前的数据；杜绝后果泄漏。
6. **预注册纪律**：oracle 阈值、特征族、外推/网格划分在跑前承诺、落盘；排除点只依据"标签不确定"，绝不依据"规律对没对"；裁决直白渲染，不 fudge。
7. **裁决三态**：PASS/REAL-GAP 要有分量；FAIL 直说；前提不满足/不可表征 = INCONCLUSIVE（换触发或场景），**不是对方法的判决**。

---

## 附录：关键文件与 tag 索引

**Tags（均已 push）**
- `planc/ctrlauth-stage0-v2-20260615`
- `planc/ctrlauth-island-v1-20260615`
- `planc/ctrlauth-oracleA-v1-20260615`
- `planc/ctrlauth-demand-v1-20260615`
- `planc/ctrlauth-ep-v1-20260615`

**结果产物（节选）**
- `planc/results/oracleA_v1_result.json` · `oracleA_v1_report.md`
- `planc/results/demand_v1_result.json` · `demand_v1_report.md`
- `planc/results/mechanisms_result.json`
- `planc/results/ep_movement_result.json` · `ep_movement_report.md`
- 图表：`planc/analysis/oracleA_v1_*.png`、`demand_v1_*.png`、`ep_movement_*.png`

**未纳入提交（保持原状）**
- `planc/results/minalt_groundcontact_v3_report.md`（用户未暂存修改）
- 新 SITL 原始日志、`ep_movement_v1_partial.json`（本地未提交）

---

*文档结束。任一剩余实验（§4.1–§4.4）可按需展开成完整的 Coding Agent prompt（沿用 §6 纪律 + 预注册裁决块）。*

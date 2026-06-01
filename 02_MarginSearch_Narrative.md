# MarginSearch 研究方案

**版本**：v2.1（单支柱聚焦版；对 SparsePilot v1.2 的重大迭代）
**目标会议风格**：ISSTA / ASE / FSE / ICSE 等 SE 方向 CCF-A
**核心定位**：以 viability 理论为结构锚点，对飞控合法飞手输入的 **feasible-but-unsafe violation 边界**做 query-efficient 的边界刻画、跨条件 warm-start falsification 与 minimal trigger 提取。**单支柱、小而美（RouthSearch 体量）**；飞手行为锚定的 severity 分层下放为后续工作（见 §11）。

**建议主标题**：

> **MarginSearch: Tracing the Feasible-but-Unsafe Boundary of Pilot Inputs in UAV Flight-Control Programs via Viability-Anchored Manifold Continuation**

**命名说明**：与 RouthSearch 同属一个研究 genre（principled criterion + structured search + offline oracle + PX4/ArduPilot 评估）。RouthSearch 用 Routh-Hurwitz 判据结构化 PID 参数搜索；MarginSearch 用 viability / Nagumo barrier 判据结构化飞手输入空间中"安全 vs 不可避免违规"的 **margin**（边界）。"Margin" 同时指安全裕度、到 violation 的距离、以及所要刻画的边界本身。

---

## 0. 这是一次迭代——v1.2 发生了什么

v1.2（SparsePilot / SparseShield）的承重假设是 **SPH（Support Persistence Hypothesis）**：robustness-gradient support 全局 sparse 且沿搜索路径 persistent，从而支撑 query-efficient 的 support 复用。

初步实验**否定了 SPH**。诊断结论：

- 安全区（远离 violation）主要是 **robustness 平台**（梯度 ≈ 0），少数情况是**浅而散的碗**；两者都**没有可利用的低维结构**。
- 稀疏结构（若有）只在 **violation 边界附近**出现——即"在搜索已付完代价之后"，无法支撑全局复用。

**迭代的核心翻转**：平台不是要掩盖的负面结果，而是新方法的**第一幕动机**——平台意味着梯度走不出安全区，这正是为什么必须先有结构无关的搜索把系统推到边界，再在边界附近利用结构。研究对象因此从"全局稀疏 support 复用"重构为"**低维 violation 边界流形的刻画与跨条件 warm-start**"。

**被丢弃 / 降级**：

- 全局 SPH（被数据否定）。
- SparseShield（runtime guard，security 味、要反复打补丁）。
- **pilot learning 从核心移出**——它属于不同的贡献品类（field-data 驱动的真实性/severity 方法学），与核心（搜索方法 + 边界刻画）混在一篇会变成"两个半篇"。下放为后续工作（§11）。

**第一篇定位**：单支柱、聚焦、可独立评估。核心已是完整故事——方法 + 结构性发现 + 理论锚点 + 自洽评估，**无需第二支柱**。

---

## 1. 一句话主张

> 飞控程序对真实飞手可输入的时序控制缺少 mode-aware 的语义校验。对每个 flight-mode contract，feasible-but-unsafe 的输入集中在一个 **mode-conditioned 低维边界流形**上；该流形是状态空间 viability 边界在闭环映射下的输入空间原像，在 regime 内随条件**连续移动**、在 mode transition 处**跳变**。MarginSearch 用硬剪枝降维、结构无关搜索（契约续化 + 大协调移动）抵达边界、符号 bracketing 确认违规、局部流形刻画恢复稀疏敏感方向、跨条件 warm-start 续化高效铺开边界，并将 violation 最小化为人类可读、return-to-neutral 后仍违规的 minimal trigger。理论上由 viability / Nagumo barrier / HJ reachability 提供边界存在性、低维性、连续性与切换断裂的**结构性**担保（非可计算）。

---

## 2. 威胁模型 / 问题定义（SE testing 框架，非安全）

### 2.1 输入与执行

飞手输入序列 \(U=[u_1,\dots,u_T],\ u_t=[r_t,p_t,y_t,th_t]\)。给定 mode \(m\)、初始状态 \(x_0\)、mission \(M\)，闭环飞控程序产生 flight log \(Y(U)\)。给定 mode contract \(\varphi\)（STL/MTL），offline oracle 给出判定与 robustness：

\[
Y(U)\models\varphi \quad,\quad \rho_\varphi(Y(U))>0\ \text{safe},\ <0\ \text{violation}.
\]

### 2.2 输入空间模型（不是 threat model）

\[
\mathcal{F}_{pilot}(m)=\{U:\ u_t\in[-1,1]^4,\ |u_t-u_{t-1}|\le R_m,\ \text{respects deadzone \& mode-availability}\}.
\]

- 这是**测试输入空间规约**：真实飞手能发出的合法时序输入。输入信号 rate-limited 且连续（人手打不出瞬跳）——这是 feasibility 的定义。
- 排除**自毁（self-sabotage）**：直接命令坏结果（如低空对地推满下行杆）的输入不计入研究区域；否则平凡。PGFuzz 的 self-sabotage 排除是先例。
- 固定不变：PID / firmware / sensor / mixer / actuator / failsafe / mission / airframe（全部正确、未被篡改）。

### 2.3 研究区域（几何）

设 F（全可行盒）⊇ P（非自毁合理区）；C 为控制器安全区。
平凡期望 P ⊆ C；**本文要展示 P ⊄ C**，且越界输入位于 P 内部（不贴满杆边界）。
**T**（trigger 流形）= P 中 \(\rho_\varphi<0\) 的部分 = 要刻画的 violation 边界。

该几何框架**自动给出干净的归因**：P\C 中的违规，输入可归因于合理飞手、违规可归因于控制器。

---

## 3. 理论锚点（viability）：给结构，不给位置

| 理论对象 | 在 MarginSearch 中的角色 |
|---|---|
| **Nagumo 切锥条件 / barrier theory**（De Donà–Levine、Isaacs barrier） | 局部边界判据（Routh-Hurwitz 类比）：边界 = 可行速度场与约束切锥相切处；binding 的 active constraint 通常少 → **预测边界局部稀疏** |
| **Viability kernel**（Aubin） | "存在控制能保持安全"的状态子集；其补（在名义安全集内）= **不可避免违规** = 状态级 feasible-but-unsafe |
| **HJ reachability** | 安全集 = value function 水平集；有界输入可达集 = value function 下水平集 → **边界 codim-1**，低维流形假设的背书 |
| **viability kernel 比较定理 / 连续依赖** | 边界随条件连续移动 → **warm-start 的理论担保** |
| **switched-system viability / 混合可达性** | 边界在切换面断裂 → **transition 处跳变 = bug** |

**必须吞下的限制**：以上在黑盒闭环固件上**不可计算**（HJB PDE 维数灾难 + 需模型 + 非光滑 STL）。这正是 search-based falsification 存在的理由。所以 MarginSearch **不计算**这些判据，而是用它们作为**结构性假设的理论陈述**，再用搜索经验性地恢复并利用这些结构。

**与 RouthSearch 的不对称（要诚实写出）**：Routh-Hurwitz 是参数空间闭式判据，能**定位**边界；viability 只给**结构**（存在/低维/连续/切换断裂/active-constraint 稀疏），不给位置 → MarginSearch 搜索负担更重，而"理论保证有结构却算不出、用搜索恢复"即为缝隙。

**状态空间 vs 输入空间**：viability 是状态空间对象；MarginSearch 搜输入空间。T 是状态空间 viability 边界在闭环映射下的**原像**，映射有 fold 时更复杂 → 结构大体传递但需经验验证。

---

## 4. 中心假设（取代 SPH）

给定 mode / property，定义到 violation 边界的距离由 \(|\rho_\varphi|\) 度量。

**H1 — 低维边界流形（Low-Dim Trigger Manifold）**
T 的边界局部由少数敏感 channel-time 方向（= active constraints）张成；其有效维数 \(\ll D\)。

**H2 — 跨条件连续（Cross-Condition Boundary Continuity）**
在一个 regime（同 mode、同动力学分支）内，边界随条件（state / contract / mission）连续移动，使一个已解条件可 warm-start 邻近条件。

**H3 — 过渡断裂（Transition Discontinuity = Bug）**
连续性在 mode / sensor-source transition 处断裂；该跳变恰为 feasible-but-unsafe 的核心触发形态。

**可证伪指标**：
- H1：多边界点处 active 方向的 PCA / participation ratio（有效维数）。
- H2：相邻条件边界点的位移连续性；warm-start 相对冷启动的 query 节省。
- H3：连续性断点是否定位于 transition；断点处是否即 violation。

---

## 5. MarginSearch 算法（单支柱：五步 + 输出）

```text
[1] 硬剪枝（Hard Pruning）— 先验型降维（不依赖梯度）
    architectural prune：从源码/文档剪掉 mode m 下可证无效的 stick channel-time group。
    feasibility/basis：pulse/ramp/分段常数基底，4T → 4K；bounded-rate + 人类动作结构。
    → 把环境维数从原始 tick 压到可行输入流形维数。注意：这是先验降维，
      不是流形内在维降维（嵌入未知，找出它仍是搜索本身）。

[2] 抵达边界（Stage 1）— 结构无关，治平台冷启动
    平台（梯度≈0）下不能靠梯度下降走出安全区，故：
    (a) 契约续化（continuation in contract space）：先解放松后的 shaped contract
        （violation 区大、易命中），再逐步收紧到真 contract、沿途 track 边界收缩；
    (b) 大协调移动：在 [1] 的 group/basis 上做大幅、协调的多窗扰动
        （平台是一阶/局部性质，靠大移动可达边界，局部梯度看不见它）。
    （可选的 log-seeded 热启动属后续工作，见 §11，核心不依赖之。）

[3] 符号确认与定位（Sign-Based Bracketing）— 不赌梯度
    取一对异号点（ρ>0 与 ρ<0），沿连线 bisection 定位边界点。
    对 cliff / ramp 都成立；不依赖"逼近时梯度变大"（平台数据更像 cliff）。
    更贴 RouthSearch（用稳定/不稳定符号 refine 边界）。

[4] 局部流形刻画（Manifold Probing）— 恢复稀疏敏感方向
    在边界点探可行方向：
      跨过去方向（|Δρ| 大）= 流形法向 = 稀疏敏感角色（≈ active constraints）；
      沿着走方向（|Δρ| 小，留在边界）= 流形切空间。
    可选：patch 内若近似线性，用线性/Koopman surrogate 做 Newton/bisection 精修
      （仅 patch 内插值；跨 regime 不外推）。

[5] 跨条件 warm-start 续化（Continuation）— 真正的效率来源
    沿边界切空间移动 = RouthSearch 式 coordinate / continuation 搜索，铺开一片连通分量；
    跨相邻条件（state/contract/mission）warm-start：用已解边界的法向/切空间做热启动，
      省掉邻居的冷启动随机代价。
    效率计量单位 = campaign（N 个相关条件），非单次。单次冷搜索 ≈ 随机（诚实让步）。
    连续性断裂处停止续化 → 触发 restart（新连通分量）并标记为候选 transition bug（H3）。

[输出] Trigger 提取与边界刻画
    delta debugging 将 violation 最小化为 short-duration、low-support、return-to-neutral
      后仍 violation 的人类可读 trigger。
    边界刻画产物：低维流形描述 + 连续分量 + transition 断点清单。
    （发现的 trigger 可进一步按 human-reachability 排序，属后续工作，见 §11。）
```

---

## 6. 非平凡性（Non-Triviality）— 完全靠结构判据，不依赖 pilot 模型

trigger 应尽量满足：bounded-rate；非持续满幅 / 低幅；short-duration；small support；**默认/合法参数下触发**；与 mode contract 明确相关；**且落在 P 内部、不贴满杆边界**。

**最强证据形态：return-to-neutral 后仍 violation。** 它直接证明违规不是"一直按着坏杆"，而是输入归位后系统仍越界——这是飞控自身缺陷最有力的证据，且**完全不需要 pilot 模型**。第一篇的非平凡性主张就压在 return-to-neutral + non-self-sabotage 上，站得住。

（把非平凡性从"结构上合理"升级到"定量上人类可达"，是后续工作的事，见 §11。）

---

## 7. 评估计划（RQ，单支柱）

- **RQ1（H1）**：每个 mode×property 的 violation 边界流形有效维数是否 ≪ D？（active-direction PCA / participation ratio）
- **RQ2（H2，决定论文性质）**：相邻条件的边界是否连续？warm-start 相对每次冷启动的随机/falsification baseline 在 **campaign** 上节省多少 query？
- **RQ3（机制）**：边界逼近是 cliff 还是 ramp？bracketing 在两种情形下的稳健性。
- **RQ4（H3）**：连续性断点是否定位于 transition？断点处是否即 violation？
- **RQ5（trigger）**：找到的 trigger 是否 minimal、non-trivial（return-to-neutral 仍违规）、root-cause-distinct？
- **RQ6（中心区，作为动机的实证）**：安全区是平台 / 浅散碗（无可利用结构），naive 梯度 / 全局低维 BO 在此必然失败。

**Baseline**：均匀随机；robustness-guided falsification（S-TaLiRo / PSY-TaLiRo 式、local descent + SA）；surrogate-guided falsification（decision-tree / Koopman）；高维 sparse / active-subspace BO（SAASBO / GraCe / REMBO）；GA / CMA-ES。
**主图**：support sparsity（TopKCoverage）与稳定性（Jaccard / MassOverlap）**作为到边界距离 \(|\rho_\varphi|\) 的函数** + 边界流形有效维数 + 跨条件 warm-start 节省曲线。此图取代 v1.2 的全局 support heatmap，是 MarginSearch 的 "RouthSearch boundary figure"。

---

## 8. 论文性质的分岔

由 **RQ2（沿边界 / 跨条件 warm-start 是否显著省 query）** 单独决定：

- 立得住 → **方法论文**：两阶段 + 跨条件摊销 efficiency + 理论锚定。
- 立不住（仅 H1、H3 成立，摊销不显著）→ **特征化论文**：低维边界流形 + 过渡跳变即 bug + minimal trigger，放弃 reuse efficiency 承诺（仍可发）。

---

## 9. 线性关系的纪律（写作红线）

- **局部线性真**（ΔY ≈ J·ΔU + ε）；**全局线性假**（平台/悬崖/饱和/切换/浅散碗反驳之）。
- 用法：局部线性 = **Stage-2 patch 内精修引擎**（步骤 [4]）；其断裂 = bug 信号。
- 纪律：patch 内**插值合法**；跨 regime **外推非法**（平台 J→0 预测"永不违规"，悬崖 J 跳）。
- **禁语**（沿用 v1.2 §27）：不得声称"飞控输入输出线性 → 可攻击"。对外只说"regime 内分段近似线性，violation 集中在线性失效处"。

---

## 10. Related Work 定位

- **RouthSearch**：最重要叙事参考。同 genre，判据从 Routh-Hurwitz（参数空间、可定位）换为 viability/Nagumo（状态空间、仅结构）。
- **UAV / RV fuzzing**（RVFuzzer、PGFuzz 等）：对象是参数 / GCS 命令；MarginSearch 对象是 manual pilot-stick 时序，固定参数/传感器/固件，且做边界刻画，而非仅 violation trace。PGFuzz 的 self-sabotage 排除是本文非平凡性约束的先例。
- **CPS falsification**（S-TaLiRo / PSY-TaLiRo、local descent + SA、surrogate / Koopman-guided）：继承 robustness-guided oracle 形式；差异在 viability-anchored 边界续化 + 跨条件 warm-start。需正面区分 surrogate-guided（已能高效找多反例）。
- **Reachability / viability theory**（HJ、Aubin viability、barrier theory、switched-system viability）：本文的结构理论锚点；强调其不可计算性正是搜索的存在理由。

---

## 11. Future Work / Paper 2 Roadmap — 飞手行为锚定的 severity

pilot learning 是一个**不同品类**的贡献（field-data 驱动的真实性/severity 方法学），自然建立在第一篇的 trigger 之上，构成独立的后续论文。要点：

- **种子用途（log-seeded 提效）= 次要。** 用近边界的真实 log 片段（低 \(\rho_\varphi\)）热启动 Stage 1，跳过平台冷启动。这只是个优化，**不足以单独成篇**；可作为 Paper 2 的配菜，或在 Paper 1 一句话提及为可选热启动来源。第一篇的 efficiency 故事在 campaign 级 warm-start 上成立，**不依赖**它。
- **severity / 双流形交集（T ∩ H）= 真正的 Paper 2 贡献。** 在真实飞行 log 上训生成式 \(p_{human}(U)\)，给每个 trigger 打"有多像人"的分数：
  - **T ∩ H = 高危真 bug**（人会做且违规）；T \ H = 潜伏低危（能做不会做）；H \ T = 安全人类操作。
  - 评估问题变成"trigger 流形有多少落在人类可达内"——T∩H 大 = 严重人类可触漏洞；T∩H≈∅ = 该 contract 对正常飞手**实质受保护**（正面发现）。
  - 理论接口：T∩H = "人类可信输入是否离开 viability kernel"，viability 直接背书。
- **Paper 2 的独立性要求**：severity 度量须对**外部锚**验证（真实 incident 报告 / 人类判断 / log 中的近违规事件），不能只说"加了个距离分数"或"种子让搜索更快"——否则太薄。**让 Paper 2 建在 severity 方法学上，而非建在种子提效上。**
- **风险（属 Paper 2，不拖累 Paper 1）**：手动/RC log 含原始 stick 的可得性；开环重放是行为先验非反应式 pilot；额外 sim-to-real。
- **Paper 1 的前向指针（写一句即可）**：发现的 trigger 可进一步按 human-reachability 排序与定量 severity 分层，留作 future work。

**分拆的好处**：Paper 1 不必等 pilot 数据；Paper 2 干净 cite Paper 1、是递进关系；两篇而非一篇。
**唯一张力**：若 reviewer 觉 Paper 1 相对 RouthSearch + falsification 太增量，severity stratification 是备用差异化牌，可临时拉回。默认仍走分拆，不为最坏情况牺牲聚焦。

---

## 12. 局限与 Threats to Validity（Paper 1，诚实清单）

1. **理论不可计算**：viability 仅提供结构假设，不提供完整性证明；本文不许诺穷尽边界（testing 本不许诺，Dijkstra）。
2. **完整性限制**：continuation 只覆盖连通分量；跨分量需 restart 且不可证明找全；边界在 transition 处有 strata。
3. **状态 vs 输入空间**：T 为 viability 边界原像，映射 fold 可能破坏结构传递，需经验验证。
4. **sim-to-real**：violation 在 sim 判，违规真实性继承 sim-to-real gap。
5. **承重 bet 未全验**：H2（跨条件 warm-start 省 query）目前数据不足，是最大开放风险。

---

## 13. 最小闭环（动手顺序）

1. near-boundary probing：测 cliff vs ramp（RQ3）、边界局部稀疏（H1/RQ1）。
2. 两个相邻条件的边界数据：粗看 warm-start 省不省（H2/RQ2，决定论文性质）。
3. 实现 bracketing + 局部流形 probing + 续化（步骤 [3][4][5]）。
4. delta debugging 做 minimal trigger，确认 return-to-neutral 仍违规（RQ5 非平凡性）。
5. 把 Nagumo 切锥条件写入 background，作为"边界局部稀疏 = active constraint 少"的理论陈述。

---

## 14. 当前关键词

```text
feasible-but-unsafe pilot input        viability kernel / Nagumo barrier
mode-contract violation                 violation boundary manifold (low-dim)
sign-based bracketing                    cross-condition warm-start continuation
transition discontinuity = bug          return-to-neutral still violates (non-triviality)
hard pruning (architectural + basis)     query-efficient falsification (campaign-level)
[Paper 2] pilot behavior prior (p_human) / severity stratification (T ∩ H)
```

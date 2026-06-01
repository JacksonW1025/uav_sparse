# SparsePilot RQ1 实施计划

**读者**：coding agent（Claude Code / Codex）
**目标**：在新建仓库中搭建 measurement pipeline，验证 SparsePilot 方法的核心经验假设 SPH 是否成立
**硬件**：NVIDIA Jetson AGX Orin 64GB（ARM64），PX4 / ArduPilot / jMAVSim / Gazebo / SITL 已预装
**本轮不做**：完整 SparsePilot 攻击搜索、baseline 对比、SparseShield、delta debugging、cheap sparse probing 实现

---

## 如何使用本文档

按以下顺序阅读和执行：

| 步骤 | 内容                                        | 状态   |
| ---- | ------------------------------------------- | ------ |
| 1    | 读 §0 背景 → 理解项目目标和你要建什么       | 阅读   |
| 2    | 读 §1 三条主张 → 理解 pipeline 要回答的问题 | 阅读   |
| 3    | 读 §2 固定参数 → 知道哪些值不能改           | 阅读   |
| 4    | 完成 §3 开工前确认 → 回答 4 个问题给作者    | 交付 1 |
| 5    | 实现 §4 仓库结构 + 配置                     | 实现   |
| 6    | 按 §5 模块契约实现各模块                    | 实现   |
| 7    | 执行 §6 Phase 0 → 报告并暂停                | 交付 2 |
| 8    | 执行 §7 Phase 1 → 报告并暂停                | 交付 3 |
| 9    | 执行 §8 Phase 2 → 报告并暂停                | 交付 4 |
| 10   | 执行 §9 Phase 3 → 报告并暂停                | 交付 5 |
| 11   | 执行 §10 Phase 4 → 全部交付                 | 交付 6 |

每个 Phase 完成必须停下并报告，**不要连续推进**。原因：早期 phase 的超参（perturb_delta、sim_speed_factor、seed 数量）会决定后期 phase 的有效性。Phase 1 出问题但跑到 Phase 3 才发现，浪费 30+ 小时仿真时间。

参考章节 §11–§14 在执行过程中随时查阅：判定规则、不做清单、坑位排查、最终交付清单。

---

## 0. 背景：先理解你要做什么、为什么

### 0.1 研究项目要解决的问题

无人机飞控程序（PX4、ArduPilot 等）依赖 PID 和一系列 mode-specific 控制逻辑执行飞行任务。每个飞行模式（Position、Hold、Loiter、Land 等）本质是对飞手行为的契约：例如 PX4 Hold 承诺"sticks 回中后保持当前位置和高度"，AP Loiter 承诺"sticks 回中后停止位移"。

飞控程序对飞手 stick 输入只做 **语法层** 检查：值是否在合法范围、相邻时刻变化率是否合规、deadzone 是否正确处理。它不做 **语义层** 检查：一串完全合法的 stick 输入会不会让飞机进入一个 mode 契约失效的状态？答案是会。存在一些短暂、非满幅、回中后仍 violation 的输入序列，能通过所有原生检查但破坏 mode 契约。这类输入称为 **feasible-but-unsafe pilot input**。

本研究项目（代号 **SparsePilot**）的最终目标：在 PID、sensor、firmware、mixer 全部固定为合法默认值的前提下，系统化地发现这类输入，并把它们最小化为人类可读的 trigger，用于 paper 的 evidence 和后续 runtime guard 设计。

### 0.2 为什么不能直接用 fuzzing 或通用黑盒优化

飞手输入是高维时序信号。用 0.5 秒窗口、5 秒 horizon、4 个 channel 参数化后，搜索空间是 4 × 10 = 40 维连续空间。每次评估一个候选输入意味着跑一次完整仿真，墙钟约 10 秒级。在这个 budget 下：

- 随机 fuzzing：维度太高，命中率低
- 通用黑盒优化器（CMA-ES、SAASBO 等）：query 预算消耗快，且没有利用飞控的领域结构

SparsePilot 的核心赌注是：**飞控的输入-输出影响关系有可利用的稀疏结构**。具体来说，对一个固定的 mode contract φ、固定的飞行状态、固定的短 horizon，property robustness 关于 pilot input 的梯度

```
g = ∇_U ρ_φ(Y(U))
```

在大多数维度上接近零，只有少数几个 "channel-time group" 在起作用。如果能用少量 query 估计出这些关键 group（称为 *support*），就能把昂贵的仿真预算集中花在它们上面，避开稠密随机扰动。

### 0.3 关键术语（agent 必须分清）

| 术语               | 含义                                                         |
| ------------------ | ------------------------------------------------------------ |
| Pilot input `U`    | 飞手通过 RC / joystick / MAVLink `MANUAL_CONTROL` 发送的 stick 时序信号，4 个 channel × T 个时刻 |
| Channel-time group | 把输入切成离散窗口后，"哪个 channel 在哪个时间窗"的组合，例如 `pitch@1.0-1.5s`。本计划 D=40 表示 40 个 group |
| Mode contract `φ`  | 用 STL/MTL 表达的 mode 语义保证，例如 "Hold 回中后 8 秒内位置漂移 < 2 米" |
| Robustness `ρ_φ`   | STL 公式在仿真轨迹上的连续松弛值。ρ > 0 满足契约（越大越安全），ρ < 0 违反契约（越负越严重） |
| Support `S`        | 梯度 `g` 中绝对值显著的 group 集合 `S = {i : |g_i| significant}`，是方法真正要追的对象 |

直观地说，support 回答的问题是："要让 Hold 模式回中后漂移变大，哪几个 stick channel 在哪几个时间窗里改最有效？"

### 0.4 Support Persistence Hypothesis (SPH)

SparsePilot 整套方法建立在一个尚未证实的经验假设上：

> 对固定 mode、固定短 horizon，property 特定的 robustness 梯度 support 同时满足：
>
> 1. **Sparse**：|S| 远小于 D，大部分 channel-time group 对 robustness 影响很小
> 2. **Mode-conditioned**：不同 mode 或不同 property 的 support 几何上明显不同；这种稀疏不是飞控架构本身的全局稀疏（否则可以从源码读出，无需运行时探测）
> 3. **Persistent**：沿小步局部搜索路径 U(0) → U(1) → ...，连续两步的 support 大部分重叠，不会每步完全重排

SPH 成立 → cheap sparse probing 加 active-set search 加 persistence reuse 都有效，可以显著节省仿真预算 → SparsePilot 方法成立。
SPH 不成立 → 整套方法没有理论支点 → paper 主张必须重构。

### 0.5 为什么本轮必须先验证 SPH、必须用 full FD 验证

SPH 是经验假设，不是数学定理，**必须先用实验直接验证**。这就是本轮工作的使命。SparsePilot 方法本身（cheap sparse probing、active-set search、guard、delta debugging）属于后续 RQ，在 SPH 没有立住之前不要碰。

本轮必须用 **reference full finite-difference gradient** 测量 support，不能用 SparsePilot 自家的 cheap sparse probing。原因：cheap probing 用 LASSO / OMP 等稀疏恢复方法，其 L1 归纳偏置会 *人为* 制造稀疏 support。如果用 cheap probing 跑出来的 support 看起来 sparse、persistent，无法判断这是 SPH 真成立、还是 LASSO 的稀疏先验造成的假象。

Full FD 是逐 group 各做一次正负扰动、直接读出

```
g_i = (ρ_φ(Y(U + δ e_i)) - ρ_φ(Y(U - δ e_i))) / (2δ)
```

没有任何稀疏先验。D=40 时一次完整 snapshot 是 80 次 query，墙钟约 13 分钟，可承受。

### 0.6 这份计划要建什么

agent 要在新仓库里建立一条 measurement pipeline，完成下面四件事：

1. 在 4 个 (platform, mode) scenario × 3 个连续 robustness property × 3 个 seed 上，对 zero pilot input 跑 reference full FD gradient snapshot
2. 从每个 snapshot 输出 channel-time group heatmap，可视化梯度幅值在 40 个 group 上的分布（对应 SPH 子主张 *Sparse*）
3. 比较跨 scenario / 同 scenario 跨 seed 的 support 重叠（对应 SPH 子主张 *Mode-conditioned*）
4. 在每个 (scenario, property, seed) 上从 zero input 出发做 5 步局部 gradient-sign 更新，每步重新做 full FD snapshot，测量连续两步 support 的重叠（对应 SPH 子主张 *Persistent*）

完成后产出三张主图 + 三张 csv 表，足以判定 SPH 在测试 scenario 上是否成立。

---

## 1. 三条主张（本轮 RQ）

pipeline 必须回答三个具体问题：

1. **Sparse**：固定 mode 和 property 时，robustness gradient 是否集中在少量 channel-time groups 上？
2. **Mode-conditioned**：相同 property 下，不同 mode/platform 的 support 是否显著不同？同 scenario 不同 seed 的 support 是否稳定？
3. **Persistent**：沿小步局部更新路径，连续两步的 support 是否大部分重叠？

每个问题的判定标准见 §11。三个问题都用 §0.5 论证过的 reference full FD 测量。

---

## 2. 固定实验参数

以下参数 **不要修改**。它们是经过权衡的最小可行配置。

### 2.1 输入参数化

```text
horizon_s              = 5.0       # 扰动窗口长度
window_s               = 0.5       # 每个 channel-time group 的时长
num_windows            = 10
channels               = [roll, pitch, yaw, throttle]
D                      = 4 * 10 = 40
neutral_tail_s         = 8.0       # post-neutral observation 窗口
perturb_delta          = 0.08      # FD 步长
path_eta               = 0.06      # path persistence 单步更新幅度（必须 < perturb_delta）
```

`path_eta < perturb_delta` 是硬约束：保证 path persistence 每步落在 FD 探测过的局部线性区内。

### 2.2 Scenario 列表（4 个）

| scenario_id    | platform  | 扰动期 mode | 观察期 mode | 备注                                         |
| -------------- | --------- | ----------- | ----------- | -------------------------------------------- |
| `px4_position` | PX4       | Position    | Position    | 同 mode，sticks 自然回中                     |
| `px4_hold`     | PX4       | Position    | Hold        | mode switch at t=5s（PX4 Hold 不接受 stick） |
| `ap_loiter`    | ArduPilot | Loiter      | Loiter      | 同 mode                                      |
| `ap_althold`   | ArduPilot | AltHold     | AltHold     | 同 mode；只评估 alt 和 velocity property     |

`px4_hold` 必须做 mode switch，因为 PX4 Hold 拒绝 stick。这正好提供"扰动后被强制接管"的语义，是 SPH 在 mode transition 下是否成立的弱测试。

### 2.3 Property 列表（3 个，全连续 robustness）

```text
post_neutral_xy_drift:
  ρ = d_max_m - max_{t ∈ [neutral_start, neutral_start + neutral_tail_s]} ||pos_xy(t) - pos_xy(neutral_start)||
  d_max_m = 2.0

post_neutral_alt_drift:
  ρ = h_max_m - max_{t ∈ [neutral_start, neutral_start + neutral_tail_s]} |alt(t) - alt(neutral_start)|
  h_max_m = 1.0

post_neutral_xy_velocity:
  ρ = v_max_mps - max_{t ∈ [neutral_start, neutral_start + neutral_tail_s]} ||vel_xy(t)||
  v_max_mps = 1.0
```

`neutral_start = horizon_s = 5.0 s`（按仿真物理时间）。`ap_althold` 不评估 xy_drift（AltHold 不锁水平位置，xy 漂移是 mode 设计行为不是 violation）。

### 2.4 Seeds

每个 (scenario, property) 跑 3 个 seed：`[0, 1, 2]`。seed 控制 simulator 初始状态的小幅扰动（初始 yaw、起始位置 ± 噪声），用来量化 simulator 非决定性对 support 的影响。

### 2.5 Scenario 时间线（统一）

```text
t = 0.0 s   simulator 启动 + arm + takeoff
t ≈ 8.0 s   稳态悬停在 takeoff_alt = 5 m（PX4）或 10 m（ArduPilot）
t_zero      标定为飞行稳态时刻（每个 platform 各自检测，写进 metadata）
[t_zero, t_zero + 5.0]      扰动期：按 50 Hz × 墙钟换算 注入 stick
t_neutral = t_zero + 5.0    sticks 全部归零；若 scenario 要求，此刻切换 mode
[t_neutral, t_neutral + 8.0] 观察期：sticks 保持 0
t_end = t_neutral + 8.0     仿真结束，dump log
```

**关键工程坑**：如果 SITL 使用加速比（PX4 `PX4_SIM_SPEED_FACTOR` 或 AP `SIM_SPEEDUP`），MANUAL_CONTROL 的发送间隔必须按墙钟时间换算。例如 `sim_speed_factor=5` + 50 Hz 物理频率 → 墙钟需要每 4 ms 发一次。如果按 20 ms 发，PX4 会把 stick 当作长时间静止，扰动信号丢失但 ULog 看起来一切正常。**Phase 1 smoke test 必须验证 manual_control_setpoint 在 ULog 里随时间变化。**

---

## 3. 开工前确认（交付 1）

**在写任何代码之前**，回答以下四个问题，作为对作者的第一次交付：

1. 本轮工作要回答哪三个问题？为什么必须用 reference full FD 而不是 cheap sparse probing 作为本轮真值源？
2. `path_eta` 为什么必须严格小于 `perturb_delta`？如果反过来会发生什么？
3. 在加速比 `sim_speed_factor=5` 下，MANUAL_CONTROL 应该按每多少毫秒发送一次？为什么？
4. 如果 Phase 2 跑出来的 heatmap 看起来是均匀噪声，你会先尝试哪三件事？哪三件事不会做？

四个问题回答清楚后再开始 §4。如果某个问题答不上来，回头重读 §0 和 §1。

---

## 4. 仓库结构与配置

### 4.1 仓库取名

仓库根目录名作者会另行指定（例如 `uav-input-falsification`）。Python package 名建议用 `sparsepilot`（无下划线，PEP 8 风格）。下方目录树中的占位用 `{repo}/` 和 `sparsepilot/`。

### 4.2 目录树

```text
{repo}/
├── README.md
├── pyproject.toml
├── configs/
│   ├── rq1_minimal.yaml
│   └── synthetic_sanity.yaml
├── src/sparsepilot/
│   ├── __init__.py
│   ├── config.py
│   ├── groups.py
│   ├── input_model.py
│   ├── properties.py
│   ├── logs.py
│   ├── query.py
│   ├── gradients.py
│   ├── support.py
│   ├── metrics.py
│   ├── plots.py
│   ├── vehicle/
│   │   ├── __init__.py
│   │   ├── base.py
│   │   ├── synthetic.py        # Phase 0 用，必须先于真实 adapter 实现
│   │   ├── px4.py
│   │   └── ardupilot.py
│   └── runners/
│       ├── sanity.py
│       ├── smoke.py
│       ├── fd_snapshot.py
│       ├── path_persistence.py
│       └── analyze.py
├── runs/
├── scripts/
│   ├── kill_sim.sh
│   └── start_px4.sh
└── tests/
    ├── test_input_model.py
    ├── test_properties.py
    ├── test_metrics.py
    └── test_fd_on_synthetic.py
```

### 4.3 主配置文件

`configs/rq1_minimal.yaml`：

```yaml
experiment_id: rq1_minimal_v0

input:
  horizon_s:            5.0
  window_s:             0.5
  neutral_tail_s:       8.0
  channels:             [roll, pitch, yaw, throttle]
  min_value:            -0.7
  max_value:            0.7
  max_delta_per_window: 0.25
  perturb_delta:        0.08

properties:
  post_neutral_xy_drift:    {d_max_m: 2.0}
  post_neutral_alt_drift:   {h_max_m: 1.0}
  post_neutral_xy_velocity: {v_max_mps: 1.0}

scenarios:
  - id: px4_position
    platform: px4
    perturb_mode: Position
    observe_mode: Position
    takeoff_alt_m: 5.0
    properties: [post_neutral_xy_drift, post_neutral_alt_drift, post_neutral_xy_velocity]

  - id: px4_hold
    platform: px4
    perturb_mode: Position
    observe_mode: Hold
    takeoff_alt_m: 5.0
    properties: [post_neutral_xy_drift, post_neutral_alt_drift, post_neutral_xy_velocity]

  - id: ap_loiter
    platform: ardupilot
    perturb_mode: Loiter
    observe_mode: Loiter
    takeoff_alt_m: 10.0
    properties: [post_neutral_xy_drift, post_neutral_alt_drift, post_neutral_xy_velocity]

  - id: ap_althold
    platform: ardupilot
    perturb_mode: AltHold
    observe_mode: AltHold
    takeoff_alt_m: 10.0
    properties: [post_neutral_alt_drift, post_neutral_xy_velocity]

seeds: [0, 1, 2]

persistence_path:
  steps:               5
  top_m_update_groups: 3
  eta:                 0.06

simulator:
  px4:
    sim_speed_factor:   5.0
    mavlink_url:        udpin:127.0.0.1:14540
    manual_control_hz:  50
    cleanup_each_run:   true
  ardupilot:
    sim_speedup:        5.0
    mavlink_url:        udpin:127.0.0.1:14550
    manual_control_hz:  50
    cleanup_each_run:   true

logging:
  level:     INFO
  jsonl:     runs/rq1_minimal_v0/logs/queries.jsonl
```

`configs/synthetic_sanity.yaml` 用于 Phase 0，scenarios 部分换成单个 `synthetic` scenario，platform 字段写 `synthetic`。

---

## 5. 模块契约

每个模块给出接口、行为约束、单元验收。Agent 实现时必须按签名来；签名内部的算法细节可以自由发挥。

### 5.1 `groups.py`

```python
@dataclass(frozen=True)
class Group:
    group_id:   int
    channel:    str
    window_id:  int
    t_start:    float
    t_end:      float

def build_groups(horizon_s, window_s, channels) -> list[Group]
def group_count(groups) -> int
```

约束：group_id 在固定配置下完全确定（同 config 多次调用必须返回完全相同的列表）。D=40 时 group_id 范围 [0, 39]，按 `(window_id, channel)` lexicographic 排列。

### 5.2 `input_model.py`

```python
def zero_theta(groups) -> np.ndarray            # shape (D,)
def project_theta(theta, config) -> np.ndarray  # 投影回可行集
def perturb_group(theta, group_id, delta, sign, config) -> np.ndarray
def theta_to_sequence(theta, groups, config) -> pd.DataFrame
```

`project_theta` 步骤：先 clip 到 `[min_value, max_value]`，再按 window_id 顺序前向扫描应用 `max_delta_per_window`（相邻 window 同一 channel 的值差不超过该阈值）。

`theta_to_sequence` 返回 DataFrame：

```text
t_s, roll, pitch, yaw, throttle
0.00, 0.0, 0.0, 0.0, 0.0
0.02, 0.0, 0.0, 0.0, 0.0
...
```

时间步长按 `1.0 / manual_control_hz` 物理时间生成，覆盖 `[0, horizon_s + neutral_tail_s]`，最后 `neutral_tail_s` 段所有列为 0。

**验收**：`pytest tests/test_input_model.py` 至少检查 zero_theta 全 0、perturb_group 仅改一个 group、project 后无越界、相邻窗口差合法、neutral tail 全 0。

### 5.3 `vehicle/base.py`

```python
class VehicleAdapter(ABC):
    @abstractmethod
    def prepare(self, scenario: ScenarioCfg, seed: int) -> None:
        """启动 / reset simulator，等待稳态悬停。"""

    @abstractmethod
    def run(self, input_sequence: pd.DataFrame, scenario: ScenarioCfg, output_dir: Path) -> Path:
        """执行 input_sequence，返回 raw log 路径。"""

    @abstractmethod
    def parse_log(self, raw_log_path: Path) -> pd.DataFrame:
        """转换成统一 schema。"""

    @abstractmethod
    def shutdown(self) -> None:
        """彻底清理子进程。"""
```

统一 parsed log schema（columns）：

```text
time_s, x_m, y_m, z_m, alt_m, vx_mps, vy_mps, vz_mps,
roll_rad, pitch_rad, yaw_rad, mode, t_zero_s, t_neutral_s
```

`t_zero_s` 和 `t_neutral_s` 是标量（每行重复），由 adapter 写入用于 property 计算。坐标系统一用 ENU（PX4 内部 NED 由 adapter 翻成 ENU）。

### 5.4 `vehicle/synthetic.py`

```python
class SyntheticAdapter(VehicleAdapter):
    """f(theta) = simulator-free response，用 known sparse Jacobian 生成 parsed log。"""
```

输入 → 输出映射：

```text
true_jacobian: 一个稀疏矩阵 J of shape (T, D)，预生成时设
  J[t, i] = nonzero only for ~5 选定的 (i, t) 对
parsed_log.pos_xy(t) = J[t, :] @ theta + small_noise(seed)
parsed_log.alt(t)    = J_alt[t, :] @ theta + small_noise(seed)
```

Phase 0 用它跑 FD snapshot，验证 `support_recall@k(estimated, true_support) >= 0.85`。如果失败说明 FD 实现、projection、metrics 三者有 bug，**必须在碰真仿真前修好**。

### 5.5 `vehicle/px4.py` 和 `vehicle/ardupilot.py`

各自包装现有环境。具体启动命令由 agent 根据当前 AGX Orin 上的 PX4 / ArduPilot 安装结构决定，但必须满足：

- `prepare` 完成时飞机已稳定悬停，`t_zero_s` 已确定
- `run` 严格按墙钟时间发送 MANUAL_CONTROL，墙钟频率 = `manual_control_hz * sim_speed_factor`
- ULog（PX4）或 .bin（ArduPilot）必须可解析，且包含 `manual_control_setpoint` topic 用于回放验证
- `shutdown` 必须 kill 所有相关子进程，不留 zombie

### 5.6 `properties.py`

```python
def compute_robustness(parsed_log, property_name, config) -> float
def compute_all_properties(parsed_log, property_names, config) -> dict[str, float]
```

`neutral_start` 从 parsed_log 的 `t_neutral_s` 列读取，不要硬编码 5.0。属性计算前先按 `t >= t_neutral_s` 切片。

**验收**：`pytest tests/test_properties.py`，并在一次 nominal smoke run 上打印三条 robustness，必须全部为有限数；正常情况下 nominal（zero theta）的三条 robustness 都应为正且接近上限。

### 5.7 `query.py`

```python
@dataclass
class QueryResult:
    query_id:   str
    theta_hash: str
    robustness: dict[str, float]
    parsed_log_path: Path
    metadata:   dict

def run_query(theta, scenario, seed, query_type, output_dir, config) -> QueryResult
```

每次 query 流程：

1. `project_theta`

2. 计算 `theta_hash`，若 `runs/.../queries/{theta_hash}_{scenario_id}_{seed}` 已存在则直接读缓存返回

3. 生成 input sequence

4. 调用 adapter 的 prepare → run → parse_log

5. 计算所有 property 的 robustness

6. 写入：

   ```text
   queries/{theta_hash}_{scenario_id}_{seed}/
     input_theta.npy
     input_sequence.csv
     raw_log.{ulg|bin}
     parsed_log.parquet
     robustness.json
     metadata.json
   ```

7. append 一行到 `runs/.../logs/queries.jsonl`

**验收**：同一 (theta, scenario, seed) 第二次调用必须命中缓存，墙钟时间 < 1 秒。

### 5.8 `gradients.py`

```python
def finite_difference_snapshot(theta, scenario, seed, config, output_dir) -> GradientSnapshot
```

算法（two-sided，per group）：

```text
for i in range(D):
    theta_plus  = perturb_group(theta, i, +perturb_delta, +1, config)
    theta_minus = perturb_group(theta, i, +perturb_delta, -1, config)
    r_plus  = run_query(theta_plus,  scenario, seed, "fd_plus")
    r_minus = run_query(theta_minus, scenario, seed, "fd_minus")
    for p in scenario.properties:
        g[p][i] = (r_plus.robustness[p] - r_minus.robustness[p]) / (2 * perturb_delta)
```

输出：

```text
snapshots/{snapshot_id}/
  theta.npy
  groups.csv
  gradient_{property}.csv   # group_id, channel, window_id, t_start, t_end, g, abs_g
  snapshot_metadata.json
```

D=40 时一次完整 snapshot 是 80 个 query（two-sided）。跨 property 共享 query → 实际仿真次数 = 80。

`gradients.py` 内必须留出 `def cheap_sparse_snapshot(...)` 的 stub，但函数体只 `raise NotImplementedError("cheap sparse probing belongs to a later RQ")`。

### 5.9 `support.py` 与 `metrics.py`

```python
# support.py
def topk_support(abs_g, k) -> set[int]
def alpha_support(abs_g, alpha) -> set[int]   # 取 abs_g >= alpha * max(abs_g)

# metrics.py
def topk_coverage(abs_g, k) -> float
def effective_sparsity(g) -> float            # (||g||_1)^2 / (||g||_2)^2
def normalized_entropy(abs_g) -> float        # entropy(abs_g / sum) / log(D)
def jaccard(a: set, b: set) -> float
def mass_overlap(s_old: set, abs_g_new) -> float
```

**验收**：`pytest tests/test_metrics.py`，覆盖边界 case（全 0 gradient、单点 gradient、均匀 gradient）。

### 5.10 `plots.py`

```python
def plot_gradient_heatmap(gradient_csv, output_png)
def plot_topk_coverage_curve(coverage_table, output_png)
def plot_jaccard_grid(jaccard_table, output_png)
def plot_mass_overlap_curve(persistence_table, output_png)
```

Heatmap：x=window_id (0..9), y=channel (roll/pitch/yaw/throttle), color=`abs_g`。

本阶段用 matplotlib 默认风格 + 合理 colorbar 即可，**不要花时间美化**。

---

## 6. Phase 0 — Synthetic sanity（交付 2）

### 6.1 目标

证明 FD pipeline、projection、oracle、metrics 实现正确。**这一步全过之前，绝对不要碰真实仿真器。** 真实仿真器的非决定性会让 debug 难度成倍上升。

### 6.2 任务清单

| 序号  | 任务                                                         |
| ----- | ------------------------------------------------------------ |
| 6.2.1 | 实现 `config.py`、`groups.py`、`input_model.py`、`metrics.py`、`support.py` |
| 6.2.2 | 实现 `vehicle/base.py`、`vehicle/synthetic.py`               |
| 6.2.3 | 实现 `properties.py`                                         |
| 6.2.4 | 实现 `gradients.py:finite_difference_snapshot`               |
| 6.2.5 | 实现 `gradients.py:cheap_sparse_snapshot` stub（raise NotImplementedError） |
| 6.2.6 | 实现 `runners/sanity.py`（配置打印 + dry-run）               |
| 6.2.7 | 实现单元测试 `test_input_model.py`、`test_properties.py`、`test_metrics.py`、`test_fd_on_synthetic.py` |

### 6.3 命令

```bash
python -m sparsepilot.runners.sanity --config configs/synthetic_sanity.yaml --dry-run
pytest tests/ -v
```

### 6.4 验收

- `--dry-run` 打印 scenarios、D=40、property 列表，无异常退出
- 四个测试文件全部通过
- 在合成 5-sparse Jacobian 上：`support_recall@5(fd_top5, true_support) >= 0.85` 且 `mass_overlap(true_support, abs_g_fd) >= 0.7`
- 加 5% 噪声重跑：两个指标都 ≥ 0.5

### 6.5 ⏸ 停下并报告

向作者交付：

1. `pytest -v` 完整输出
2. `support_recall@5` 和 `mass_overlap` 数值（clean 和 noisy 两组）
3. 实现中做过的 judgement call（例如 projection 的具体扫描方向、LASSO 不会出现在本阶段）
4. 是否有任何无法满足的接口约束

**作者确认通过后再进入 Phase 1。** 如果验收失败：调试 FD 公式、projection 是否破坏单 group 扰动、metrics 计算。

---

## 7. Phase 1 — Real simulator smoke test（交付 3）

### 7.1 目标

让 PX4 和 ArduPilot 适配器从启动到日志解析的全链路跑通；暴露所有 simulator 工程坑（端口、加速比、mode 切换、ULog topic 缺失）。本阶段预计占整个 RQ1 工程量的 30–40%。

### 7.2 任务清单

| 序号  | 任务                                                         |
| ----- | ------------------------------------------------------------ |
| 7.2.1 | 实现 `vehicle/px4.py`（启动 SITL+jMAVSim、MAVLink 连接、prepare、run、parse_log、shutdown） |
| 7.2.2 | 实现 `vehicle/ardupilot.py`（启动 AP SITL、其余同上）        |
| 7.2.3 | 实现 `scripts/kill_sim.sh`（彻底清理 SITL/jMAVSim/AP 残留进程） |
| 7.2.4 | 实现 `query.py` 含缓存                                       |
| 7.2.5 | 实现 `runners/smoke.py`：对一个 scenario 跑 3 次 zero-theta query |

### 7.3 命令

```bash
python -m sparsepilot.runners.smoke --config configs/rq1_minimal.yaml --scenario px4_position --seed 0
python -m sparsepilot.runners.smoke --config configs/rq1_minimal.yaml --scenario px4_hold     --seed 0
python -m sparsepilot.runners.smoke --config configs/rq1_minimal.yaml --scenario ap_loiter    --seed 0
python -m sparsepilot.runners.smoke --config configs/rq1_minimal.yaml --scenario ap_althold   --seed 0
```

### 7.4 验收

对每个 scenario：

1. 3 次 zero-theta query 的 robustness 数值的相对标准差 < 5%（simulator 决定性足够）
2. ULog/parsed_log 里 `manual_control_setpoint` 列在 [0, 5s) 是非零变化的、在 [5s, 13s] 是 0（注入真的成功）
3. `mode` 列在 `t_neutral_s` 之后等于 observe_mode（mode switch 真的成功）
4. nominal robustness 三条都为正且接近上限（飞机没有意外漂移）
5. 同 (theta, scenario, seed) 第二次调用 < 1 秒（缓存命中）

### 7.5 ⏸ 停下并报告

向作者交付：

1. 4 个 scenario 各 3 次 nominal query 的 robustness 数值表（用于估计 simulator 噪声）
2. 每个 scenario 的 `manual_control_setpoint` 和 `mode` 列的可视化（confirm 注入和切换都成功）
3. 单次仿真平均墙钟时间（用于估计 Phase 2/3 总时长）
4. 此阶段遇到的所有 simulator 坑、如何解决、是否调整了 `sim_speed_factor`

**作者确认通过后再进入 Phase 2。**

---

## 8. Phase 2 — FD snapshot grid（交付 4）

### 8.1 目标

对所有 (scenario × property × seed) 在 zero theta 上跑 reference FD snapshot，回答 SPH 的 *Sparse* 和 *Mode-conditioned* 两条主张。

### 8.2 任务清单

| 序号  | 任务                                                 |
| ----- | ---------------------------------------------------- |
| 8.2.1 | 实现 `plots.py:plot_gradient_heatmap`                |
| 8.2.2 | 实现 `runners/fd_snapshot.py`（支持 --all 跑全网格） |
| 8.2.3 | 每完成一个 snapshot 立即出 heatmap                   |

### 8.3 命令

```bash
python -m sparsepilot.runners.fd_snapshot --config configs/rq1_minimal.yaml --all --theta zero
```

### 8.4 预算估算

```text
4 scenarios × 3 seeds × 80 queries/snapshot = 960 queries
单次墙钟 ≈ 15 s（含 prepare 开销）
总墙钟 ≈ 4 小时
```

如果总墙钟超过预估 50%，先停下来检查（很可能是单次墙钟超 20s，意味着 prepare 没优化或 sim_speedup 没生效）。

如果时间不够，降级策略：先跑 2 seeds × 4 scenarios，确认形状再补第三个 seed。

### 8.5 产物

```text
runs/rq1_minimal_v0/snapshots/{scenario}_seed{s}/
  gradient_{property}.csv
  snapshot_metadata.json
runs/rq1_minimal_v0/figures/
  heatmap_{scenario}_{property}_seed{s}.png   # 32 张（AltHold 少一个 property）
```

### 8.6 验收

- 32 张 heatmap 全部生成（4 scenarios × 3 properties × 3 seeds，AltHold 减掉 3 张 = 27 + 9 = 32... 实际 px4_position/px4_hold/ap_loiter 各 9 张 + ap_althold 6 张 = 33 张，按实际产出）
- 32 张里大多数应当呈现"少数几个 group 显著亮"的形态而不是均匀噪声
- `runs/.../logs/queries.jsonl` 包含所有 query 元数据

### 8.7 ⏸ 停下并报告

向作者交付：

1. 全部 heatmap 图（打包或挑选代表性几张随报告附上）
2. 每个 (scenario, property, seed) 的 `top4_coverage`、`top8_coverage`、`effective_sparsity` 三个数值
3. 同 scenario 跨 seed 的 support Jaccard / MassOverlap 初步对比（用于 mode-conditioned 判定）
4. 异常情况：哪些 heatmap 看起来是噪声？是否怀疑 SPH 在某些 case 上不成立？

**关键决策点**：如果绝大多数 heatmap 看起来是均匀噪声，**SPH 不成立**，必须停下来和作者讨论是否要重新评估方法方向，不要自行死调参试图通过。

---

## 9. Phase 3 — Path persistence（交付 5）

### 9.1 目标

回答 SPH 的 *Persistent* 主张：沿小步局部梯度更新路径，连续两步的 support 是否保持重叠。

### 9.2 任务清单

| 序号  | 任务                               |
| ----- | ---------------------------------- |
| 9.2.1 | 实现 `runners/path_persistence.py` |
| 9.2.2 | path 算法（见 §9.3）               |
| 9.2.3 | 每步重新做 full FD snapshot 并保存 |

### 9.3 算法

```text
theta_0 = zero_theta
for r in 0..4:
    snap_r = finite_difference_snapshot(theta_r, scenario, seed)
    g       = snap_r.gradient[property]
    S_top3  = topk_support(abs(g), top_m_update_groups=3)
    direction = zeros(D)
    direction[S_top3] = sign(g[S_top3])
    theta_{r+1} = project_theta(theta_r - path_eta * direction)
```

### 9.4 命令与降本

```bash
python -m sparsepilot.runners.path_persistence --config configs/rq1_minimal.yaml --all
```

**预算估算**：

```text
4 scenarios × 3 seeds × 3 properties × 5 steps × 80 queries/step = 14400 queries
理论时间 ≈ 60 小时
```

**降本手段**：

1. **跨 property 复用 query**：同一 (scenario, seed, step) 的 80 个 query 在所有 property 间共享 → 仿真次数降到 4 × 3 × 5 × 80 = 4800
2. **从 2 seeds 起步**：先跑 seed 0、1，确认曲线形状再补 seed 2
3. **path 步数先跑 3 步**：曲线初步形状清楚后再补到 5 步

实际目标：2 seeds × 3 steps 跑通 ~12 小时，全量 3 seeds × 5 steps ~30 小时。**先跑降本版本交付，再决定要不要补全量。**

### 9.5 产物

```text
runs/rq1_minimal_v0/paths/{scenario}_seed{s}/
  step_{r}/gradient_{property}.csv
  step_{r}/theta.npy
  path_summary.json
```

### 9.6 验收

- 所有 path 跑完（按降本版本算）
- 每条 path 的 robustness 沿 r 单调或近单调下降（证明步长真的在沿梯度走，不是随机漂）
- 连续两步的 support 至少有部分重叠

### 9.7 ⏸ 停下并报告

向作者交付：

1. 每条 path 的 robustness vs r 曲线（验证下降趋势）
2. 每条 path 的 (jaccard_top4, jaccard_top8, mass_overlap_top4, mass_overlap_top8) per (r, r+1) 数值
3. 是否观察到 SPH persistent 主张不成立的 case
4. 实际墙钟时间，建议是否补全量

**作者确认通过后再进入 Phase 4。**

---

## 10. Phase 4 — Analysis（交付 6，最终交付）

### 10.1 目标

汇总 Phase 2 和 Phase 3 数据，输出三张 csv + 三张主图 + 完整 README，作为 RQ1 的最终结论材料。

### 10.2 任务清单

| 序号   | 任务                                                         |
| ------ | ------------------------------------------------------------ |
| 10.2.1 | 实现 `runners/analyze.py`（聚合所有 snapshot 数据，产出三张 csv 和三张图） |
| 10.2.2 | 实现 `plots.py:plot_topk_coverage_curve`、`plot_jaccard_grid`、`plot_mass_overlap_curve` |
| 10.2.3 | 完成 README.md（含怎么从 0 跑到 Phase 4 的步骤、单次仿真大约多久、已知失败模式） |

### 10.3 命令

```bash
python -m sparsepilot.runners.analyze --config configs/rq1_minimal.yaml --run-dir runs/rq1_minimal_v0
```

### 10.4 产物

```text
runs/rq1_minimal_v0/summary/
  rq1_sparsity.csv          # 每个 snapshot 一行
  rq1_mode_conditioned.csv  # 每对 scenario 一行
  rq1_persistence.csv       # 每个 (path, step_r, step_r+1) 一行

runs/rq1_minimal_v0/figures/
  fig_sparsity_topk_coverage.png
  fig_mode_conditioned_jaccard.png
  fig_persistence_mass_overlap.png
```

csv schema 见 §11。

### 10.5 验收

见 §14 完整交付清单。

### 10.6 ⏸ 最终交付

向作者交付：

1. 三张 csv 文件
2. 三张主图
3. 完整 README
4. 一段对应 §11 三条主张是否成立的初步结论文字（不超过 200 字）

---

## 11. RQ1 判定规则

### 11.1 Sparse

`rq1_sparsity.csv` schema：

```text
scenario_id, property, seed, theta_origin, top4_coverage, top8_coverage, effective_sparsity, normalized_entropy
```

`theta_origin` 取值 `zero`（Phase 2 single-point snapshot）或 `path_step_{r}`（Phase 3 沿路径的 snapshot）。

**支持 sparse 的证据**：

- `top4_coverage` 中位数 ≥ 0.5，`top8_coverage` 中位数 ≥ 0.75
- `effective_sparsity` 中位数 ≤ 15（D=40，越小越稀疏）
- 视觉上 heatmap 出现明显热点

**不支持的迹象**：`top8_coverage` < 0.4，heatmap 接近均匀。这种情况下要检查 `perturb_delta` 是不是太小（信号未出）或太大（飞出局部线性区）。

### 11.2 Mode-conditioned

`rq1_mode_conditioned.csv` schema：

```text
property, seed, scenario_a, scenario_b, same_platform, same_mode, jaccard_top4, jaccard_top8, mass_overlap
```

对每个 (property, seed)，枚举 scenario 对 (a, b)，标记 `same_mode`（两者 observe_mode 相同）。

**支持 mode-conditioned 的证据**：

- 同 scenario 不同 seed 的 Jaccard 中位数 > 0.5
- 跨 platform 的 Jaccard 中位数 < 0.3
- 同 mode（即同 scenario 不同 seed）的 mass_overlap 中位数显著高于跨 scenario 的 mass_overlap

**关键对比**：`(px4_position, px4_hold)` vs `(px4_position, ap_loiter)`。前者同 platform 跨 mode，后者跨 platform。如果两者 overlap 接近，说明 "mode-conditioned" 是 platform-conditioned，需要更细的对比设计。

### 11.3 Persistent

`rq1_persistence.csv` schema：

```text
scenario_id, property, seed, step_r, step_next, jaccard_top4, jaccard_top8, mass_overlap_top4, mass_overlap_top8, robustness_r, robustness_next
```

**支持 persistent 的证据**：

- 连续两步的 `mass_overlap_top8` 中位数 ≥ 0.6
- `jaccard_top8` 中位数 ≥ 0.4
- robustness 沿路径单调或近单调下降

**优先使用 MassOverlap 而不是 Jaccard**：support 边缘的一两个 group 进出会让 Jaccard 抖动剧烈，但 MassOverlap 衡量的是"旧 support 是否还能覆盖新梯度的主要质量"，更稳健也更直接对应 SPH 的实用含义。

### 11.4 最终判定

三句话同时成立 → RQ1 通过，paper 主张有支撑：

> **Sparse**: Across mode-property pairs, top-k channel-time groups capture most of the robustness-gradient mass.
> **Mode-conditioned**: Supports differ significantly across modes/platforms; supports within the same scenario across seeds are stable.
> **Persistent**: Along local update paths, the previous support continues to cover a large portion of the next-step gradient mass.

任一条不成立 → 停下来评估，**不要硬调到通过**。可能需要：换 scenario、换 property、调 perturb_delta、缩小 path_eta、增加 seed 取平均、或承认 SPH 在某些 case 上不成立并据此调整 paper claim。

---

## 12. 硬性"不做"清单

- 不实现 cheap sparse probing（`gradients.py:cheap_sparse_snapshot` 留 stub raise NotImplementedError）
- 不实现 Route A / property-weighted output sensitivity
- 不实现 persistence-gated re-estimation、active-set search、delta debugging、SparseShield
- 不写任何 baseline（PGFuzz / SAASBO / GraCe / SPSA / GA / CMA-ES）
- 不调整 PID / firmware / sensor model
- 不用 Gazebo（性能开销不必要；只用 jMAVSim 或 AP SITL 自带物理）
- 不做 STL softmin/softmax smoothing（本批 property 全连续）
- 不做并行多 simulator 实例
- 不做美图、不做 dashboard、不集成 wandb/mlflow
- 不写 docker、k8s、CI 配置

Agent 如果觉得某项必要，**先问，不要自己加**。

---

## 13. 常见坑位与排查

| 现象                                          | 原因                                                    | 排查                                                         |
| --------------------------------------------- | ------------------------------------------------------- | ------------------------------------------------------------ |
| `manual_control_setpoint` ULog 中是平的       | MAVLink 发送频率没按 sim_speedup 换算                   | 检查 px4.py 中是否乘了 sim_speed_factor                      |
| 同 theta 3 次 robustness 差异 > 10%           | sim_speedup 太大 / 调度噪声                             | sim_speed_factor 调低到 3，确认稳定再试 5                    |
| `mode` 列没切到 Hold                          | mode_id 枚举错                                          | 从当前 PX4 源码读 enum，不要抄网上的旧值                     |
| nominal robustness 是负的                     | takeoff 没稳就开始扰动 / takeoff_alt 太低导致地面效应   | 拉长 prepare 等待时间、确认 t_zero 的判定                    |
| jMAVSim 启动后僵死                            | Java 残留 / 端口占用                                    | `kill_sim.sh` 必须 `pkill -f jmavsim` + `pkill -f px4` + 清理 `/tmp/px4-*` |
| AP SITL 没 takeoff                            | mode set 时机太早，飞机还未 armed                       | prepare 流程加 arm 等待 + GPS lock 等待                      |
| project_theta 把单 group 扰动平摊到多个 group | max_delta_per_window 太小                               | perturb_delta 必须 < max_delta_per_window，目前 0.08 < 0.25 OK |
| Phase 2 跨 seed 不复用 cache                  | seed 影响 prepare（初始状态），theta_hash 不应包含 seed | metadata 里写 seed，cache key = (theta_hash, scenario_id, seed) |
| LASSO 报错或没收敛                            | 还没到本阶段对应的 RQ                                   | 本阶段不应触发；若触发说明 agent 越界做了 cheap sparse       |

---

## 14. 最终交付清单

Phase 4 完成时，仓库必须有：

```text
[ ] configs/rq1_minimal.yaml, configs/synthetic_sanity.yaml
[ ] src/sparsepilot/ 所有模块按 §5 签名实现
[ ] tests/ 四个测试文件全过
[ ] runs/rq1_minimal_v0/queries/ 含所有 query 缓存
[ ] runs/rq1_minimal_v0/snapshots/ 含 Phase 2 全部 snapshot
[ ] runs/rq1_minimal_v0/paths/ 含 Phase 3 全部 path
[ ] runs/rq1_minimal_v0/summary/ 含三张 csv
[ ] runs/rq1_minimal_v0/figures/ 含 Phase 2 heatmaps + Phase 4 三张主图
[ ] runs/rq1_minimal_v0/logs/queries.jsonl 包含所有 query 元数据
[ ] README.md 含怎么从 0 跑到 Phase 4 的步骤、单次仿真大约多久、已知失败模式
[ ] 所有 cheap_sparse / route_a / delta_debug / shield 入口都是 NotImplementedError stub
[ ] §10.6 中的一段不超过 200 字的初步结论
```

完成所有项目后，向作者发送最终交付报告。
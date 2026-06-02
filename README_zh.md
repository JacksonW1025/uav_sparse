# uav_sparse / CADET

中文说明。英文版见 [README.md](README.md)。

本仓库是 **CADET** 的研究代码仓：
**C**ontrol-**A**llocation-**D**irected **E**xploration of
**T**riggers，中文可称为“控制分配导向的触发探索”。

CADET 研究的是无人机飞控中的一类 bug：**可行但不安全的飞手输入**。
也就是说，飞手给出的摇杆序列是合法的、受速率限制的、不是明显自毁的满杆操作；
但飞手把摇杆回中之后，飞行器仍然违反某个飞行模式安全契约。仓库的 Python
包名是 `cadet`；仓库名仍保持为 `uav_sparse`。

谱系上，CADET 是 RouthSearch 风格工作的后继：RouthSearch 用原则性的稳定性
判据结构化 PID 参数搜索；CADET 固定 PID、传感器、固件、混控器和任务，改用
控制分配结构来组织飞手输入搜索。

## 当前状态

这是一个研究测量 pipeline，不是整理完毕的 benchmark artifact。

当前最强证据来自 **PX4 `px4_position` / POSCTL，seed 0**。`docs/`
下四份文档记录了从 SPH 到 CADET 的完整演化：全局稀疏梯度失败，边界热启动失败，
模式切换 bug 没有出现，但控制通道导向的最小触发合成站住了。

阅读和使用本仓库时要注意：

- 文档中的定量数字来自单种子本地实验。
- `artifacts/` 包含 seed-0 小体积 report/theta 产物，可用于审计当前数字。
  `runs/` 仍被忽略，因为其中是体积很大的原始日志和 per-query 缓存。
- ArduPilot adapter 已存在，但 CADET 还没有在 ArduPilot 上完成验证。
- Direction-A / CADET 核心 runner 目前在代码里冻结为 `px4_position`、seed `0`。

四份研究文档是仓库语境的一部分：

- `docs/01_Research_Narrative_CADET.md`
- `docs/02_CADET_Method.md`
- `docs/03_Paper_Outline_ISSTA_CADET.md`
- `docs/CADET_工作全景梳理.md`

## 研究问题

给定一个固定的飞控配置：

- 固定 PID 增益、传感器、固件、混控器和任务；
- 一个飞行模式，例如 PX4 POSCTL；
- 一个用 STL 风格鲁棒度 oracle 表达的安全契约；

CADET 在飞手输入空间里搜索触发机动 `U`，要求：

- `U` 可行：摇杆幅值有界，窗口间变化率有界；
- `U` 非平凡：内部、非饱和，而不是满杆滥用；
- 摇杆会回到中位；
- 飞行器仍然在回中后违反安全契约。

输入模型是一个 40 维向量：

- 10 个时间窗口；
- 每个窗口 0.5 s；
- 4 个通道：`roll`、`pitch`、`yaw`、`throttle`；
- 5 s 主动输入时域；
- 随后接 8 s 回中尾段。

当前实现的契约在 `src/cadet/properties.py`：

| 属性 | 鲁棒度 |
| --- | --- |
| `post_neutral_xy_drift` | `2.0 m - 最大水平漂移` |
| `post_neutral_alt_drift` | `1.0 m - 最大高度漂移` |
| `post_neutral_xy_velocity` | `1.0 m/s - 最大水平速度` |

鲁棒度 `rho > 0` 表示安全，`rho < 0` 表示违例。边界判断使用重复仿真和
2-sigma 规则：鲁棒违例要求 `rho_mean + 2 * rho_std < 0`。

## CADET 的主张

CADET 的主张很窄，也正因为窄才可信：

> 针对“可行但不安全的飞手输入”这类 bug，控制分配导向的搜索可以可靠地产生
> 小支撑、人类可读、单/双通道、回中后的触发机动；这些触发是随机搜索会错过、
> 通道无关搜索加 delta-debugging 也无法可靠恢复的。

CADET **不**主张：

- 更快找到任意违例；
- 一定比所有通道无关 baseline 得到更低的峰值摇杆幅值；
- 存在 viability theory 或 cell-sparse 边界保证；
- 可以检测模式切换交接 bug；
- 在补齐实验前，已经具备跨种子或跨平台泛化性。

## 方法概要

CADET 是一个两段式方法。

1. 先不用梯度到达违例区域，因为安全区域表现为鲁棒度平台。
2. 再利用控制分配结构：对某个属性，只搜索实际驱动该受限运动的少数控制通道。

当前方法分五个阶段：

| 阶段 | 作用 |
| --- | --- |
| 0. 参数化 | 把 tick 级摇杆输入转成 40 维窗口/通道模型。 |
| 1. 到达边界 | 用非梯度采样和包夹找到鲁棒安全点与鲁棒违例点。 |
| 2. 推导活跃通道 | 将属性映射到通道，例如 `xy_velocity -> roll,pitch`，再用抗噪方向探针验证。 |
| 3. 搜索约简空间 | 只在活跃通道的 envelope 空间中搜索，并包夹内部边界。 |
| 4. 合成触发 | 轻度约简支撑和幅值，输出可读触发及其通道/时间签名。 |

核心机制是：干净的最小触发通常不能从一个稠密、浅违例中逐元删除得到。
Delta-debugging 需要鲁棒性裕度才能删除元胞；而有 bug 价值的温和违例靠近边界，
裕度很小。CADET 因此直接搜索通道约简空间。

## 仓库结构

| 路径 | 作用 |
| --- | --- |
| `configs/` | 实验配置。`rq1_minimal.yaml` 是主 PX4/AP 配置；`synthetic_sanity.yaml` 用于离线检查。 |
| `src/cadet/input_model.py` | 摇杆限位、速率限位投影，以及 theta 到时间序列的转换。 |
| `src/cadet/groups.py` | 40 个通道-时间 group。 |
| `src/cadet/properties.py` | 回中后的鲁棒度契约。 |
| `src/cadet/query.py` | 查询执行、缓存、日志和 simulator adapter 分发。 |
| `src/cadet/vehicle/` | PX4、ArduPilot 和 synthetic simulator adapter。 |
| `src/cadet/violation_search.py` | 结构无关的粗违例边界搜索。 |
| `src/cadet/runners/` | 历史阶段和当前实验 runner。 |
| `artifacts/` | seed-0 CADET 脊柱 run 的小体积 CSV/JSON/theta 产物。 |
| `tests/` | 输入模型、度量、属性、synthetic FD、H3、Direction-A 逻辑测试。 |
| `scripts/start_px4.sh` | 启动 PX4 jMAVSim SITL。 |
| `scripts/kill_sim.sh` | 清理 PX4 / ArduPilot simulator 进程。 |
| `docs/` | 研究叙事、方法规范、论文大纲和代码到主张的对应说明。 |

## 主要实验 Runner

| 主张 / 阶段 | Runner | 备注 |
| --- | --- | --- |
| 安全区域是平台，梯度主要是噪声 | `cadet.runners.repeated_fd`, `cadet.runners.persistence_pilot` | SPH 阶段有限差分测量。 |
| 边界敏感度是通道各向异性的 | `cadet.runners.margin_stage1_redo` | 使用更大的方向探针步长 `delta=0.2`；历史 runner 仍需要早期 boundary/stage run。 |
| 跨条件热启动不节省查询 | `cadet.runners.route1_h2_campaign` | H2 负面结果；默认 Point V anchor 已归档到 `artifacts/`。 |
| POSCTL -> LOITER 交接是干净的 | `cadet.runners.h3_transition` | H3 负面结果；prior theta 候选已归档到 `artifacts/`。 |
| 随机、通道无关、通道导向三臂探针 | `cadet.runners.direction_a_probe` | Direction-A / CADET 核心证据；预注册 J=5。 |
| delta-debugging 必要性 baseline | `cadet.runners.direction_a_ddmin` | 需要把 Direction-A probe 输出作为 `--probe-dir`。 |

文档中记录的单种子 Direction-A 关键数字：

| 臂 | 搜索方式 | 内部违例数 | 触发形态 |
| --- | --- | ---: | --- |
| A | 均匀随机 | 0 | 只能找到饱和/平凡违例。 |
| B | 通道无关内部包夹 | 7 | 峰值幅值可能更低，但稠密且四通道全活跃。 |
| C | roll/pitch 通道导向搜索 | 18 | 支撑 4-8，只含 roll/pitch，可读性强。 |

ddmin baseline 只有 `4/10` 个起点被最小化成干净触发；最终支撑中位数仍是
`14.5`，而通道导向触发约为 `6`。

## 安装

需要 Python 3.10+。

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[test]"
```

如果直接使用仓库已有的本地虚拟环境：

```bash
source .venv/bin/activate
```

PX4 SITL 默认路径是 `/home/car/PX4-Autopilot`。如果你的 PX4 checkout
在其他位置，设置 `PX4_ROOT`：

```bash
PX4_ROOT=/path/to/PX4-Autopilot scripts/start_px4.sh
```

清理 simulator 进程：

```bash
scripts/kill_sim.sh
```

## 快速检查

配置和参数化 dry run：

```bash
python -m cadet.runners.sanity \
  --config configs/synthetic_sanity.yaml \
  --dry-run
```

运行测试。带 ROS 的机器上建议禁用第三方 pytest 插件自动加载，否则
`launch_testing_ros` 可能打断测试收集。

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

当前 workspace 已验证：

```text
31 passed in 22.80s
```

## 运行 PX4 实验

在一个终端启动 PX4 SITL：

```bash
PX4_ROOT=/path/to/PX4-Autopilot scripts/start_px4.sh
```

在另一个终端运行 PX4 smoke check：

```bash
python -m cadet.runners.smoke \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --skip-probes
```

运行当前 Direction-A / CADET 三臂探针：

```bash
python -m cadet.runners.direction_a_probe \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/direction_a_px4_position_seed0_v0
```

然后用该 probe 输出运行 ddmin 必要性 baseline：

```bash
python -m cadet.runners.direction_a_ddmin \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --probe-dir runs/direction_a_px4_position_seed0_v0 \
  --run-dir runs/direction_a_ddmin_px4_position_seed0_v1
```

运行结构无关的粗边界搜索：

```bash
python -m cadet.violation_search \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/rq1_boundary_v0
```

运行历史 H1 边界各向异性链：

```bash
python -m cadet.runners.margin_stage0 \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage0_v1

python -m cadet.runners.margin_stage1 \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_v1

python -m cadet.runners.margin_stage1_redo \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_redo_v1
```

从归档的 Point V anchor 运行 H2：

```bash
python -m cadet.runners.route1_h2_campaign \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --theta-v artifacts/margin_stage1_redo_v1/theta_V.npy \
  --run-dir runs/route1_h2_px4_position_seed0_vmax_pilot_v0
```

从归档 prior theta 候选运行 H3：

```bash
python -m cadet.runners.h3_transition \
  --config configs/rq1_minimal.yaml \
  --seed 0 \
  --run-dir runs/h3_transition_seed0_v1
```

历史 H1 命令仍需要前置 run，因为 refined candidate `.npz` 和部分 FD snapshot
有意不纳入 Git。从头重跑 campaign 前，先查看对应 `--help` 并显式传入路径。

## 输出产物

实验输出写入 `runs/`：

- `queries/*/input_theta.npy`
- `queries/*/input_sequence.csv`
- `queries/*/robustness.json`
- `queries/*/metadata.json`
- `reports/*.csv`
- `reports/*_summary.json`
- `reports/*_report.md`
- `groups.csv`

这些文件默认不提交。写论文时，应把小体积 report 和触发产物单独归档，
不要只依赖本地未提交目录：

- `reports/*.csv`
- `reports/*_summary.json`
- `reports/pre_registration.json`
- 最小触发 `theta.npy`
- 论文表格实际使用的数据

## 小体积 Artifacts

已跟踪的 `artifacts/` 目录包含 seed-0 CADET 脊柱 run 中适合进 Git 的部分：

- `artifacts/direction_a_px4_position_seed0_v0/`
- `artifacts/direction_a_ddmin_px4_position_seed0_v1/`
- `artifacts/margin_stage1_redo_v1/`
- `artifacts/rq1_boundary_v0/`
- `artifacts/margin_stage0_v1/`

每个子目录都有 README，说明对应 RQ、runner、命令、包含的 CSV/JSON/theta
文件，以及被剥离的 raw log。它们用于审计报告数字和提供默认 theta anchor；
真正重跑 PX4 仍会把新数据写到 `runs/`。

## 耗时与失败模式

本地归档 seed-0 运行中，单次 PX4/jMAVSim repeat 约 20 秒墙钟。一个 J=5
鲁棒度点约 1-2 分钟。Direction-A probe 约 6.6 小时 / 1200 次成功 repeat；
ddmin baseline 约 10.7 小时 / 1965 次 repeat；H2 与 H3 pilot 约 6.8 小时和
1.7 小时。

常见失败模式：

- ROS pytest 插件会打断测试收集；使用 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q`；
- `PX4_ROOT` 缺失或指错会导致 SITL 无法启动；
- PX4/jMAVSim 残留进程和端口占用需要 `scripts/kill_sim.sh`；
- simulator speedup 太大时，重复鲁棒度会变得过噪；
- 历史 H1 runner 缺前置 run 时会失败，需要显式传路径；
- 原始 `runs/` 数据只在本地，除非主动整理小工件进 `artifacts/`。

## 已知缺口

论文级主张之前必须补齐：

- 在至少两个额外 seed 上重跑 Direction-A probe 和 ddmin；
- 加入第二个属性，例如 `xy_drift`；
- 从控制分配原则性推导活跃通道，而不是继续使用当前对 `xy_velocity`
  写死的 `["roll", "pitch"]`；
- 在 ArduPilot 上验证 CADET；
- 按不同触发族核算代价；
- 仅使用 tracked artifacts 和文档化前置命令，从 fresh clone 重跑历史 H1。

工程 caveat：

- `configs/rq1_minimal.yaml` 使用摇杆限位 `[-0.7, 0.7]`，但 Direction-A
  probe 会覆盖为 `[-1.0, 1.0]`，以便饱和类别可达。论文和报告中必须统一或明确说明可行性口径。
- `runs/`、`*.ulg`、`*.parquet` 和 `*.npz` 被忽略。仓库会保持轻量，但主张所需产物需要有意识地整理。

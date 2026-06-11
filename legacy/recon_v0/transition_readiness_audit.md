# PX4 Position->Hold Transition 就绪审计

日期：2026-06-08

范围：只做就绪审计。未启动新的 transition 实验，未设计新的实验流程。只读取当前代码、只读检查 `/mnt/nvme/px4_work/uav_sparse` 下既有归档日志、查 PX4 官方文档，并在任何新实验数据之前做机械性 harness 修复。

## 机械修复

1. `4ffaa6cd35be4af58080ec4b1cdefc80c7b57e59` (`harness: reset PX4 default motion params per run`)
   - `PX4Adapter.prepare()` 现在每次 PX4 run 都显式设置并回读 `MPC_ACC_HOR=3.0`、`MPC_JERK_MAX=8.0`；场景级 `param_overrides` 仍可覆盖默认值。
2. `328c3abda2d4b55864fb9921b40e52dc9dbd6171` (`harness: key query cache by runtime overrides`)
   - `run_query()` cache id 现在纳入运行时 `t_switch_s` 和场景 `param_overrides`，避免不同切换时刻或参数设置误命中同一缓存。
3. `c2ca455cfa04af19676bfa4763c0d13a03607091` (`harness: log transition velocity diagnostics`)
   - parsed log 现在包含 `xy_speed_mps` 和 `velocity_at_transition_mps`。
   - metadata/jsonl 现在包含 `transition_observed_t_s`、`velocity_at_transition_mps`、请求到完成延迟，以及 `xy_speed_peak_5_7_mps`、`xy_speed_peak_7_9_mps`、`xy_speed_peak_9_11_mps`、`xy_speed_peak_11_13_mps`。

验证命令：

```text
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/pytest \
  tests/test_px4_param_overrides.py \
  tests/test_query_runtime_cache_tag.py \
  tests/test_query_telemetry_diagnostics.py \
  tests/test_terminal_window.py \
  tests/test_h3_transition.py \
  tests/test_properties.py -q

22 passed in 1.41s
```

## A. 切换机制

代码现实：

- `px4_transition` 将 `observe_mode: Hold` 映射为 PX4 `LOITER`。
- `MavlinkVehicleMixin._execute_sequence()` 每个 tick 发送 manual control。`row_t_s >= scenario.t_switch_s` 后，每 0.2 s 发送一次 `scenario.observe_mode` 模式命令，直到 `_mode_matches(last_mode, observe_mode)` 判定实际完成。
- 实际完成来自 heartbeat 派生的模式状态，不来自首次请求本身。

只读归档证据：

- Amendment 03 zero transition 日志：`5.0s` 请求，`5.5s` 观察到 `LOITER`，模式集合 `POSCTL,LOITER`。
- H3 maneuver transition 样本：`4.0s` 请求，`4.34s` 观察到 `LOITER`，模式集合 `POSCTL,LOITER`。

逐次运行切换时刻：

- H3 已通过 `dataclasses.replace(base, t_switch_s=...)` 在 `_transition_scenario()` 中逐次改变切换时刻。
- 机械 cache 阻塞已由 `328c3ab...` 修复：runtime tag 能区分 `ts4p000`、`ts5p000` 和参数覆盖 hash。

日志：

- 既有 transition parsed log 已包含 `transition_observed_t_s`。
- `c2ca455...` 补齐 metadata 中未加前缀的切换时刻、velocity-at-transition、延迟和四个绝对子窗峰值。

状态：PASS，机械修复后满足。

## B. 飞手输入与切换共存

代码现实：

- `COM_RC_IN_MODE=1` 在 PX4 prepare 中设置，transition mode command 不会覆盖它。
- `[0, t_switch]` 内 manual-control sequence 正常发送；切换命令只是从 `t_switch_s` 起额外发送的 MAVLink mode command。
- 切换后 harness 仍继续发送 manual-control 消息。对本次 Position->Hold 交接实验，theta 应在 `t_switch_s` 及之后保持中位，以避免 stick override 歧义。

重要注意：

- PX4 Hold 是 autonomous mode，但 PX4 文档也说明 RC stick movement 默认可能把 auto mode 切回 Position。因此实验不能依赖“非零 post-switch 杆量一定被忽略”。应只使用非零 support 落在 `[0, t_switch]` 的机动，并在切换后保持中位。
- 现有 H3 代码记录 `return_to_neutral_by_t_switch`，但它是诊断/分类字段，不是全局硬 gate。

状态：对本次要求的 Position 阶段建速机动模型为 PASS；不要在后续设计里引入非零 post-switch 杆量，除非另行人工决策。

## C. Oracle 对 transition 日志可用

代码现实：

- `compute_robustness(parsed_log, property, config, window=(11.0,13.0))` 在 `time_s >= t_neutral_s` 的 tail 内按绝对 `time_s` 过滤窗口。
- 窗口是绝对时间，不是相对 `t_neutral_s`；`t_neutral_s=5.0` 时 `[11,13]` 是终端窗，且与 `t_switch` 解耦。

只读归档证据：

- Amendment 03 transition zero 日志有 `t_neutral_s=5.0`，`time_s` 覆盖到 `13.0`，模式 `POSCTL,LOITER`，并含 `vx_mps/vy_mps`。
- 该日志上 `compute_robustness(..., "post_neutral_xy_velocity", window=(11.0,13.0)) = 0.966400`。
- H3 maneuver transition 样本同样含逐 tick 模式和 `vx_mps/vy_mps`；窗口 rho 为 `0.930510`。

状态：PASS。

## D. 差分归因能力

代码现实：

- 输入参数化与场景无关：`theta_to_sequence()` 用同一 theta/groups/config 生成同一 tick sequence。
- H3 已把同一 candidate theta 分别喂给 `px4_position` (NS) 与 `px4_transition` (SW)，并用相同 `theta_hash` 做 pair 分类。
- 预期唯一行为差异是 transition 场景在 `t_switch_s` 发出 Hold/LOITER 命令。

边界：

- 这只证明 theta/input plumbing 兼容；后续实验设计仍必须选择在 `t_switch_s` 后中位的机动族，才能保持纯交接归因。

状态：PASS。

## E. 参数卫生

修复前：

- `configs/rq1_minimal.yaml` 未指定 `param_overrides`。
- 旧 H3 transition metadata 缺少 `MPC_ACC_HOR/MPC_JERK_MAX` 回读键，说明 transition run 不能保证默认 reset。

修复后：

- `PX4_DEFAULT_PARAM_OVERRIDES = {"MPC_ACC_HOR": 3.0, "MPC_JERK_MAX": 8.0}` 会合并进每次 PX4 prepare。
- `_apply_param_overrides()` 对每个参数 set、readback、verify，并记录 target/readback/type/reboot-required。
- 场景级 `param_overrides` 仍支持扫描，同时未覆盖的默认项仍会显式设置。
- `cleanup_each_run: true` 仍会每 run 关闭 PX4 进程；显式 set/readback 现在防止 position/transition 交替运行时的参数持久化污染。

状态：PASS，`4ffaa6c...` 后满足。

## F. Hold 契约 anti-trap

官方来源：https://docs.px4.io/main/en/flight_modes_mc/hold

PX4 官方 Hold 页写明 Hold causes the vehicle to "stop and hover at its current GPS position and altitude".

这是直接的 stop/hover 承诺，不只是“保持当前速度/状态”。因此 `post_transition_xy_velocity -> 0` 风格契约对 MC Hold/LOITER 交接后仍有合法根据。

状态：PASS。

## G. 日志完整性

要求项与当前状态：

- `transition_observed_t_s`：transition parsed log 已有；现在 metadata/jsonl 也有无前缀字段。
- 切换时刻实际水平速度：新增 `velocity_at_transition_mps`，写入 parsed log 和 metadata。
- 模式时间线：parsed log 有逐 tick `mode`、`custom_mode`、`px4_main_mode`、`px4_sub_mode`。
- 水平速度时间线：parsed log 有 `vx_mps`、`vy_mps`；现在还派生 `xy_speed_mps`。
- 四子窗峰值：metadata/jsonl 新增 `xy_speed_peak_5_7_mps`、`xy_speed_peak_7_9_mps`、`xy_speed_peak_9_11_mps`、`xy_speed_peak_11_13_mps`。

把新诊断 helper 应用于旧日志得到的只读归档数值：

- Amendment 03 zero transition：`transition_observed_t_s=5.5`，`velocity_at_transition_mps=0.015610`，峰值 `[5,7]=0.029021`、`[7,9]=0.046425`、`[9,11]=0.037718`、`[11,13]=0.033600`。
- H3 maneuver t4 样本：`transition_observed_t_s=4.34`，`velocity_at_transition_mps=0.190492`，峰值 `[5,7]=0.094678`、`[7,9]=0.045319`、`[9,11]=0.070221`、`[11,13]=0.069490`。

状态：PASS，`c2ca455...` 后满足。

## 最终判定

READY.

未发现 D/F 硬阻塞。harness 已可进入后续 Position->Hold transition 实验 prompt；前提是实验设计把飞手机动限制在 `[0, t_switch]`，并且不依赖非零 post-switch 杆量被忽略。

STOP.

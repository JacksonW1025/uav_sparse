# CADET 检查点状态

日期：2026-06-06

本 checkpoint commit 记录的状态：探索阶段。本文只是检查点索引，不新增
confirmatory 结论。

## 核心状态

- 锚点 property：`post_neutral_xy_velocity`。
- 当前最可信读法：经 `artifacts/monotonicity_check_summary/` 的口径核对，
  已归档的锚点违例在终端窗 [11,13] s 度量下极可能为度量窗口假象。
- 口径差异：原始 `compute_robustness(..., post_neutral_xy_velocity, ...)`
  从 `t_neutral_s` 到日志结束取整段 post-neutral tail 峰值；这些 run 中
  即 [5,13] s 的 `|v_xy|` 峰值。这会抓到松杆后的刹车暂态。
- 终端窗核对：在 [11,13] s 终端度量下，被核对的 Direction-A internal
  theta 为 `robust_safe`；内部 theta 的终端残余水平速度约 0.04-0.05 m/s，
  饱和 roll/pitch 探针约 0.31-0.32 m/s，均低于 1.0 m/s 阈值。

## 已完成指针

- 跨种子三臂 + ddmin：`artifacts/seed_replication_*`、
  `artifacts/recut_distinct_v0/`、`artifacts/recut_channel_gentle_v0/`。
  已归档结论是 partial cross-seed stability：thin-target/random-miss 和
  relevant-channel directionality survive，但强 seed-0 support/sparsity
  claim 并未原样跨 seed 存活。
- bug 类重定义：回中后残余运动，当前锚点集中在
  `post_neutral_xy_velocity` / 松杆后的水平残余速度。
- 通道迁移：`artifacts/residual_rate_summary/` 显示 climb/yaw 在 Tier 1
  没有迁移；Phase 2 和 ddmin 未运行。
- RGA Step A：`artifacts/cross_coupling_summary/` 标记为
  `exploratory-hypothesis`，Step A candidates present，exact inverse，
  condition number `98.6525`。
- 契约网格 Phase-0：`artifacts/contract_grid_summary/` 标记为
  `exploratory-hypothesis`，seed 0，G01-G12，已归档 Phase-0 grid cells 和
  probe points。
- 口径核对：`artifacts/monotonicity_check_summary/` 记录 [11,13] s grid
  metric 与原始 tail-peak metric 的差异。

## 待连锁核对

- 用 [11,13] s 终端窗重判 Direction-A 三臂 + ddmin。
- 对 seed 1/2 做衰减剖面抽查。
- 随后确认 thin-target / cost advantage 在终端窗下还剩多少。

## 纪律备注

- `artifacts/cross_coupling_prereg.md` 与
  `artifacts/contract_grid_prereg.md` 均为对应运行后的事后提交，所以相关
  结论保持 `exploratory-hypothesis`。
- 本 checkpoint 不更改任何既有报告的 conclusion label、provenance label
  或数值。
- 全景梳理：[docs/CADET_工作全景梳理_v3.md](CADET_工作全景梳理_v3.md)。

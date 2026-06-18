VERDICT: DEGENERATE

# 能量逆风 P 分层延伸探针

结论：clean_unsafe@Dreached drops to zero only where prior residual cells are dominated by D_not_reached; this is early RTL trigger, not real closure.

## 拼接分层表

| BATT_LOW_MAH | capacity % | clean_safe | clean_unsafe | clean_unsafe@Dreached | D_not_reached | contract_violated | blocked |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 150 | 23.1 | 12 | 30 | 30 | 0 | 0 | 0 |
| 220 | 33.8 | 26 | 16 | 16 | 0 | 0 | 0 |
| 300 | 46.2 | 37 | 5 | 5 | 0 | 0 | 0 |
| 400 | 61.5 | 40 | 2 | 2 | 0 | 0 | 0 |
| 500 | 76.9 | 12 | 0 | 0 | 30 | 30 | 0 |
| 600 | 92.3 | 0 | 0 | 0 | 42 | 42 | 0 |

`BATT_CAPACITY=650 mAh`；`SIM_BATT_CAP_AH=0.650 Ah` (模型容量 650 mAh)。
现实性标注：运营低电阈值通常到约 20%；本探针把 >40% 的关闭视为不现实储备下的关闭，不记作 `RESIDUAL_CLOSES_REAL`。
`D_not_reached` 在本报告中计入两类场景：`achieved/commanded < 0.9`，或 DataFlash/parser 明确标出 `D_not_reached_before_low_failsafe`。后一类表示低电 RTL 在命令距离达成前触发；即使惯性或后续轨迹让最大距离超过 0.9D，也不作为真实关闭证据。

## v1 400 mAh 残留格跟踪

| old D | old wind | old shortfall m | new layer | status | label | got_home | achieved/commanded | shortfall m |
| ---: | ---: | ---: | ---: | --- | --- | --- | ---: | ---: |
| 140 | 12.0 | 6.75 | 500 | D_not_reached | contract_violated | True | 0.470 | 0.00 |
| 140 | 12.0 | 6.75 | 600 | D_not_reached | contract_violated | False | 1.023 | 127.17 |
| 160 | 12.0 | 28.94 | 500 | D_not_reached | contract_violated | True | 0.411 | 0.00 |
| 160 | 12.0 | 28.94 | 600 | D_not_reached | contract_violated | False | 1.022 | 147.57 |

## DEGENERATE 说明

新增高储备档的下降由 `D_not_reached` 驱动：低电 RTL 触发太早，远距工况没有真正施加。因此这些档不能支持“残留关得掉”的结论。

## 新档残留重复

无新增 D-reached clean_unsafe 残留格，因此没有触发残留重复。

## 后果类型

新增档 clean_unsafe consequence_type 分布：`{}`。
未出现 `uncontrolled`；本探针不改变 v1 后果偏软、以 `controlled_land_away` 为主的定性。

## 图

- pstrat_residual_curve: ![](planc/analysis/energy_headwind_pstrat_ext_pstrat_residual_curve.png)

## 审计

本探针只新增 `BATT_LOW_MAH=500/600` 两档；其它 P、D×wind 网格、harness、parser、oracle 均沿用 v1。每个新增 run 均落盘 DataFlash、param dump、parsed CSV 和 oracle sidecar。
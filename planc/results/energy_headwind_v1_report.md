VERDICT: PASS

# 能量逆风 v1：RTL 容量阈值不足

结论：All preregistered energy-headwind threshold-insufficiency checks passed.

## 五个预注册判据

- 前提满足：**True**。机制复核=True；主层最小 achieved/commanded=1.017；返航地速单调=True。
- robust clean_unsafe 区：**True**。主层 clean_unsafe=16；稳定重复边界点=3。
- 失效保护正确触发且 PGFUZZ 不可见：**True**。clean_unsafe 上坏点=0。
- BATT_LOW_MAH 分层单调收缩：**True**。clean_unsafe count is non-increasing as BATT_LOW_MAH increases。
- 宽范围边界可刻画：**True**。主层有 same-D 翻转的 D=[80.0, 100.0, 120.0, 140.0]。

## 固定配置与隔离

主层 `BATT_LOW_MAH=220`；分层 `[150.0, 220.0, 300.0, 400.0]`。固定 `BATT_FS_LOW_ACT=2`、`BATT_FS_CRT_ACT=1`、`RTL_ALT=2000`、`WPNAV_SPEED=800`、`AVOID_ENABLE=0`、`FENCE_ENABLE=0`、`SIM_WIND_DIR=270`。出航向东顺风，RTL 返航向西逆风。

## 前提复核

| check | key numbers |
| --- | --- |
| battery RTL mechanism | no-wind mechanism rows all clean/got_home=True |
| D achieved | min achieved/commanded on main layer=1.017 |
| return wind coupling | wind 0.0: 7.87 m/s; wind 6.0: 6.48 m/s; wind 12.0: 3.05 m/s |

## 主层 D x wind 标签

| D m | wind 0 | wind 3 | wind 6 | wind 7.5 | wind 9 | wind 12 |
| ---: | --- | --- | --- | --- | --- | --- |
| 40 | clean_safe | clean_safe | clean_safe | clean_safe | clean_safe | clean_safe |
| 60 | clean_safe | clean_safe | clean_safe | clean_safe | clean_safe | clean_safe |
| 80 | clean_safe | clean_safe | clean_safe | clean_safe | clean_safe | clean_unsafe |
| 100 | clean_safe | clean_safe | clean_safe | clean_safe | clean_unsafe | clean_unsafe |
| 120 | clean_safe | clean_safe | clean_safe | clean_unsafe | clean_unsafe | clean_unsafe |
| 140 | clean_safe | clean_safe | clean_unsafe | clean_unsafe | clean_unsafe | clean_unsafe |
| 160 | clean_unsafe | clean_unsafe | clean_unsafe | clean_unsafe | clean_unsafe | clean_unsafe |

## P 分层（承重项）

| BATT_LOW_MAH | clean_safe | clean_unsafe | contract_violated | D_not_reached |
| ---: | ---: | ---: | ---: | ---: |
| 150 | 12 | 30 | 0 | 0 |
| 220 | 26 | 16 | 0 | 0 |
| 300 | 37 | 5 | 0 | 0 |
| 400 | 40 | 2 | 0 | 0 |

## sigma（只报告，不设门）

本实验明确不设置 sigma 前提门，也不设置严重度回归 MAE 门；这些重复只说明标签和后果大小的稳定性。

| D | wind | n | stable | mean shortfall m | sigma m | labels |
| ---: | ---: | ---: | --- | ---: | ---: | --- |
| 80 | 12.0 | 5 | True | 10.32 | 0.423 | clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe |
| 100 | 9.0 | 5 | True | 0.44 | 0.339 | clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe |
| 120 | 9.0 | 5 | True | 20.38 | 0.345 | clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe, clean_unsafe |

## 宽范围边界刻画（非学习贡献）

主层 same-D 翻转出现在 D=[80.0, 100.0, 120.0, 140.0]。这里的边界刻画用于说明确定性和平滑移动，不作为噪声鲁棒学习/预测贡献。
搜索效率仅作次要报告：offline replay of discrete bisection along D for each BATT_LOW_MAH x wind using completed grid labels，离线二分查询 72 次，完整主网格/分层网格点 168 个。

## 后果类型

主层 clean_unsafe consequence_type 分布：`{'controlled_land_away': 16}`。
本次 clean_unsafe 主要是 `controlled_land_away`：受控迫降在非 home 位置。报告不把它夸成坠机；它是 RTL 返航承诺未兑现的 SOTIF 后果。

## threshold-insufficiency 解释

`BATT_LOW_MAH` 隐含承诺是触发 RTL 时仍有足够返航储备。本实验中电池 failsafe 和 RTL/LAND 都按规约正确触发，Tier-1 契约检查器看不到预防性违约；但在合法逆风下仍出现 clean_unsafe。随着 `BATT_LOW_MAH` 增大，clean_unsafe 区单调收缩，说明缺口是软件配置阈值的函数，而不是单纯“逆风耗电”的物理常识。

## 图

- outcome_vs_D_wind: ![](planc/analysis/energy_headwind_v1_outcome_vs_D_wind.png)
- pstrat_batt_low_mah_monotonic: ![](planc/analysis/energy_headwind_v1_pstrat_batt_low_mah_monotonic.png)
- boundary_wide_range: ![](planc/analysis/energy_headwind_v1_boundary_wide_range.png)
- consequence_type_distribution: ![](planc/analysis/energy_headwind_v1_consequence_type_distribution.png)

## 审计文件

每个 run 均有 `planc/logs/<run_id>.BIN`、`<run_id>_params.json`、`<run_id>_parsed.csv`、`<run_id>_parsed.oracle.json`。结果 JSON 中 `summary.points[*]` 带有对应路径。

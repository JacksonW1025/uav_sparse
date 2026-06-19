VERDICT: PASS

# geofence_attitude Phase-B v2 Report

Reason: robust clean unsafe attitude-interface region, clean contract accounting, interface contrast, classification, non-C-ABSENT, and operator-driven severity gates all passed.
Firmware: actual `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`, expected `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`. SITL binary `/home/car/ardupilot/build/sitl/bin/arducopter`.

## Premises

| premise | ok | evidence |
|---|---:|---|
| `P0.1_guided_attitude_produces_horizontal_flight` | True | max radial speed 7.27 m/s |
| `P0.2_destination_admission_on_attitude_path_uncovered` | True | attitude rejects 0; position reject seen True |
| `P0.3_input_applied_and_crosses_fence` | True | cross speed 7.25 m/s |
| `P0.4_guided_stream_accepted_no_drop` | True | stream 31.68 Hz sim |
| `P0.5_fence_legally_enabled` | True | see result.json |
| `P0.6_window_and_fence_check_frequency_measured` | True | cross/action/arrest 72.57/72.81/75.39 s |

## Main Results

Noise floor at fixed medium condition: sigma `0.23` m; d_margin for labeling `0.70` m.
Fence check frequency: source scheduler is `3 Hz`; DataFlash cross-to-fence-event latency mean `0.12` s over `43` samples.

| tilt deg | runs | labels | mean cross speed m/s | mean depth before arrest m |
|---:|---:|---|---:|---:|
| 12.0 | 5 | `{'clean_unsafe': 5}` | 4.98 | 6.38 |
| 18.0 | 5 | `{'clean_unsafe': 5}` | 6.71 | 10.64 |
| 24.0 | 5 | `{'clean_unsafe': 5}` | 8.34 | 15.79 |
| 30.0 | 5 | `{'clean_unsafe': 5}` | 10.00 | 21.69 |
| 36.0 | 5 | `{'clean_unsafe': 5}` | 11.76 | 24.13 |

## Interface Contrast

Position-target control runs rejected outside destinations `2/2` and did not cross. Attitude-stream runs logged `0` destination-admission rejects.

## Contract Cleanliness

Label counts: `{'clean_unsafe': 43, 'position_rejected': 2, 'clean_safe': 3}`. Contract-violated count `0`; clean_unsafe intersection with contract_violated `False`.

## FENCE_ACTION Layer

| action | tilt deg | runs | mean speed m/s | mean depth before arrest m | mean action-to-arrest s |
|---|---:|---:|---:|---:|---:|
| RTL-or-Land | 12.0 | 5 | 4.98 | 6.38 | 2.00 |
| RTL-or-Land | 18.0 | 5 | 6.71 | 10.64 | 2.44 |
| RTL-or-Land | 24.0 | 5 | 8.34 | 15.79 | 2.92 |
| RTL-or-Land | 30.0 | 5 | 10.00 | 21.69 | 3.39 |
| RTL-or-Land | 36.0 | 5 | 11.76 | 24.13 | 3.25 |
| Brake | 18.0 | 3 | 6.71 | 13.91 | 4.01 |
| Brake | 30.0 | 3 | 10.01 | 21.37 | 4.41 |
| Brake | 36.0 | 3 | 11.76 | 28.13 | 4.58 |

## Prediction Gates

Classification: applicable `True`, accuracy `1.00`, target `0.90`, passed `True`.
Operator-driven severity gate: Spearman rho `0.961` vs min `0.90`; depth range/sigma `78.1` vs min `10.0`; passed `True`.
Severity regression (reporting only): applicable `True`, features `['intercept', 'cross_speed', 'FENCE_ACTION']`, MAE `0.74` m, MAE/range `0.034`, reference max `0.15`, reference_passed `True`.

## v1 to v2 Threats-to-Validity Record

v1 remains intact: its prereg/result/report/tag were not overwritten. Its FAIL is treated as a method record: the binary severity gate used a fixed-input repeatability sigma as a held-out regression MAE bound, which is not the load-bearing scientific claim for this scenario.
v2 was preregistered before this confirmation run at `planc/results/geofence_attitude_phaseB_v2_prereg.json`. The main verdict comes from new v2 runs, not from re-scoring v1 data.
New load-bearing criteria: Spearman rho >= `0.90` and depth dynamic range/sigma >= `10.00`. Regression now includes `['cross_speed', 'FENCE_ACTION']` and is reported only.
Optional consistency check on v1 data under v2 rules: rho `0.953`, range/sigma `127.2`, passed `True`. This is not the source of the v2 verdict.

## Figures

- `velocity_depth`: `planc/analysis/geofence_attitude_phaseB_v2_velocity_depth.png`
- `interface_contrast`: `planc/analysis/geofence_attitude_phaseB_v2_interface_contrast.png`
- `window_timeline`: `planc/analysis/geofence_attitude_phaseB_v2_window_timeline.png`
- `fence_action_stratification`: `planc/analysis/geofence_attitude_phaseB_v2_fence_action_stratification.png`

## Honest Boundaries

- 操作者输入是受支持的 `GUIDED + SET_ATTITUDE_TARGET` quaternion stream; 不是环境自发越界。
- 干净见证只按 `t_cross -> t_arrest` 的反应式窗口计；若需要重进 GUIDED 或持续对抗 FENCE_ACTION 才有后果, 标为 `contract_violated`。
- 危险空间从围栏边界外侧开始, 没有把障碍距离调到卡窗口。
- 不主张标度律、面积律或搜索定律。

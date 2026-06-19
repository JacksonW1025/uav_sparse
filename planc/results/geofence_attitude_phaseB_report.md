VERDICT: FAIL

# geofence_attitude Phase-B Report

Reason: premises held but PASS criteria were not met and the C-ABSENT arrest condition did not cover all runs.
Firmware: actual `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`, expected `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`. SITL binary `/home/car/ardupilot/build/sitl/bin/arducopter`.

## Premises

| premise | ok | evidence |
|---|---:|---|
| `P0.1_guided_attitude_produces_horizontal_flight` | True | max radial speed 7.27 m/s |
| `P0.2_destination_admission_on_attitude_path_uncovered` | True | attitude rejects 0; position reject seen True |
| `P0.3_input_applied_and_crosses_fence` | True | cross speed 7.26 m/s |
| `P0.4_guided_stream_accepted_no_drop` | True | stream 31.65 Hz sim |
| `P0.5_fence_legally_enabled` | True | see result.json |
| `P0.6_window_and_fence_check_frequency_measured` | True | cross/action/arrest 88.58/88.77/91.39 s |

## Main Results

Noise floor at fixed medium condition: sigma `0.14` m, d_margin `0.42` m, MAE bound `0.21` m.
Fence check frequency: source scheduler is `3 Hz`; DataFlash cross-to-fence-event latency mean `0.12` s over `43` samples.

| tilt deg | runs | labels | mean cross speed m/s | mean depth before arrest m |
|---:|---:|---|---:|---:|
| 12.0 | 5 | `{'clean_unsafe': 5}` | 4.98 | 6.43 |
| 18.0 | 5 | `{'clean_unsafe': 5}` | 6.71 | 10.71 |
| 24.0 | 5 | `{'clean_unsafe': 5}` | 8.34 | 16.20 |
| 30.0 | 5 | `{'clean_unsafe': 5}` | 10.01 | 21.69 |
| 36.0 | 5 | `{'clean_unsafe': 5}` | 11.76 | 23.98 |

## Interface Contrast

Position-target control runs rejected outside destinations `2/2` and did not cross. Attitude-stream runs logged `0` destination-admission rejects.

## Contract Cleanliness

Label counts: `{'clean_unsafe': 43, 'position_rejected': 2, 'clean_safe': 3}`. Contract-violated count `0`; clean_unsafe intersection with contract_violated `False`.

## FENCE_ACTION Layer

| action | tilt deg | runs | mean speed m/s | mean depth before arrest m | mean action-to-arrest s |
|---|---:|---:|---:|---:|---:|
| RTL-or-Land | 12.0 | 5 | 4.98 | 6.43 | 2.00 |
| RTL-or-Land | 18.0 | 5 | 6.71 | 10.71 | 2.44 |
| RTL-or-Land | 24.0 | 5 | 8.34 | 16.20 | 2.93 |
| RTL-or-Land | 30.0 | 5 | 10.01 | 21.69 | 3.37 |
| RTL-or-Land | 36.0 | 5 | 11.76 | 23.98 | 3.24 |
| Brake | 18.0 | 3 | 6.71 | 12.82 | 4.03 |
| Brake | 30.0 | 3 | 10.00 | 21.41 | 4.43 |
| Brake | 36.0 | 3 | 11.76 | 28.27 | 4.58 |

## Prediction Gates

Classification: applicable `True`, accuracy `1.00`, target `0.90`, passed `True`.
Severity regression: applicable `True`, MAE `0.86` m, bound `0.21` m, passed `False`.

## Figures

- `velocity_depth`: `planc/analysis/geofence_attitude_phaseB_velocity_depth.png`
- `interface_contrast`: `planc/analysis/geofence_attitude_phaseB_interface_contrast.png`
- `window_timeline`: `planc/analysis/geofence_attitude_phaseB_window_timeline.png`
- `fence_action_stratification`: `planc/analysis/geofence_attitude_phaseB_fence_action_stratification.png`

## Honest Boundaries

- 操作者输入是受支持的 `GUIDED + SET_ATTITUDE_TARGET` quaternion stream; 不是环境自发越界。
- 干净见证只按 `t_cross -> t_arrest` 的反应式窗口计；若需要重进 GUIDED 或持续对抗 FENCE_ACTION 才有后果, 标为 `contract_violated`。
- 危险空间从围栏边界外侧开始, 没有把障碍距离调到卡窗口。
- 不主张标度律、面积律或搜索定律。

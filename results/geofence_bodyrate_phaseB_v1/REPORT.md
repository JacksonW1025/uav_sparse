VERDICT: FAIL
MATRIX: prediction-gate-failed

# geofence_bodyrate Phase-B v1 Report

Reason: robust clean_unsafe witnesses were observed, but the preregistered held-out severity prediction gate failed.
Firmware: actual `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`, expected `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce`. SITL binary `/home/car/ardupilot/build/sitl/bin/arducopter`.
Interface path: supported MAVLink `GUIDED + SET_ATTITUDE_TARGET` with `ATTITUDE_IGNORE`, body roll-rate field active, thrust field `0.5` as zero climb-rate. No ACRO fallback and no RC override were used.

## Four Criteria

| criterion | passed | evidence |
|---|---:|---|
| premise | True | all Phase-0 checks `True` |
| robust clean_unsafe | True | count `25`, depth range `[4.910944587420019, 24.80648047609381]`, window range `[4.531585791688926, 10.536545616133182]` |
| zero preventive violations / PGFUZZ invisible | True | contract_violated `0`, destination rejects `0` |
| prediction gates | False | classification `1.00`, extrapolation `1.00`, Spearman `0.386`, range/sigma `20.8` |

## Premises

| premise | ok | evidence |
|---|---:|---|
| `P0.1_guided_bodyrate_supported_and_applied` | True | rate actual/cmd 127.10/120.00 deg/s |
| `P0.2_bodyrate_has_no_destination_admission_reject` | True | destination rejects 0 |
| `P0.3_bodyrate_drives_horizontal_crossing` | True | cross speed 11.11 m/s |
| `P0.4_reactive_fence_action_fires` | True | action RTL at 88.43 s |
| `P0.5_fence_and_avoidance_params_legal` | True | see verdict.json |
| `P0.6_stream_rate_and_altitude_fidelity` | True | stream 36.69 Hz sim |

## Noise And Labels

Noise fixed point: sigma `1.19` m over `15` runs; `d_margin = 3*sigma = 3.57` m. Labels use `clean_unsafe` only for outside depth `> d_margin`; depths in `(0, d_margin]` are ambiguous.

## M x E Grid

| rate deg/s | wind m/s | margin m | runs | stable | labels | mean depth m | mean cross speed m/s | mean peak roll deg |
|---:|---:|---:|---:|---:|---|---:|---:|---:|
| 20 | 0 | 2.0 | 3 | True | `{'clean_safe': 3}` | 0.00 | n/a | 5.41 |
| 20 | 3 | 2.0 | 3 | True | `{'ambiguous': 2, 'clean_safe': 1}` | 0.00 | n/a | 0.51 |
| 20 | 6 | 2.0 | 2 | False | `{'ambiguous': 2}` | 0.00 | n/a | -5.92 |
| 20 | 9 | 2.0 | 2 | False | `{'ambiguous': 2}` | 0.00 | n/a | -6.20 |
| 40 | 0 | 2.0 | 3 | True | `{'clean_safe': 3}` | 0.00 | n/a | 10.63 |
| 40 | 3 | 2.0 | 3 | True | `{'clean_safe': 3}` | 0.00 | n/a | 6.11 |
| 40 | 6 | 2.0 | 2 | True | `{'clean_safe': 2}` | 0.00 | n/a | -0.98 |
| 40 | 9 | 2.0 | 2 | False | `{'ambiguous': 2}` | 4.75 | 3.96 | 0.01 |
| 60 | 0 | 2.0 | 3 | True | `{'clean_safe': 3}` | 0.00 | n/a | 16.30 |
| 60 | 3 | 2.0 | 3 | True | `{'ambiguous': 2, 'clean_safe': 1}` | 1.87 | 2.35 | 12.03 |
| 60 | 6 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 6.25 | 4.48 | 5.35 |
| 60 | 9 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 12.26 | 6.79 | 0.01 |
| 90 | 0 | 2.0 | 2 | True | `{'ambiguous': 1, 'clean_safe': 1}` | 0.82 | 1.61 | 23.73 |
| 90 | 3 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 5.35 | 5.00 | 19.12 |
| 90 | 6 | 2.0 | 2 | True | `{'clean_unsafe': 1, 'ambiguous': 1}` | 4.10 | 4.41 | 11.32 |
| 90 | 9 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 10.80 | 7.26 | 5.92 |
| 120 | 0 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 21.65 | 10.49 | 32.91 |
| 120 | 3 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 22.20 | 12.00 | 29.53 |
| 120 | 6 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 23.00 | 12.85 | 23.96 |
| 120 | 9 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 22.52 | 12.20 | 13.02 |
| 180 | 0 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 23.61 | 12.82 | 43.33 |
| 180 | 3 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 23.72 | 16.50 | 44.75 |
| 180 | 6 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 22.98 | 16.85 | 38.17 |
| 180 | 9 | 2.0 | 2 | True | `{'clean_unsafe': 2}` | 21.85 | 16.86 | 29.44 |

## Prediction

Classification split: train low conditions `{'rate_deg_s_lte': 90.0, 'wind_m_s_lte': 6.0}`, extrapolate high conditions `{'rate_deg_s_gte': 120.0, 'wind_m_s_gte': 6.0}`. Accuracy `1.00`, extrapolation accuracy `1.00`, target `0.90`, passed `True`.
Severity gate: Spearman predicted-vs-actual depth `0.386` vs min `0.90`; depth range/sigma `20.8` vs min `10.0`; passed `False`.
Reporting-only diagnostic: achieved cross-speed vs actual depth Spearman `0.952`. This is not used to rescue the v1 verdict because the committed v1 gate used the held-out depth regression above.
Severity regression is reporting-only: features `['intercept', 'commanded_rate_deg_s', 'wind_m_s', 'achieved_peak_roll_deg']`, MAE `13.14` m, MAE/range `0.530`.

## P Layer

AC_Fence circle breach check in this SHA compares home distance directly to FENCE_RADIUS; FENCE_MARGIN is not used by check_fence_circle with AVOID_ENABLE=0. The layer is reported empirically, not forced into the verdict.

| FENCE_MARGIN m | runs | labels | clean_unsafe count | mean depth m | median depth m | max depth m |
|---:|---:|---|---:|---:|---:|---:|
| 1.0 | 24 | `{'clean_safe': 6, 'ambiguous': 5, 'clean_unsafe': 13}` | 13 | 10.77 | 8.04 | 27.06 |
| 2.0 | 54 | `{'clean_safe': 17, 'ambiguous': 12, 'clean_unsafe': 25}` | 25 | 8.47 | 2.80 | 24.81 |
| 4.0 | 24 | `{'clean_safe': 4, 'ambiguous': 7, 'clean_unsafe': 13}` | 13 | 10.74 | 5.67 | 25.86 |
| 8.0 | 24 | `{'clean_safe': 5, 'ambiguous': 7, 'clean_unsafe': 12}` | 12 | 10.10 | 6.01 | 25.17 |
Observed monotone non-increasing mean depth: `False`.

## Figures

- `grid_labels`: `results/geofence_bodyrate_phaseB_v1/figures/grid_labels.png`
- `depth_vs_rate_by_wind`: `results/geofence_bodyrate_phaseB_v1/figures/depth_vs_rate_by_wind.png`
- `fence_margin_layer`: `results/geofence_bodyrate_phaseB_v1/figures/fence_margin_layer.png`
- `representative_trajectory`: `results/geofence_bodyrate_phaseB_v1/figures/representative_trajectory.png`

## Honest Boundary Checklist

- Availability is `supported MAVLink/GUIDED + SET_ATTITUDE_TARGET body-rate`; no RC override was used.
- Geofence action is a reactive fallback, not a preventive contract violation; clean windows stop at return/arrest and no mode switch back into GUIDED is used after FENCE_ACTION.
- The consequence is horizontal outside depth and outside duration, not crash or natural environmental reachability.
- `FENCE_MARGIN` is reported empirically because the current circle breach source does not use it as an early trigger with avoidance disabled.
- No claims are made about scaling laws, area laws, or search complexity.

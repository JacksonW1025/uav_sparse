VERDICT: FAIL

# planc GCS link-loss boundary-excursion second scenario

## Four decisive criteria

- Premise satisfied: **True**.
- Robust contract-clean unsafe region: **True**; clean_unsafe count=20, stable repeated clean_unsafe count=4, boundary flips reported=default_v07_w00, default_v07_w09, default_v07_w12, default_v07_w15.
- Link-loss failsafe contract clean and PGFUZZ-invisible: **True**; contract_violated count=0.
- Held-out prediction with extrapolation >= 90%: **False**; interpolation=0.778, extrapolation=1.000, combined=0.846.

Decision reason: held-out prediction including extrapolation is below 90% or incomplete

## Premise

Premise conclusion: **True** - GCS failsafe triggered and excursion responded monotonically to speed, wind, and timeout.

| run | speed m/s | wind m/s | timeout s | label | overshoot m | observed timeout s | parsed log | oracle |
| --- | ---: | ---: | ---: | --- | ---: | ---: | --- | --- |
| linkloss_premise_speed_base | 8.00 | 9.00 | 5.00 | clean_unsafe | 8.07 | 4.95 | planc/logs/linkloss_premise_speed_base_parsed.csv | planc/logs/linkloss_premise_speed_base_parsed.oracle.json |
| linkloss_premise_speed_high | 10.00 | 9.00 | 5.00 | clean_unsafe | 19.04 | 4.65 | planc/logs/linkloss_premise_speed_high_parsed.csv | planc/logs/linkloss_premise_speed_high_parsed.oracle.json |
| linkloss_premise_wind_base | 10.00 | 0.00 | 5.00 | clean_unsafe | 19.71 | 4.91 | planc/logs/linkloss_premise_wind_base_parsed.csv | planc/logs/linkloss_premise_wind_base_parsed.oracle.json |
| linkloss_premise_wind_high | 10.00 | 12.00 | 5.00 | clean_unsafe | 20.63 | 4.75 | planc/logs/linkloss_premise_wind_high_parsed.csv | planc/logs/linkloss_premise_wind_high_parsed.oracle.json |
| linkloss_premise_timeout_short | 10.00 | 9.00 | 2.00 | clean_safe | 0.00 | 1.76 | planc/logs/linkloss_premise_timeout_short_parsed.csv | planc/logs/linkloss_premise_timeout_short_parsed.oracle.json |
| linkloss_premise_timeout_default | 10.00 | 9.00 | 5.00 | clean_unsafe | 22.19 | 4.98 | planc/logs/linkloss_premise_timeout_default_parsed.csv | planc/logs/linkloss_premise_timeout_default_parsed.oracle.json |
| linkloss_premise_timeout_long | 10.00 | 9.00 | 8.00 | clean_unsafe | 49.45 | 7.74 | planc/logs/linkloss_premise_timeout_long_parsed.csv | planc/logs/linkloss_premise_timeout_long_parsed.oracle.json |

- gcs_failsafe_triggers_and_RTLs (hard), ok=True.
- overshoot_increases_with_speed (hard): baseline=8.07, stressed=19.04, ok=True.
- overshoot_increases_with_tailwind (hard): baseline=19.71, stressed=20.63, ok=True.
- overshoot_increases_with_timeout (hard): values=['0.00', '22.19', '49.45'], ok=True.

## Scenario

Fixed P uses `FS_GCS_ENABLE=1` (RTL), `FS_OPTIONS=0`, `SYSID_MYGCS=255`, `FENCE_ENABLE=1`, `FENCE_TYPE=2` circular fence, `FENCE_RADIUS=120 m`, `FENCE_ACTION=0` Report Only, and `AVOID_ENABLE=0`. All controlled parameters are read back per run.
The AUTO mission waypoint is 260 m east of home, outside the operator fence. The GCS heartbeat is stopped at 80 m from home, which is 40 m inside the fence.
M scans command speed over [5, 6, 7, 8, 9, 10] m/s using `MAV_CMD_DO_CHANGE_SPEED`; E scans outbound tailwind over [0, 3, 6, 9, 12, 15] m/s with `SIM_WIND_DIR=270` and `SIM_WIND_TURB=0`. `WPNAV_SPEED` is fixed above the maximum command speed, so it is a cap rather than the scanned input.
Oracle: unsafe means `max_distance > R + 1.50 m`; severity is `max_distance - R`. The boundary is the configured operator fence, not a constructed analysis line.

## Three-Zone Field

Default layer `default` (`FS_GCS_TIMEOUT=5 s`) counts: clean_safe=16, clean_unsafe=20, contract_violated=0, blocked=0.

| speed m/s | wind m/s | label | overshoot m | max dist m | observed timeout s | stable | runs |
| ---: | ---: | --- | ---: | ---: | ---: | --- | --- |
| 5 | 0 | clean_safe | 0.00 | 106.31 | 4.46 | True | linkloss_default_v05_w00_r1 |
| 5 | 3 | clean_safe | 0.00 | 110.17 | 5.13 | True | linkloss_default_v05_w03_r1 |
| 5 | 6 | clean_safe | 0.00 | 106.85 | 4.56 | True | linkloss_default_v05_w06_r1 |
| 5 | 9 | clean_safe | 0.00 | 108.55 | 4.80 | True | linkloss_default_v05_w09_r1 |
| 5 | 12 | clean_safe | 0.00 | 110.26 | 5.13 | True | linkloss_default_v05_w12_r1 |
| 5 | 15 | clean_safe | 0.00 | 107.43 | 4.36 | True | linkloss_default_v05_w15_r1 |
| 6 | 0 | clean_safe | 0.00 | 114.83 | 4.97 | True | linkloss_default_v06_w00_r1 |
| 6 | 3 | clean_safe | 0.00 | 111.90 | 4.41 | True | linkloss_default_v06_w03_r1 |
| 6 | 6 | clean_safe | 0.00 | 114.17 | 4.74 | True | linkloss_default_v06_w06_r1 |
| 6 | 9 | clean_safe | 0.00 | 116.15 | 5.07 | True | linkloss_default_v06_w09_r1 |
| 6 | 12 | clean_safe | 0.00 | 116.16 | 5.00 | True | linkloss_default_v06_w12_r1, linkloss_default_v06_w12_r2, linkloss_default_v06_w12_r3 |
| 6 | 15 | clean_safe | 0.00 | 114.83 | 4.64 | True | linkloss_default_v06_w15_r1, linkloss_default_v06_w15_r2, linkloss_default_v06_w15_r3 |
| 7 | 0 | clean_safe | 0.96 | 120.71 | 4.90 | False | linkloss_default_v07_w00_r1, linkloss_default_v07_w00_r2, linkloss_default_v07_w00_r3 |
| 7 | 3 | clean_safe | 0.13 | 118.99 | 4.65 | True | linkloss_default_v07_w03_r1, linkloss_default_v07_w03_r2, linkloss_default_v07_w03_r3 |
| 7 | 6 | clean_safe | 0.73 | 120.36 | 4.76 | True | linkloss_default_v07_w06_r1, linkloss_default_v07_w06_r2, linkloss_default_v07_w06_r3 |
| 7 | 9 | clean_safe | 1.73 | 121.46 | 4.91 | False | linkloss_default_v07_w09_r1, linkloss_default_v07_w09_r2, linkloss_default_v07_w09_r3 |
| 7 | 12 | clean_unsafe | 1.66 | 121.38 | 4.84 | False | linkloss_default_v07_w12_r1, linkloss_default_v07_w12_r2, linkloss_default_v07_w12_r3 |
| 7 | 15 | clean_unsafe | 3.07 | 123.07 | 4.89 | False | linkloss_default_v07_w15_r1, linkloss_default_v07_w15_r2, linkloss_default_v07_w15_r3 |
| 8 | 0 | clean_unsafe | 5.47 | 125.47 | 4.72 | True | linkloss_default_v08_w00_r1, linkloss_default_v08_w00_r2, linkloss_default_v08_w00_r3 |
| 8 | 3 | clean_unsafe | 5.60 | 125.60 | 4.73 | True | linkloss_default_v08_w03_r1, linkloss_default_v08_w03_r2, linkloss_default_v08_w03_r3 |
| 8 | 6 | clean_unsafe | 5.72 | 125.72 | 4.72 | True | linkloss_default_v08_w06_r1, linkloss_default_v08_w06_r2, linkloss_default_v08_w06_r3 |
| 8 | 9 | clean_unsafe | 5.31 | 125.31 | 4.61 | True | linkloss_default_v08_w09_r1, linkloss_default_v08_w09_r2, linkloss_default_v08_w09_r3 |
| 8 | 12 | clean_unsafe | 10.66 | 130.66 | 5.18 | True | linkloss_default_v08_w12_r1 |
| 8 | 15 | clean_unsafe | 9.20 | 129.20 | 4.85 | True | linkloss_default_v08_w15_r1 |
| 9 | 0 | clean_unsafe | 12.91 | 132.91 | 4.88 | True | linkloss_default_v09_w00_r1 |
| 9 | 3 | clean_unsafe | 15.33 | 135.33 | 5.08 | True | linkloss_default_v09_w03_r1 |
| 9 | 6 | clean_unsafe | 10.98 | 130.98 | 4.61 | True | linkloss_default_v09_w06_r1 |
| 9 | 9 | clean_unsafe | 11.81 | 131.81 | 4.61 | True | linkloss_default_v09_w09_r1 |
| 9 | 12 | clean_unsafe | 15.38 | 135.38 | 4.95 | True | linkloss_default_v09_w12_r1 |
| 9 | 15 | clean_unsafe | 13.30 | 133.30 | 4.51 | True | linkloss_default_v09_w15_r1 |
| 10 | 0 | clean_unsafe | 19.65 | 139.65 | 4.91 | True | linkloss_default_v10_w00_r1 |
| 10 | 3 | clean_unsafe | 22.83 | 142.83 | 5.22 | True | linkloss_default_v10_w03_r1 |
| 10 | 6 | clean_unsafe | 18.51 | 138.51 | 4.75 | True | linkloss_default_v10_w06_r1 |
| 10 | 9 | clean_unsafe | 16.31 | 136.31 | 4.42 | True | linkloss_default_v10_w09_r1 |
| 10 | 12 | clean_unsafe | 20.47 | 140.47 | 4.75 | True | linkloss_default_v10_w12_r1 |
| 10 | 15 | clean_unsafe | 21.57 | 141.57 | 4.75 | True | linkloss_default_v10_w15_r1 |

PGFUZZ-invisible check: every `clean_unsafe` point requires a GCS failsafe ERR plus RTL mode change at the configured timeout, with no unrelated ERR/EV/STATUSTEXT failsafes and with parameter readback success. Report-Only fence breach ERR/STATUSTEXT records are explicitly treated as oracle measurement reports, not contract violations.

## Predictive Rule

Formula: `unsafe probability = sigmoid(beta0 + beta_v*v + beta_vw*v*wind + beta_w*wind)`.
interpolation: n=9, accuracy=0.778, model_ok=True, train_points=9.
extrapolation: n=4, accuracy=1.000, model_ok=True, train_points=16.
Combined held-out accuracy: 0.846.

## P Stratification

Conclusion: clean_unsafe count is nondecreasing as FS_GCS_TIMEOUT lengthens; shorter timeout shrinks the unsafe region.

| layer | FS_GCS_TIMEOUT s | clean_unsafe | clean_safe | contract_violated |
| --- | ---: | ---: | ---: | ---: |
| conservative | 2 | 0 | 36 | 0 |
| default | 5 | 20 | 16 | 0 |
| lenient | 8 | 35 | 1 | 0 |

## Search Efficiency

discrete bisection over speed for each wind, replayed against completed grid run results. Queries to bracket boundaries: 12 vs full grid 36.

## Reproducibility

Repeated near-boundary points: 12; boundary flips: default_v07_w00, default_v07_w09, default_v07_w12, default_v07_w15.
For each run id in the field table, audit files are `planc/logs/<run_id>_params.json`, `planc/logs/<run_id>_parsed.csv`, and `planc/logs/<run_id>_parsed.oracle.json`.

## Unified Method Statement

The RTL energy scenario and this GCS link-loss scenario are both threshold-insufficiency specification gaps: the former is an energy-budget threshold, while this one is a data-link time-budget threshold. In both, ArduCopter follows the configured failsafe contract, but a legal operating condition crosses an external safety oracle.

## Figures

- premise: ![](planc/analysis/linkloss_premise.png)
- result_field: ![](planc/analysis/linkloss_result_field.png)
- severity: ![](planc/analysis/linkloss_severity_heatmap.png)
- p_stratification: ![](planc/analysis/linkloss_p_stratification.png)
- train_test: ![](planc/analysis/linkloss_train_test.png)

## Limitations

This is ArduCopter SITL, not HITL. The verdict applies to this SITL vehicle, parameter set, and pre-registered link-loss boundary excursion scenario. Logs, readbacks, parsed CSVs, and oracle sidecars are kept under `planc/logs/` for independent audit.

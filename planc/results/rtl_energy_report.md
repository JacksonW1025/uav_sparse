VERDICT: PASS

# planc RTL energy spec-gap decisive test

## Four decisive criteria

- Premise satisfied: **True**.
- Robust contract-clean unsafe region: **True**; clean_unsafe count=14, boundary flips=none.
- Battery contract clean and PGFUZZ-invisible: **True**; contract_violated count=0.
- Held-out prediction with extrapolation >= 90%: **True**; interpolation=1.000, extrapolation=1.000, combined=1.000.

Decision reason: All decisive criteria are satisfied.

## Premise

Premise conclusion: **True** - wind and mass monotonically increased consumed mAh.

| run | model mass kg | wind m/s | consumed mAh | voltage drop rate V/s | parsed log | oracle |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| rtl_premise_nominal_no_wind | 3.00 | 0.00 | 524.46 | 0.00000 | planc/logs/rtl_premise_nominal_no_wind_parsed.csv | planc/logs/rtl_premise_nominal_no_wind_parsed.oracle.json |
| rtl_premise_nominal_headwind | 3.00 | 10.00 | 610.77 | 0.00000 | planc/logs/rtl_premise_nominal_headwind_parsed.csv | planc/logs/rtl_premise_nominal_headwind_parsed.oracle.json |
| rtl_premise_heavy_no_wind | 3.25 | 0.00 | 567.37 | 0.00000 | planc/logs/rtl_premise_heavy_no_wind_parsed.csv | planc/logs/rtl_premise_heavy_no_wind_parsed.oracle.json |

- wind_response_consumed_mAh (hard): baseline=524.46, stressed=610.77, ok=True.
- wind_response_voltage_drop_rate (reported): baseline=0.00, stressed=0.00, ok=False.
- mass_response_consumed_mAh (hard): baseline=524.46, stressed=567.37, ok=True.
- mass_response_voltage_drop_rate (reported): baseline=0.00, stressed=0.00, ok=False.
- Caveat: BAT voltage stayed constant in this SITL binary; the premise gate uses consumed mAh, which is the same signal used by the configured capacity failsafes.

## Scenario

Fixed P uses `BATT_FS_LOW_ACT=2` (RTL), `BATT_FS_CRT_ACT=1` (LAND), `BATT_CAPACITY=650 mAh`, `SIM_BATT_CAP_AH=0.65 Ah`, `BATT_CRT_MAH=60 mAh`, and no geofence.
M scans outbound distance D over [40, 60, 80, 100, 120, 140] at 8.00 m/s. E scans wind over [0, 3, 6, 9, 12, 15] m/s with `SIM_WIND_DIR=270`, so outbound east is downwind and RTL westbound return is into wind.
`BATT_LOW_VOLT=0` and `BATT_CRT_VOLT=0` intentionally disable voltage failsafes; the legal documented capacity thresholds are the tested contract, with all set parameters read back per run.

## Three-Zone Field

Default layer `default` counts: clean_safe=22, clean_unsafe=14, contract_violated=0, blocked=0.

| D m | wind m/s | label | final dist m | dist at low FS m | stable | runs |
| ---: | ---: | --- | ---: | ---: | --- | --- |
| 40 | 0 | clean_safe | 0.00 | 42.96 | True | rtl_default_D040_W00_r1 |
| 40 | 3 | clean_safe | 0.02 | 42.55 | True | rtl_default_D040_W03_r1 |
| 40 | 6 | clean_safe | 0.05 | 42.63 | True | rtl_default_D040_W06_r1 |
| 40 | 9 | clean_safe | 0.09 | 42.67 | True | rtl_default_D040_W09_r1 |
| 40 | 12 | clean_safe | 0.13 | 42.83 | True | rtl_default_D040_W12_r1, rtl_default_D040_W12_r2, rtl_default_D040_W12_r3 |
| 40 | 15 | clean_unsafe | 17.08 | 42.79 | True | rtl_default_D040_W15_r1, rtl_default_D040_W15_r2, rtl_default_D040_W15_r3 |
| 60 | 0 | clean_safe | 0.01 | 62.91 | True | rtl_default_D060_W00_r1 |
| 60 | 3 | clean_safe | 0.02 | 62.90 | True | rtl_default_D060_W03_r1 |
| 60 | 6 | clean_safe | 0.04 | 62.67 | True | rtl_default_D060_W06_r1 |
| 60 | 9 | clean_safe | 0.09 | 62.83 | True | rtl_default_D060_W09_r1 |
| 60 | 12 | clean_safe | 5.66 | 62.60 | True | rtl_default_D060_W12_r1, rtl_default_D060_W12_r2, rtl_default_D060_W12_r3 |
| 60 | 15 | clean_unsafe | 36.58 | 62.24 | True | rtl_default_D060_W15_r1, rtl_default_D060_W15_r2, rtl_default_D060_W15_r3 |
| 80 | 0 | clean_safe | 0.01 | 82.23 | True | rtl_default_D080_W00_r1 |
| 80 | 3 | clean_safe | 0.02 | 82.59 | True | rtl_default_D080_W03_r1 |
| 80 | 6 | clean_safe | 0.05 | 82.66 | True | rtl_default_D080_W06_r1 |
| 80 | 9 | clean_safe | 0.37 | 82.36 | True | rtl_default_D080_W09_r1, rtl_default_D080_W09_r2, rtl_default_D080_W09_r3 |
| 80 | 12 | clean_unsafe | 25.32 | 82.38 | True | rtl_default_D080_W12_r1, rtl_default_D080_W12_r2, rtl_default_D080_W12_r3 |
| 80 | 15 | clean_unsafe | 56.61 | 82.28 | True | rtl_default_D080_W15_r1 |
| 100 | 0 | clean_safe | 0.01 | 102.21 | True | rtl_default_D100_W00_r1 |
| 100 | 3 | clean_safe | 0.02 | 102.52 | True | rtl_default_D100_W03_r1 |
| 100 | 6 | clean_safe | 0.32 | 102.72 | True | rtl_default_D100_W06_r1, rtl_default_D100_W06_r2, rtl_default_D100_W06_r3 |
| 100 | 9 | clean_unsafe | 15.34 | 102.27 | True | rtl_default_D100_W09_r1, rtl_default_D100_W09_r2, rtl_default_D100_W09_r3 |
| 100 | 12 | clean_unsafe | 45.30 | 102.09 | True | rtl_default_D100_W12_r1 |
| 100 | 15 | clean_unsafe | 76.41 | 102.09 | True | rtl_default_D100_W15_r1 |
| 120 | 0 | clean_safe | 0.01 | 122.81 | True | rtl_default_D120_W00_r1 |
| 120 | 3 | clean_safe | 0.02 | 122.01 | True | rtl_default_D120_W03_r1 |
| 120 | 6 | clean_safe | 6.97 | 122.34 | True | rtl_default_D120_W06_r1, rtl_default_D120_W06_r2, rtl_default_D120_W06_r3 |
| 120 | 9 | clean_unsafe | 35.15 | 122.14 | True | rtl_default_D120_W09_r1, rtl_default_D120_W09_r2, rtl_default_D120_W09_r3 |
| 120 | 12 | clean_unsafe | 65.08 | 122.14 | True | rtl_default_D120_W12_r1 |
| 120 | 15 | clean_unsafe | 96.61 | 122.29 | True | rtl_default_D120_W15_r1 |
| 140 | 0 | clean_safe | 0.27 | 142.36 | True | rtl_default_D140_W00_r1 |
| 140 | 3 | clean_safe | 3.53 | 142.55 | True | rtl_default_D140_W03_r1, rtl_default_D140_W03_r2, rtl_default_D140_W03_r3 |
| 140 | 6 | clean_unsafe | 26.95 | 142.51 | True | rtl_default_D140_W06_r1, rtl_default_D140_W06_r2, rtl_default_D140_W06_r3 |
| 140 | 9 | clean_unsafe | 55.72 | 142.71 | True | rtl_default_D140_W09_r1 |
| 140 | 12 | clean_unsafe | 85.45 | 142.17 | True | rtl_default_D140_W12_r1 |
| 140 | 15 | clean_unsafe | 116.66 | 142.33 | True | rtl_default_D140_W15_r1 |

PGFUZZ-invisible check: every `clean_unsafe` point has `contract_clean_all=True`, battery low RTL and critical LAND actions are checked against the configured mAh thresholds, and points with any other ERR/EV/STATUSTEXT failsafe are labeled `contract_violated` instead.

## Predictive Rule

Formula: `unsafe probability = sigmoid(beta0 + beta_D*D + beta_DW*D*wind + beta_W*wind)`.
interpolation: n=9, accuracy=1.000, model_ok=True, train_points=9.
extrapolation: n=4, accuracy=1.000, model_ok=True, train_points=16.
Combined held-out accuracy: 1.000.

## P Stratification

Conclusion: clean_unsafe count is non-increasing as BATT_LOW_MAH rises.

| layer | BATT_LOW_MAH | clean_unsafe | clean_safe | contract_violated |
| --- | ---: | ---: | ---: | ---: |
| lenient | 170 | 22 | 14 | 0 |
| default | 220 | 14 | 22 | 0 |
| conservative | 270 | 9 | 27 | 0 |

## Search Efficiency

discrete bisection over D for each wind, replayed against completed grid run results. Queries to bracket boundaries: 11 vs full grid 36.

## Reproducibility

Repeated near-boundary points: 12; boundary flips: none.
For each run id in the field table, the committed audit files are `planc/logs/<run_id>_params.json`, `planc/logs/<run_id>_parsed.csv`, and `planc/logs/<run_id>_parsed.oracle.json`.

## Figures

- premise: ![](planc/analysis/rtl_energy_premise.png)
- result_field: ![](planc/analysis/rtl_energy_result_field.png)
- severity: ![](planc/analysis/rtl_energy_severity_heatmap.png)
- p_stratification: ![](planc/analysis/rtl_energy_p_stratification.png)
- train_test: ![](planc/analysis/rtl_energy_train_test.png)

## Limitations

This is still SITL, not HITL. The decisive verdict applies to this ArduCopter SITL energy model and the pre-registered RTL energy trap. Logs, parameter readbacks, parsed CSVs, and oracle sidecars are kept under `planc/logs/` for independent audit.

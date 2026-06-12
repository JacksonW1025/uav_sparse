VERDICT: PASS

# Minimum Altitude Fence Ground-Contact Scenario

## Decisive Criteria

- Premise satisfied: **True**
- Robust clean unsafe region: **True**
- Fence-triggered, contract-clean PGFUZZ-invisible gap: **True**
- Prediction gates passed: **True** (classification=True, severity=True)
- Reason: All decisive criteria are satisfied.

## Premise

Premise conclusion: **True** - height fence recovery, descent-rate application, and mass response all held.

| check | ok | detail |
|---|---:|---|
| mechanism_fence_RTL_then_danger_floor | True | `minalt_premise_mechanism_m125` |
| commanded_descent_rate_applied | True | `minalt_premise_descent_rate_m100` |
| mass_increase_lowers_min_agl | True | `[{"mass_kg": 3.0, "mass_multiplier": 1.0, "min_agl_m": 2.7503623962402344, "model_name": "m100", "run_id": "minalt_premise_mass_response_m100"}, {"mass_kg": 3.75, "mass_multiplier": 1.25, "min_agl_m": 2.6697158813476562, "model_name": "m125", "run_id": "minalt_premise_mass_response_m125"}, {"mass_kg": 4.2, "mass_multiplier": 1.4, "min_agl_m": 2.6068828105926514, "model_name": "m140", "run_id": "minalt_premise_mass_response_m140"}, {"mass_kg": 5.1, "mass_multiplier": 1.7, "min_agl_m": 2.442631959915161, "model_name": "m170", "run_id": "minalt_premise_mass_response_m170"}]` |

The fence floor is activated after takeoff because ArduCopter rejects arming below an enabled `FENCE_ALT_MIN`; the active P parameters are read back before the descent stimulus.

## Measurement Precision

`sigma_boundary=0.0140 m`, `sigma_unsafe=0.0040 m`, `d_margin=0.0419 m`, `h_floor=2.00 m`, `mae_bound=0.0210 m`.

Ambiguous points are excluded from classification only when their label CI crosses `h_floor`; they remain in severity-regression holdout accounting.

| point | label | mean min-AGL m | CI low | CI high | basis |
|---|---:|---:|---:|---:|---|
| default_d053_m100 | ambiguous | 1.992 | 1.964 | 2.020 | label_CI_crosses_h_floor |

## Three-Zone Field

Default layer counts: clean_safe=11, clean_unsafe=12, ambiguous=1, contract_violated=0, blocked=0.

| point | down m/s | mass x | mean min-AGL m | label | contract violations | runs |
|---|---:|---:|---:|---|---|---|
| default_d050_m100 | 5.0 | 1.00 | 2.750 | clean_safe | none | 5/5 |
| default_d050_m125 | 5.0 | 1.25 | 2.673 | clean_safe | none | 5/5 |
| default_d050_m140 | 5.0 | 1.40 | 2.607 | clean_safe | none | 5/5 |
| default_d050_m170 | 5.0 | 1.70 | 2.443 | clean_safe | none | 5/5 |
| default_d051_m100 | 5.1 | 1.00 | 2.500 | clean_safe | none | 5/5 |
| default_d051_m125 | 5.1 | 1.25 | 2.424 | clean_safe | none | 5/5 |
| default_d051_m140 | 5.1 | 1.40 | 2.348 | clean_safe | none | 5/5 |
| default_d051_m170 | 5.1 | 1.70 | 2.151 | clean_safe | none | 5/5 |
| default_d052_m100 | 5.2 | 1.00 | 2.237 | clean_safe | none | 5/5 |
| default_d052_m125 | 5.2 | 1.25 | 2.177 | clean_safe | none | 5/5 |
| default_d052_m140 | 5.2 | 1.40 | 2.091 | clean_safe | none | 5/5 |
| default_d052_m170 | 5.2 | 1.70 | 1.863 | clean_unsafe | none | 5/5 |
| default_d053_m100 | 5.3 | 1.00 | 1.992 | ambiguous | none | 5/5 |
| default_d053_m125 | 5.3 | 1.25 | 1.903 | clean_unsafe | none | 5/5 |
| default_d053_m140 | 5.3 | 1.40 | 1.830 | clean_unsafe | none | 5/5 |
| default_d053_m170 | 5.3 | 1.70 | 1.560 | clean_unsafe | none | 5/5 |
| default_d054_m100 | 5.4 | 1.00 | 1.735 | clean_unsafe | none | 5/5 |
| default_d054_m125 | 5.4 | 1.25 | 1.647 | clean_unsafe | none | 5/5 |
| default_d054_m140 | 5.4 | 1.40 | 1.585 | clean_unsafe | none | 5/5 |
| default_d054_m170 | 5.4 | 1.70 | 1.261 | clean_unsafe | none | 5/5 |
| default_d055_m100 | 5.5 | 1.00 | 1.490 | clean_unsafe | none | 5/5 |
| default_d055_m125 | 5.5 | 1.25 | 1.397 | clean_unsafe | none | 5/5 |
| default_d055_m140 | 5.5 | 1.40 | 1.332 | clean_unsafe | none | 5/5 |
| default_d055_m170 | 5.5 | 1.70 | 0.976 | clean_unsafe | none | 5/5 |

`CRASH` and ground-contact records are treated as unsafe outcome signals, not preventive contract violations. Preventive violations are parameter/readback errors, missing or mistimed min-alt fence action, unrelated failsafes, or unrelated dirty `STATUSTEXT`/`ERR` records.

## Prediction

Classification: interpolation accuracy=1.000, extrapolation accuracy=1.000, combined=1.000.
Severity regression: MAE=0.0105 m, bound=0.0210 m, pass=True.

## P Stratification

clean_unsafe count shrinks or stays flat as FENCE_ALT_MIN increases

| layer | FENCE_ALT_MIN m | clean_unsafe | clean_safe | ambiguous | contract_violated |
|---|---:|---:|---:|---:|---:|
| low | 4.0 | 22 | 1 | 1 | 0 |
| default | 5.0 | 12 | 11 | 1 | 0 |
| high | 7.0 | 0 | 24 | 0 | 0 |

## Search Efficiency

discrete bisection over descent rate for each mass, replayed against completed noise-aware grid results: 12 queries versus 24 full-grid points.

## Three-Dimensional Unified Claim

The three planc scenarios use the same threshold-insufficiency machine across three subsystems and dimensions: energy budget (`BATT_LOW_MAH`), time budget (`FS_GCS_TIMEOUT`), and height budget (`FENCE_ALT_MIN`). In this scenario the configured minimum-altitude fence triggers the specified RTL recovery, but the height budget can be insufficient under legal high descent rate and mass conditions.

## Limits

This remains SITL evidence. The unsafe consequence is defined by min-AGL relative to a 2 m danger floor and by ground-contact/CRASH outcome records. The `(b)` energy result remains the main result; this is the third subsystem generalization.

## Artifacts

- premise: `planc/analysis/minalt_premise.png`
- result_field: `planc/analysis/minalt_result_field.png`
- severity: `planc/analysis/minalt_severity_heatmap.png`
- p_stratification: `planc/analysis/minalt_p_stratification.png`
- train_test: `planc/analysis/minalt_train_test.png`
- severity_regression: `planc/analysis/minalt_severity_regression.png`
- Structured results: `planc/results/minalt_groundcontact_results.json`
- Parsed logs and sidecars: `planc/logs/minalt_*_params.json`, `planc/logs/minalt_*_parsed.csv`, `planc/logs/minalt_*_parsed.oracle.json`
- Local raw DataFlash logs: `planc/logs/minalt_*.BIN` (ignored by Git, retained in this workspace when present)

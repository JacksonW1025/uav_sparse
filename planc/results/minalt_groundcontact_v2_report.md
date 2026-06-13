VERDICT: INCONCLUSIVE

# Minimum Altitude Fence Ground-Contact Scenario

Experiment version: `minalt_groundcontact_v2_wide_range_turbulence`.

## Decisive Criteria

- Premise satisfied: **False**
- Robust clean unsafe region: **False**
- Fence-triggered, contract-clean PGFUZZ-invisible gap: **False**
- Prediction gates passed: **False** (classification=False, severity=False, meaningful_extrapolation=None)
- Reason: Premise failed: turbulence_raises_sigma_to_realistic_scale.

## Premise

Premise conclusion: **False** - one or more premise gates failed; verdict is not meaningful as a PASS/FAIL.

| check | ok | detail |
|---|---:|---|
| mechanism_fence_RTL_then_danger_floor | True | `minalt_v2_premise_mechanism_m125` |
| commanded_descent_rates_2_5_8_applied | True | `[{"audit": {"commanded_descent_rate_m_s": 2.0, "median_actual_down_m_s": 1.9349898099899292, "median_error_m_s": -0.0650101900100708, "p10_actual_down_m_s": 1.4340522289276123, "p90_actual_down_m_s": 1.9619898080825806, "representative_actual_down_m_s": 1.9619898080825806, "representative_error_m_s": -0.03801019191741939, "representative_quantile": 90.0, "samples": 137, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 2.0, "run_id": "minalt_v2_premise_descent_rate_d020_m100"}, {"audit": {"commanded_descent_rate_m_s": 5.0, "median_actual_down_m_s": 4.387680530548096, "median_error_m_s": -0.6123194694519043, "p10_actual_down_m_s": 2.18864221572876, "p90_actual_down_m_s": 4.77401180267334, "representative_actual_down_m_s": 4.77401180267334, "representative_error_m_s": -0.22598819732665998, "representative_quantile": 90.0, "samples": 65, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 5.0, "run_id": "minalt_v2_premise_descent_rate_d050_m100"}, {"audit": {"commanded_descent_rate_m_s": 8.0, "median_actual_down_m_s": 6.198633670806885, "median_error_m_s": -1.8013663291931152, "p10_actual_down_m_s": 2.0989843845367435, "p90_actual_down_m_s": 7.2785139083862305, "representative_actual_down_m_s": 7.2785139083862305, "representative_error_m_s": -0.7214860916137695, "representative_quantile": 90.0, "samples": 45, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 8.0, "run_id": "minalt_v2_premise_descent_rate_d080_m100"}]` |
| mass_increase_lowers_min_agl | True | `[{"completed_repetitions": 5, "mass_kg": 3.0, "mass_multiplier": 1.0, "min_agl_m": 1.9876914262771606, "model_name": "m100", "required_repetitions": 5, "run_ids": ["minalt_v2_premise_mass_response_r01_m100", "minalt_v2_premise_mass_response_r02_m100", "minalt_v2_premise_mass_response_r03_m100", "minalt_v2_premise_mass_response_r04_m100", "minalt_v2_premise_mass_response_r05_m100"], "sample_std_min_agl_m": 0.011012838682165054}, {"completed_repetitions": 5, "mass_kg": 3.75, "mass_multiplier": 1.25, "min_agl_m": 1.9025260210037231, "model_name": "m125", "required_repetitions": 5, "run_ids": ["minalt_v2_premise_mass_response_r01_m125", "minalt_v2_premise_mass_response_r02_m125", "minalt_v2_premise_mass_response_r03_m125", "minalt_v2_premise_mass_response_r04_m125", "minalt_v2_premise_mass_response_r05_m125"], "sample_std_min_agl_m": 0.006288896743264432}, {"completed_repetitions": 5, "mass_kg": 4.2, "mass_multiplier": 1.4, "min_agl_m": 1.8617003917694093, "model_name": "m140", "required_repetitions": 5, "run_ids": ["minalt_v2_premise_mass_response_r01_m140", "minalt_v2_premise_mass_response_r02_m140", "minalt_v2_premise_mass_response_r03_m140", "minalt_v2_premise_mass_response_r04_m140", "minalt_v2_premise_mass_response_r05_m140"], "sample_std_min_agl_m": 0.02760390083486434}, {"completed_repetitions": 5, "mass_kg": 5.1, "mass_multiplier": 1.7, "min_agl_m": 1.6660399675369262, "model_name": "m170", "required_repetitions": 5, "run_ids": ["minalt_v2_premise_mass_response_r01_m170", "minalt_v2_premise_mass_response_r02_m170", "minalt_v2_premise_mass_response_r03_m170", "minalt_v2_premise_mass_response_r04_m170", "minalt_v2_premise_mass_response_r05_m170"], "sample_std_min_agl_m": 0.02810888780343791}]` |
| turbulence_raises_sigma_to_realistic_scale | False | `sigma_boundary=0.0096 m, required [0.0500, 0.5000] m, raised_from_v1=False, complete=True, contract_clean=False` |

The fence floor is activated after takeoff because ArduCopter rejects arming below an enabled `FENCE_ALT_MIN`; the active P parameters are read back before the descent stimulus.
The descent-rate premise explicitly covers 2, 5, and 8 m/s with the down-speed limits set above 8 m/s, so this run audits the full wide range rather than a boundary-only band.

## Measurement Precision

Wind/turbulence settings: `{"SIM_WIND_DIR": 270.0, "SIM_WIND_SPD": 4.0, "SIM_WIND_TURB": 0.75}`.

`sigma_boundary=0.0096 m`, `sigma_unsafe=0.0141 m`, `d_margin=0.0287 m`, `h_floor=2.00 m`, `mae_bound=0.0143 m`.

Ambiguous points are excluded from classification only when their label CI crosses `h_floor`; they remain in severity-regression holdout accounting.

| noise group | point | mean min-AGL m | sample std m | repetitions | contract clean | violations |
|---|---|---:|---:|---:|---:|---|
| boundary | default_d050_m100 | 1.987 | 0.0047 | 5 | True | none |
| boundary | default_d053_m125 | 2.092 | 0.0111 | 5 | True | none |
| boundary | default_d060_m170 | 2.013 | 0.0113 | 5 | True | none |
| unsafe | default_d050_m170 | 1.650 | 0.0098 | 5 | True | none |
| unsafe | default_d070_m125 | 0.849 | 0.0129 | 5 | True | none |
| unsafe | default_d080_m170 | 0.708 | 0.0182 | 5 | False | other_ERR_subsystems |

| point | label | mean min-AGL m | CI low | CI high | basis |
|---|---:|---:|---:|---:|---|

## Three-Zone Field

Default layer counts: clean_safe=0, clean_unsafe=0, ambiguous=0, contract_violated=0, blocked=28.
The wide grid was not executed because the phase-0 premise gate was not satisfied; blocked rows below are scheduled grid points with zero completed repetitions.

| point | down m/s | mass x | mean min-AGL m | label | contract violations | runs |
|---|---:|---:|---:|---|---|---|
| default_d020_m100 | 2.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d020_m125 | 2.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d020_m140 | 2.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d020_m170 | 2.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d030_m100 | 3.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d030_m125 | 3.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d030_m140 | 3.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d030_m170 | 3.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d040_m100 | 4.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d040_m125 | 4.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d040_m140 | 4.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d040_m170 | 4.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d050_m100 | 5.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d050_m125 | 5.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d050_m140 | 5.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d050_m170 | 5.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d060_m100 | 6.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d060_m125 | 6.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d060_m140 | 6.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d060_m170 | 6.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d070_m100 | 7.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d070_m125 | 7.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d070_m140 | 7.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d070_m170 | 7.0 | 1.70 | n/a | blocked | none | 0/5 |
| default_d080_m100 | 8.0 | 1.00 | n/a | blocked | none | 0/5 |
| default_d080_m125 | 8.0 | 1.25 | n/a | blocked | none | 0/5 |
| default_d080_m140 | 8.0 | 1.40 | n/a | blocked | none | 0/5 |
| default_d080_m170 | 8.0 | 1.70 | n/a | blocked | none | 0/5 |

`CRASH` and ground-contact records are treated as unsafe outcome signals, not preventive contract violations. Preventive violations are parameter/readback errors, missing or mistimed min-alt fence action, unrelated failsafes, or unrelated dirty `STATUSTEXT`/`ERR` records.

## Prediction

Classification: interpolation accuracy=n/a, extrapolation accuracy=n/a, combined=n/a.
Extrapolation split: train descent rates [2.0, 3.0, 4.0, 5.0] (range 2.0-5.0 m/s), test descent rates [6.0, 7.0, 8.0] (range 6.0-8.0 m/s), gap above train max=1.0 m/s, test width=2.0 m/s, meaningful=True.
Severity regression: MAE=n/a m, bound=0.0143 m, pass=False.

## P Stratification

clean_unsafe count shrinks or stays flat as FENCE_ALT_MIN increases

| layer | FENCE_ALT_MIN m | clean_unsafe | clean_safe | ambiguous | contract_violated |
|---|---:|---:|---:|---:|---:|
| low | 4.0 | 0 | 0 | 0 | 0 |
| default | 5.0 | 0 | 0 | 0 | 0 |
| high | 7.0 | 0 | 0 | 0 | 0 |

## Search Efficiency

discrete bisection over descent rate for each mass, replayed against completed noise-aware grid results: 12 queries versus 28 full-grid points.

## Dual-Zone Comparison

v1 used descent rates [5.0, 5.5] m/s, `sigma_boundary=0.014 m`, and an extrapolation span of about 0.2 m/s. v2 uses descent rates [2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0] m/s with wind/turbulence enabled; the measured `sigma_boundary` is 0.0096 m and the reported extrapolation gap is 1.0 m/s over a 2.0 m/s held-out high-rate field.

## Three-Dimensional Unified Claim

The three planc scenarios use the same threshold-insufficiency machine across three subsystems and dimensions: energy budget (`BATT_LOW_MAH`), time budget (`FS_GCS_TIMEOUT`), and height budget (`FENCE_ALT_MIN`). In this scenario the configured minimum-altitude fence triggers the specified RTL recovery, but the height budget can be insufficient under legal high descent rate and mass conditions.

## Limits

This remains SITL evidence. The unsafe consequence is defined by min-AGL relative to a 2 m danger floor and by ground-contact/CRASH outcome records. The `(b)` energy result remains the main result; this is the third subsystem generalization.

## Artifacts

- premise: `planc/analysis/minalt_groundcontact_v2_premise.png`
- result_field: `planc/analysis/minalt_groundcontact_v2_result_field.png`
- severity: `planc/analysis/minalt_groundcontact_v2_severity_heatmap.png`
- p_stratification: `planc/analysis/minalt_groundcontact_v2_p_stratification.png`
- train_test: `planc/analysis/minalt_groundcontact_v2_train_test.png`
- severity_regression: `planc/analysis/minalt_groundcontact_v2_severity_regression.png`
- Structured results: `planc/results/minalt_groundcontact_v2_results.json`
- Oracle preregistration: `planc/results/minalt_groundcontact_v2_oracle_preregistered.json`
- Parsed logs and sidecars: `planc/logs/minalt_v2_*_params.json`, `planc/logs/minalt_v2_*_parsed.csv`, `planc/logs/minalt_v2_*_parsed.oracle.json`
- Local raw DataFlash logs: `planc/logs/minalt_v2_*.BIN` (ignored by Git, retained in this workspace when present)

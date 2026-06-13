VERDICT: FAIL

# Minimum Altitude Fence Ground-Contact Scenario

Experiment version: `minalt_groundcontact_v3_wide_range_deterministic`.

## Decisive Criteria

- Premise satisfied: **True**
- Robust clean unsafe region: **True**
- Fence-triggered, contract-clean PGFUZZ-invisible gap: **False**
- Prediction gates passed: **False** (classification=False, severity=False, meaningful_extrapolation=True)
- Reason: contract_violated or blocked point present, or clean_unsafe is not contract-clean; held-out classification is below 90% or lacks meaningful extrapolation; severity regression MAE exceeds the preregistered noise-scale bound

## Premise

Premise conclusion: **True** - height fence recovery, wide descent-rate application, mass response, and deterministic sigma reporting all held.

| check | ok | detail |
|---|---:|---|
| mechanism_fence_RTL_then_danger_floor | True | `minalt_v3_premise_mechanism_m125` |
| commanded_descent_rates_2_5_8_applied | True | `[{"audit": {"commanded_descent_rate_m_s": 2.0, "median_actual_down_m_s": 1.936177134513855, "median_error_m_s": -0.06382286548614502, "p10_actual_down_m_s": 1.43464515209198, "p90_actual_down_m_s": 1.970759391784668, "representative_actual_down_m_s": 1.970759391784668, "representative_error_m_s": -0.02924060821533203, "representative_quantile": 90.0, "samples": 137, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 2.0, "run_id": "minalt_v3_premise_descent_rate_d020_m100"}, {"audit": {"commanded_descent_rate_m_s": 5.0, "median_actual_down_m_s": 4.403005838394165, "median_error_m_s": -0.596994161605835, "p10_actual_down_m_s": 2.342682528495789, "p90_actual_down_m_s": 4.777486801147461, "representative_actual_down_m_s": 4.777486801147461, "representative_error_m_s": -0.22251319885253906, "representative_quantile": 90.0, "samples": 64, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 5.0, "run_id": "minalt_v3_premise_descent_rate_d050_m100"}, {"audit": {"commanded_descent_rate_m_s": 8.0, "median_actual_down_m_s": 6.209625720977783, "median_error_m_s": -1.7903742790222168, "p10_actual_down_m_s": 2.0927073001861576, "p90_actual_down_m_s": 7.272966098785401, "representative_actual_down_m_s": 7.272966098785401, "representative_error_m_s": -0.7270339012145994, "representative_quantile": 90.0, "samples": 45, "source": "XKF1 primary core VD in NED frame, between commanded descent start and FENCE_ALT_MIN breach", "within_tolerance": true}, "model_name": "m100", "ok": true, "requested_descent_rate_m_s": 8.0, "run_id": "minalt_v3_premise_descent_rate_d080_m100"}]` |
| mass_increase_lowers_min_agl | True | `[{"completed_repetitions": 5, "mass_kg": 3.0, "mass_multiplier": 1.0, "min_agl_m": 1.9004999160766602, "model_name": "m100", "required_repetitions": 5, "run_ids": ["minalt_v3_premise_mass_response_r01_m100", "minalt_v3_premise_mass_response_r02_m100", "minalt_v3_premise_mass_response_r03_m100", "minalt_v3_premise_mass_response_r04_m100", "minalt_v3_premise_mass_response_r05_m100"], "sample_std_min_agl_m": 0.09833223438509035}, {"completed_repetitions": 5, "mass_kg": 3.75, "mass_multiplier": 1.25, "min_agl_m": 1.9125389575958252, "model_name": "m125", "required_repetitions": 5, "run_ids": ["minalt_v3_premise_mass_response_r01_m125", "minalt_v3_premise_mass_response_r02_m125", "minalt_v3_premise_mass_response_r03_m125", "minalt_v3_premise_mass_response_r04_m125", "minalt_v3_premise_mass_response_r05_m125"], "sample_std_min_agl_m": 0.008826743502689017}, {"completed_repetitions": 5, "mass_kg": 4.2, "mass_multiplier": 1.4, "min_agl_m": 1.8573265075683594, "model_name": "m140", "required_repetitions": 5, "run_ids": ["minalt_v3_premise_mass_response_r01_m140", "minalt_v3_premise_mass_response_r02_m140", "minalt_v3_premise_mass_response_r03_m140", "minalt_v3_premise_mass_response_r04_m140", "minalt_v3_premise_mass_response_r05_m140"], "sample_std_min_agl_m": 0.08821183210319393}, {"completed_repetitions": 5, "mass_kg": 5.1, "mass_multiplier": 1.7, "min_agl_m": 1.6457924842834473, "model_name": "m170", "required_repetitions": 5, "run_ids": ["minalt_v3_premise_mass_response_r01_m170", "minalt_v3_premise_mass_response_r02_m170", "minalt_v3_premise_mass_response_r03_m170", "minalt_v3_premise_mass_response_r04_m170", "minalt_v3_premise_mass_response_r05_m170"], "sample_std_min_agl_m": 0.059137826723342266}]` |
| deterministic_sigma_reported_not_premise_gate | True | `sigma_boundary=0.0866 m; deterministic min-AGL outcome accepted, so sigma magnitude is reported but not a premise gate; measurement_complete=True, contract_clean=False` |

The fence floor is activated after takeoff because ArduCopter rejects arming below an enabled `FENCE_ALT_MIN`; the active P parameters are read back before the descent stimulus.
The descent-rate premise explicitly covers 2, 5, and 8 m/s with the down-speed limits set above 8 m/s, so this run audits the full wide range rather than a boundary-only band.
The mass-response check uses repeated means with the existing run-to-run tolerance; the per-mass means and standard deviations above are the audit record for strict monotonicity.

## Measurement Precision

Wind/turbulence settings: `{"SIM_WIND_DIR": 270.0, "SIM_WIND_SPD": 0.0, "SIM_WIND_TURB": 0.0}`.

`sigma_boundary=0.0866 m`, `sigma_unsafe=0.0408 m`, `d_margin=0.2597 m`, `h_floor=2.00 m`, `mae_bound=0.1298 m`.

Ambiguous points are excluded from classification only when their label CI crosses `h_floor`; they remain in severity-regression holdout accounting.

## Honest Positioning

This SITL consequence is intrinsically near deterministic for vertical min-AGL: horizontal wind does not materially couple into the vertical pull-up dynamics, so `sigma_boundary` stays at centimeter scale and the noise-aware oracle is nearly idle here. The extrapolation evidence below is therefore a wide-range deterministic demonstration, not a claim of predicting through realistic consequence noise.

| noise group | point | mean min-AGL m | sample std m | repetitions | contract clean | violations |
|---|---|---:|---:|---:|---:|---|
| boundary | default_d050_m100 | 1.933 | 0.0933 | 3 | True | none |
| boundary | default_d053_m125 | 2.053 | 0.1167 | 3 | True | none |
| boundary | default_d060_m170 | 2.031 | 0.0117 | 3 | True | none |
| unsafe | default_d050_m170 | 1.698 | 0.0331 | 3 | True | none |
| unsafe | default_d070_m125 | 0.933 | 0.0545 | 3 | True | none |
| unsafe | default_d080_m170 | 0.804 | 0.0304 | 3 | False | other_ERR_subsystems |

| point | label | mean min-AGL m | CI low | CI high | basis |
|---|---:|---:|---:|---:|---|
| default_d050_m100 | ambiguous | 1.995 | 1.822 | 2.168 | label_CI_crosses_h_floor |
| default_d050_m125 | ambiguous | 1.956 | 1.783 | 2.129 | label_CI_crosses_h_floor |
| default_d050_m140 | ambiguous | 1.935 | 1.762 | 2.108 | label_CI_crosses_h_floor |
| default_d060_m170 | ambiguous | 2.021 | 1.847 | 2.194 | label_CI_crosses_h_floor |
| default_d065_m170 | ambiguous | 1.842 | 1.669 | 2.015 | label_CI_crosses_h_floor |

## Three-Zone Field

Default layer counts: clean_safe=24, clean_unsafe=10, ambiguous=5, contract_violated=1, blocked=0.
Preventive contract violation audit: 1/40 default points and 3/120 default runs were violated; locations=['default_d080_m170']; violations=['other_ERR_subsystems']; other_ERR_subsystems=['THRUST_LOSS_CHECK'].

| point | down m/s | mass x | mean min-AGL m | label | contract violations | runs |
|---|---:|---:|---:|---|---|---|
| default_d020_m100 | 2.0 | 1.00 | 3.289 | clean_safe | none | 3/3 |
| default_d020_m125 | 2.0 | 1.25 | 3.213 | clean_safe | none | 3/3 |
| default_d020_m140 | 2.0 | 1.40 | 3.165 | clean_safe | none | 3/3 |
| default_d020_m170 | 2.0 | 1.70 | 3.216 | clean_safe | none | 3/3 |
| default_d030_m100 | 3.0 | 1.00 | 3.517 | clean_safe | none | 3/3 |
| default_d030_m125 | 3.0 | 1.25 | 3.175 | clean_safe | none | 3/3 |
| default_d030_m140 | 3.0 | 1.40 | 3.103 | clean_safe | none | 3/3 |
| default_d030_m170 | 3.0 | 1.70 | 3.424 | clean_safe | none | 3/3 |
| default_d040_m100 | 4.0 | 1.00 | 3.246 | clean_safe | none | 3/3 |
| default_d040_m125 | 4.0 | 1.25 | 3.187 | clean_safe | none | 3/3 |
| default_d040_m140 | 4.0 | 1.40 | 3.081 | clean_safe | none | 3/3 |
| default_d040_m170 | 4.0 | 1.70 | 3.122 | clean_safe | none | 3/3 |
| default_d050_m100 | 5.0 | 1.00 | 1.995 | ambiguous | none | 3/3 |
| default_d050_m125 | 5.0 | 1.25 | 1.956 | ambiguous | none | 3/3 |
| default_d050_m140 | 5.0 | 1.40 | 1.935 | ambiguous | none | 3/3 |
| default_d050_m170 | 5.0 | 1.70 | 1.689 | clean_unsafe | none | 3/3 |
| default_d055_m100 | 5.5 | 1.00 | 2.937 | clean_safe | none | 3/3 |
| default_d055_m125 | 5.5 | 1.25 | 2.916 | clean_safe | none | 3/3 |
| default_d055_m140 | 5.5 | 1.40 | 2.809 | clean_safe | none | 3/3 |
| default_d055_m170 | 5.5 | 1.70 | 2.470 | clean_safe | none | 3/3 |
| default_d060_m100 | 6.0 | 1.00 | 2.776 | clean_safe | none | 3/3 |
| default_d060_m125 | 6.0 | 1.25 | 2.687 | clean_safe | none | 3/3 |
| default_d060_m140 | 6.0 | 1.40 | 2.584 | clean_safe | none | 3/3 |
| default_d060_m170 | 6.0 | 1.70 | 2.021 | ambiguous | none | 3/3 |
| default_d065_m100 | 6.5 | 1.00 | 2.827 | clean_safe | none | 3/3 |
| default_d065_m125 | 6.5 | 1.25 | 2.764 | clean_safe | none | 3/3 |
| default_d065_m140 | 6.5 | 1.40 | 2.504 | clean_safe | none | 3/3 |
| default_d065_m170 | 6.5 | 1.70 | 1.842 | ambiguous | none | 3/3 |
| default_d070_m100 | 7.0 | 1.00 | 0.976 | clean_unsafe | none | 3/3 |
| default_d070_m125 | 7.0 | 1.25 | 0.848 | clean_unsafe | none | 3/3 |
| default_d070_m140 | 7.0 | 1.40 | 0.512 | clean_unsafe | none | 3/3 |
| default_d070_m170 | 7.0 | 1.70 | 0.312 | clean_unsafe | none | 3/3 |
| default_d075_m100 | 7.5 | 1.00 | 1.561 | clean_unsafe | none | 3/3 |
| default_d075_m125 | 7.5 | 1.25 | 1.357 | clean_unsafe | none | 3/3 |
| default_d075_m140 | 7.5 | 1.40 | 0.859 | clean_unsafe | none | 3/3 |
| default_d075_m170 | 7.5 | 1.70 | 0.362 | clean_unsafe | none | 3/3 |
| default_d080_m100 | 8.0 | 1.00 | 2.475 | clean_safe | none | 3/3 |
| default_d080_m125 | 8.0 | 1.25 | 2.212 | clean_safe | none | 3/3 |
| default_d080_m140 | 8.0 | 1.40 | 1.760 | clean_unsafe | none | 3/3 |
| default_d080_m170 | 8.0 | 1.70 | 0.731 | contract_violated | other_ERR_subsystems | 3/3 |

`CRASH` and ground-contact records are treated as unsafe outcome signals, not preventive contract violations. Preventive violations are parameter/readback errors, missing or mistimed min-alt fence action, unrelated failsafes, or unrelated dirty `STATUSTEXT`/`ERR` records.

## Prediction

Classification: interpolation accuracy=0.800, extrapolation accuracy=0.571, combined=0.632.
Extrapolation split: train descent rates [2.0, 3.0, 4.0, 5.0, 5.5, 6.0] (range 2.0-6.0 m/s), test descent rates [6.5, 7.0, 7.5, 8.0] (range 6.5-8.0 m/s), gap above train max=0.5 m/s, held-out high-rate span=1.5 m/s, meaningful=True.
Severity regression: MAE=0.6413 m, bound=0.1298 m, pass=False.

## P Stratification

clean_unsafe count shrinks or stays flat as FENCE_ALT_MIN increases

| layer | FENCE_ALT_MIN m | clean_unsafe | clean_safe | ambiguous | contract_violated |
|---|---:|---:|---:|---:|---:|
| low | 4.0 | 25 | 8 | 6 | 1 |
| default | 5.0 | 10 | 24 | 5 | 1 |
| high | 7.0 | 1 | 35 | 1 | 3 |

## Search Efficiency

discrete bisection over descent rate for each mass, replayed against completed oracle-labeled grid results: 16 queries versus 40 full-grid points.

## Dual-Zone Comparison

v1 used descent rates [5.0, 5.5] m/s, `sigma_boundary=0.014 m`, and an extrapolation span of about 0.2 m/s. v3 uses descent rates [2.0, 3.0, 4.0, 5.0, 5.5, 6.0, 6.5, 7.0, 7.5, 8.0] m/s with wind/turbulence disabled; the measured `sigma_boundary` is 0.0866 m and the reported extrapolation gap is 0.5 m/s over a 1.5 m/s held-out high-rate field.

## Methodological Finding

To make a consequence noisy, the injected disturbance must couple into that consequence's dynamics: horizontal wind can couple to horizontal excursion, but it does not couple strongly to vertical min-AGL recovery. Some consequences are effectively deterministic, so their extrapolation evidence has to come from range rather than noise.

## Three-Dimensional Unified Claim

The three planc scenarios use the same threshold-insufficiency machine across three subsystems and dimensions: energy budget (`BATT_LOW_MAH`), time budget (`FS_GCS_TIMEOUT`), and height budget (`FENCE_ALT_MIN`). In this scenario the configured minimum-altitude fence triggers the specified RTL recovery, but the height budget can be insufficient under legal high descent rate and mass conditions. The height extrapolation is now exercised over the wide 2-8 m/s descent-rate range in a clearly marked deterministic region.

## Limits

This remains SITL evidence. The unsafe consequence is defined by min-AGL relative to a 2 m danger floor and by ground-contact/CRASH outcome records. The `(b)` energy result remains the main result; scenario 2 remains the noise-aware oracle's live leg, while this minimum-altitude run is deterministic height-dimension evidence.

## Artifacts

- premise: `planc/analysis/minalt_groundcontact_v3_premise.png`
- result_field: `planc/analysis/minalt_groundcontact_v3_result_field.png`
- severity: `planc/analysis/minalt_groundcontact_v3_severity_heatmap.png`
- p_stratification: `planc/analysis/minalt_groundcontact_v3_p_stratification.png`
- train_test: `planc/analysis/minalt_groundcontact_v3_train_test.png`
- severity_regression: `planc/analysis/minalt_groundcontact_v3_severity_regression.png`
- Structured results: `planc/results/minalt_groundcontact_v3_results.json`
- Oracle preregistration: `planc/results/minalt_groundcontact_v3_oracle_preregistered.json`
- Parsed logs and sidecars: `planc/logs/minalt_v3_*_params.json`, `planc/logs/minalt_v3_*_parsed.csv`, `planc/logs/minalt_v3_*_parsed.oracle.json`
- Local raw DataFlash logs: `planc/logs/minalt_v3_*.BIN` (ignored by Git, retained in this workspace when present)

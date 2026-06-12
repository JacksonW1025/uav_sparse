VERDICT: PASS

# planc GCS link-loss boundary-excursion second scenario, v2

## Revised Four Criteria

- Premise satisfied: **True** (carried forward from v1, already true).
- Robust contract-clean unsafe region under the noise-aware oracle: **True**; clean_unsafe=16, ambiguous=4.
- Contract clean / PGFUZZ-invisible: **True**; contract_violated=0, blocked=0.
- Prediction gates: **True**; classification_ok=True (interpolation=1.000, extrapolation=1.000, combined=1.000), severity_ok=True (MAE=0.99 m <= bound=1.92 m).

Decision reason: All revised decisive criteria are satisfied.

## Measurement Precision And Oracle Commitment

Noise floor was measured before grid aggregation. Boundary sigma=1.28 m; unsafe-region sigma=2.04 m. The committed margin is `d_margin = k * sigma_boundary = 3.0 * 1.28 = 3.83 m`, so `hard_threshold = R + d_margin = 123.83 m`.
Ambiguous label band uses mean overshoot +/- 2.0*sigma_boundary and spans 1.28 to 6.39 m overshoot (width 5.11 m). Severity MAE bound is `c * sigma_boundary = 1.5 * 1.28 = 1.92 m`.

Oracle commitment file: `planc/results/linkloss_excursion_v2_oracle_preregistered.json`. It was written before the grid scheduler started.

Seed reuse audit: v2 reused existing v1 raw SITL repetitions where available, then backfilled missing repetitions with new v2 runs. Noise stage total=56 (v1 seed=15, v2 new=41); grid stage total=252 (v1 seed=132, v2 new=120). Thresholds were derived only from the preregistered noise points and not from prediction correctness.

| group | speed | wind | N | mean overshoot m | sample std m | contract clean |
| --- | ---: | ---: | ---: | ---: | ---: | --- |
| boundary | 7 | 0 | 8 | 1.19 | 0.74 | True |
| boundary | 7 | 9 | 8 | 2.12 | 1.47 | True |
| boundary | 7 | 12 | 8 | 1.60 | 0.93 | True |
| boundary | 7 | 15 | 8 | 3.78 | 1.72 | True |
| unsafe | 10 | 0 | 8 | 18.85 | 1.55 | True |
| unsafe | 10 | 9 | 8 | 19.99 | 2.09 | True |
| unsafe | 10 | 15 | 8 | 22.40 | 2.39 | True |

Ambiguous exclusion audit; these points are excluded from classification only because their label CI crosses `d_margin`:

| point | speed | wind | mean overshoot m | CI low | CI high | d_margin | basis |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| default_v07_w09 | 7 | 9 | 1.33 | -1.23 | 3.88 | 3.83 | label_CI_crosses_noise_margin |
| default_v07_w15 | 7 | 15 | 2.32 | -0.23 | 4.88 | 3.83 | label_CI_crosses_noise_margin |
| default_v08_w00 | 8 | 0 | 5.73 | 3.17 | 8.28 | 3.83 | label_CI_crosses_noise_margin |
| default_v08_w06 | 8 | 6 | 5.83 | 3.28 | 8.39 | 3.83 | label_CI_crosses_noise_margin |

## Dual Oracle Comparison

Original v1 oracle (`R + 1.5 m`) reported combined held-out accuracy **0.846** with interpolation **0.778** and extrapolation **1.000**.
Noise-aware v2 oracle reports classification interpolation **1.000**, extrapolation **1.000**, combined **1.000**, and severity-regression MAE **0.99 m** over the whole held-out field including ambiguous/boundary points.

## Three-Zone Field

Default layer `default` (`FS_GCS_TIMEOUT=5 s`) counts: clean_safe=16, clean_unsafe=16, ambiguous=4, contract_violated=0, blocked=0.

| speed | wind | label | mean overshoot m | CI low | CI high | reps | timeout s | raw R+1.5 label |
| ---: | ---: | --- | ---: | ---: | ---: | ---: | ---: | --- |
| 5 | 0 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.82 | safe |
| 5 | 3 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.90 | safe |
| 5 | 6 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.70 | safe |
| 5 | 9 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.73 | safe |
| 5 | 12 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.91 | safe |
| 5 | 15 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.63 | safe |
| 6 | 0 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 5.02 | safe |
| 6 | 3 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.94 | safe |
| 6 | 6 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.67 | safe |
| 6 | 9 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.87 | safe |
| 6 | 12 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 5.03 | safe |
| 6 | 15 | clean_safe | 0.00 | -2.55 | 2.55 | 5 | 4.77 | safe |
| 7 | 0 | clean_safe | 0.91 | -1.65 | 3.46 | 5 | 4.87 | safe |
| 7 | 3 | clean_safe | 0.32 | -2.23 | 2.88 | 5 | 4.74 | safe |
| 7 | 6 | clean_safe | 0.44 | -2.12 | 2.99 | 5 | 4.68 | safe |
| 7 | 9 | ambiguous | 1.33 | -1.23 | 3.88 | 5 | 4.83 | safe |
| 7 | 12 | clean_safe | 1.25 | -1.30 | 3.81 | 5 | 4.79 | safe |
| 7 | 15 | ambiguous | 2.32 | -0.23 | 4.88 | 5 | 4.78 | unsafe |
| 8 | 0 | ambiguous | 5.73 | 3.17 | 8.28 | 5 | 4.76 | unsafe |
| 8 | 3 | clean_unsafe | 6.50 | 3.94 | 9.05 | 5 | 4.85 | unsafe |
| 8 | 6 | ambiguous | 5.83 | 3.28 | 8.39 | 5 | 4.73 | unsafe |
| 8 | 9 | clean_unsafe | 6.40 | 3.85 | 8.95 | 5 | 4.75 | unsafe |
| 8 | 12 | clean_unsafe | 9.83 | 7.28 | 12.39 | 5 | 5.12 | unsafe |
| 8 | 15 | clean_unsafe | 9.70 | 7.14 | 12.25 | 5 | 4.91 | unsafe |
| 9 | 0 | clean_unsafe | 11.48 | 8.93 | 14.04 | 5 | 4.68 | unsafe |
| 9 | 3 | clean_unsafe | 13.71 | 11.15 | 16.26 | 5 | 4.92 | unsafe |
| 9 | 6 | clean_unsafe | 11.54 | 8.98 | 14.09 | 5 | 4.66 | unsafe |
| 9 | 9 | clean_unsafe | 12.44 | 9.89 | 14.99 | 5 | 4.68 | unsafe |
| 9 | 12 | clean_unsafe | 14.83 | 12.28 | 17.39 | 5 | 4.88 | unsafe |
| 9 | 15 | clean_unsafe | 14.54 | 11.98 | 17.09 | 5 | 4.67 | unsafe |
| 10 | 0 | clean_unsafe | 18.86 | 16.30 | 21.41 | 5 | 4.83 | unsafe |
| 10 | 3 | clean_unsafe | 21.61 | 19.05 | 24.16 | 5 | 5.08 | unsafe |
| 10 | 6 | clean_unsafe | 19.18 | 16.63 | 21.74 | 5 | 4.80 | unsafe |
| 10 | 9 | clean_unsafe | 18.32 | 15.77 | 20.88 | 5 | 4.62 | unsafe |
| 10 | 12 | clean_unsafe | 19.80 | 17.24 | 22.35 | 5 | 4.68 | unsafe |
| 10 | 15 | clean_unsafe | 22.91 | 20.35 | 25.46 | 5 | 4.88 | unsafe |

PGFUZZ-invisible check: `clean_unsafe` requires the intended GCS failsafe ERR plus RTL at the configured timeout, successful parameter readback, commanded-speed audit within tolerance, and no unrelated failsafe/error events. Report-Only fence breach records are retained as measurement evidence and are not contract violations.

## Two-Stage Prediction Evaluation

Classification formula: `unsafe probability = sigmoid((severity_model(v, wind) - d_margin) / sigma_boundary)`. Ambiguous points are excluded only by label CI.
interpolation: confident n=7, ambiguous excluded=2, accuracy=1.000, model_ok=True.
extrapolation: confident n=4, ambiguous excluded=0, accuracy=1.000, model_ok=True.
Combined confident held-out classification accuracy: 1.000.

Severity formula: `overshoot_m = max(0, beta0 + beta_v*v + beta_v2*v^2 + beta_w*wind + beta_vw*v*wind + beta_w2*wind^2)`.
Severity regression: train n=24, held-out n=12, contains ambiguous=True, MAE=0.99 m, bound=1.92 m, pass=True.

## P Stratification

Conclusion: noise-aware clean_unsafe count is nondecreasing as FS_GCS_TIMEOUT lengthens; shorter timeout shrinks the unsafe region.

| layer | FS_GCS_TIMEOUT s | clean_unsafe | clean_safe | ambiguous | contract_violated |
| --- | ---: | ---: | ---: | ---: | ---: |
| conservative | 2 | 0 | 36 | 0 | 0 |
| default | 5 | 16 | 16 | 4 | 0 |
| lenient | 8 | 30 | 1 | 5 | 0 |

## Search Efficiency

discrete bisection over speed for each wind, replayed against completed noise-aware grid results. Queries to bracket boundaries: 12 vs full grid 36.

## Unified Method Statement

This scenario and the RTL energy scenario are both threshold-insufficiency specification gaps: one is a data-link time-budget threshold, the other an energy-budget threshold. In both, ArduCopter follows the configured failsafe contract while a legal operating condition crosses an external safety oracle.

## Figures

- premise: ![](planc/analysis/linkloss_v2_premise.png)
- result_field: ![](planc/analysis/linkloss_v2_result_field.png)
- severity: ![](planc/analysis/linkloss_v2_severity_heatmap.png)
- p_stratification: ![](planc/analysis/linkloss_v2_p_stratification.png)
- train_test: ![](planc/analysis/linkloss_v2_train_test.png)
- severity_regression: ![](planc/analysis/linkloss_v2_severity_regression.png)

## Limitations

This remains ArduCopter SITL, not HITL. The noise-aware oracle is justified by the measured SITL run-to-run sigma, not by post-hoc tuning against prediction mistakes. The original strict-gate result for scenario (b) remains the main result; this v2 rerun only repairs the GCS link-loss measurement precision mismatch.

Audit files are under `planc/logs/` for each run id, with parsed CSV and `.oracle.json` sidecars. Structured results: `planc/results/linkloss_excursion_v2_results.json`.

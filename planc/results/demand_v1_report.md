# Demand v1 Reparameterization Probe

VERDICT: **COMPLEX-BOUNDARY**

No eligible achieved-trajectory Phi reaches the preregistered monotonicity threshold.

## Best Phi

- Phi: `actual_rate_peak_deg_s`.
- AUC: 0.797; Spearman rho: 0.444.
- Threshold Phi*: `>= 431.63`; balanced accuracy 0.795.
- Baseline r AUC: 0.615; r Spearman rho: 0.173.

## R Axis Shape

| r | n | clean_unsafe | recovered | safe | p_clean |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 120 | 5 | 2 | 2 | 1 | 0.40 |
| 180 | 10 | 10 | 0 | 0 | 1.00 |
| 240 | 5 | 4 | 1 | 0 | 0.80 |
| 300 | 5 | 3 | 0 | 2 | 0.60 |
| 360 | 10 | 4 | 3 | 3 | 0.40 |
| 420 | 5 | 3 | 2 | 0 | 0.60 |
| 480 | 10 | 9 | 1 | 0 | 0.90 |
| 540 | 5 | 5 | 0 | 0 | 1.00 |
| 600 | 10 | 9 | 1 | 0 | 0.90 |

## Phi Bins

| bin | Phi min | Phi max | n | p_clean |
| ---: | ---: | ---: | ---: | ---: |
| 0 | 125.47 | 250.80 | 8 | 0.62 |
| 1 | 252.81 | 370.08 | 8 | 0.50 |
| 2 | 370.09 | 372.07 | 8 | 0.38 |
| 3 | 429.78 | 492.46 | 8 | 0.75 |
| 4 | 492.51 | 620.07 | 9 | 0.89 |
| 5 | 620.28 | 972.63 | 8 | 0.88 |
| 6 | 1027.07 | 1235.92 | 8 | 1.00 |
| 7 | 1242.37 | 1482.01 | 8 | 1.00 |

## Top Candidate Phi

| feature | eligible | AUC | rho | threshold |
| --- | --- | ---: | ---: | --- |
| actual_rate_peak_deg_s | True | 0.797 | 0.444 | >= 431.63 |
| actual_rate_rms_deg_s | True | 0.765 | 0.396 | >= 180.24 |
| desired_rate_peak_deg_s | True | 0.760 | 0.388 | >= 471.92 |
| actual_rate_p95_deg_s | True | 0.721 | 0.329 | >= 444.98 |
| desired_rate_p95_deg_s | True | 0.705 | 0.306 | >= 438.20 |
| desired_rate_impulse_deg | True | 0.659 | 0.238 | >= 1152.71 |
| actual_roll_range_maneuver_deg | True | 0.658 | 0.236 | >= 330.10 |
| actual_rate_impulse_deg | True | 0.647 | 0.219 | >= 1205.75 |
| command_high_rate_time_s | True | 0.615 | 0.173 | <= 2.00 |
| command_peak_accel_proxy_deg_s2 | True | 0.615 | 0.173 | >= 24000.00 |
| command_rate_impulse_deg | True | 0.615 | 0.173 | >= 960.00 |
| actual_roll_abs_peak_maneuver_deg | True | 0.611 | 0.166 | >= 177.61 |

## E/P And Reachability

- New E/P SITL confirmation grid was not run in this analysis-short-circuit pass.
- Available hard-A evidence covers only default turbulence and ANGLE_MAX=4500.
- Because no eligible Phi reaches the monotonicity threshold, Phi* is not defined well enough for an E/P movement test in this branch.
- The preregistered E/P grid and non-doublet reachability inputs are still recorded in `demand_v1_prereg.json` if a later Phi family is proposed.

## Artifacts

- Preregistration: `planc/results/demand_v1_prereg.json`
- Result JSON: `planc/results/demand_v1_result.json`
- Plot r_w_shape: `planc/analysis/demand_v1_r_w_shape.png`
- Plot phi_ordering: `planc/analysis/demand_v1_phi_ordering.png`
- Plot phi_monotonicity: `planc/analysis/demand_v1_phi_monotonicity.png`
- Plot feature_auc: `planc/analysis/demand_v1_feature_auc.png`

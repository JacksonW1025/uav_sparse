# MarginSearch Stage 1-redo Report

Scope: `px4_position`, seed 0, Point V alpha=0.44921875.

## Point V

| item | value |
| --- | ---: |
| xy_velocity J mean rho | 0.058558 |
| interior FD clean groups at delta=0.08 | 40/40 |
| probe value-margin > 0.2 groups | 40/40 |
| probe projection-hit groups | 19 |

## Delta 0.2 Directional Sensitivity

| metric | value |
| --- | ---: |
| channel participation ratio / 4 | 2.621 |
| window participation ratio / 10 | 7.619 |
| active channels top80 | pitch,roll |
| active windows top80 | 9,8,7,6,5,4 |
| top8 groups | 37,36,33,32,29,20,25,24 |

Sensitivity distribution uses `abs(mean rho(+0.2) - mean rho(-0.2)) / 0.4`.

| distribution | min | p25 | median | p75 | p90 | p95 | max |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| distribution_abs_delta_rho_span | 0.001277 | 0.021938 | 0.042073 | 0.135584 | 0.240977 | 0.320266 | 0.377253 |
| distribution_directional_sensitivity | 0.003192 | 0.054844 | 0.105184 | 0.338960 | 0.602443 | 0.800664 | 0.943133 |
| distribution_max_abs_delta_rho_from_base | 0.023974 | 0.061204 | 0.103373 | 0.323592 | 0.379938 | 0.546101 | 0.589263 |

## Eff Sparsity Vs Rho

| alpha | property | rho mean | eff sparsity | top8 | noise/sum | channel PR | window PR | active channels | active windows | top8 groups |
| ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |
| 0.000000 | post_neutral_xy_velocity | 0.972872 | 27.281 | 0.406 | 1.255 | 3.967 | 8.659 | throttle,yaw,pitch,roll | 5,7,4,8,9,0,2 | 29,22,20,3,35,30,38,17 |
| 0.000000 | post_neutral_alt_drift | 0.989478 | 23.232 | 0.488 | 1.278 | 3.582 | 8.945 | throttle,yaw,pitch | 1,5,9,3,4,8,6 | 39,7,23,25,15,34,38,20 |
| 0.150000 | post_neutral_xy_velocity | 0.955479 | 16.835 | 0.587 | 0.619 | 3.352 | 6.422 | roll,pitch,yaw | 9,8,5,7,6,1 | 36,32,33,37,20,25,29,22 |
| 0.150000 | post_neutral_alt_drift | 0.994446 | 11.800 | 0.542 | 0.986 | 3.365 | 7.472 | throttle,yaw,roll | 9,8,6,1,2,7,5 | 39,10,35,26,4,18,27,33 |
| 0.250000 | post_neutral_xy_velocity | 0.619239 | 20.589 | 0.519 | 0.490 | 2.967 | 7.690 | pitch,roll,yaw | 9,5,8,7,6,2 | 36,37,33,28,21,24,20,22 |
| 0.250000 | post_neutral_alt_drift | 0.975598 | 9.392 | 0.611 | 0.786 | 3.516 | 5.045 | throttle,roll,pitch | 9,8,6,5,0,1 | 39,37,32,35,24,4,22,20 |
| 0.350000 | post_neutral_xy_velocity | 0.426906 | 18.046 | 0.573 | 0.552 | 2.715 | 7.855 | pitch,roll | 9,7,8,0,3,2,5 | 37,36,33,29,28,32,1,25 |
| 0.350000 | post_neutral_alt_drift | 0.945844 | 8.254 | 0.566 | 0.735 | 3.123 | 5.107 | throttle,yaw,roll | 9,2,1,5,3,6 | 39,8,7,11,10,37,12,21 |
| 0.449219 | post_neutral_xy_velocity | 0.058558 | 21.583 | 0.516 | 0.680 | 3.196 | 8.307 | pitch,roll,throttle | 9,8,7,1,3,4,5 | 28,32,37,33,36,3,13,17 |
| 0.449219 | post_neutral_alt_drift | 0.910444 | 5.180 | 0.652 | 0.588 | 2.467 | 3.944 | throttle,yaw,pitch | 9,7,1,5,3,4 | 39,31,10,6,27,34,14,15 |

## Artifacts

- summary JSON: `runs/margin_stage1_redo_v1/reports/stage1_redo_summary.json`
- theta_V: `runs/margin_stage1_redo_v1/theta_V.npy`
- directional_probe: `runs/margin_stage1_redo_v1/reports/redo_pointV_delta020_directional_probe.csv`
- channel_marginal: `runs/margin_stage1_redo_v1/reports/redo_pointV_delta020_channel_marginal.csv`
- window_marginal: `runs/margin_stage1_redo_v1/reports/redo_pointV_delta020_window_marginal.csv`
- alpha_curve_metrics: `runs/margin_stage1_redo_v1/reports/redo_alpha_fd_curve_metrics.csv`

Elapsed wall time: 19315.9s.

Stop point: Stage 2 not started.

# Cascade Recheck Report

Label: exploratory / verification. Platform: PX4. Scenario: px4_position. Seeds: 0/1/2. J=5.

## Metric Scope

- Original Direction A/ddmin labels are the archived `post_neutral_xy_velocity` 2sigma labels.
- Terminal metric is absolute [11,13] s peak |v_xy| with threshold 1.0 m/s: rho = 1.0 - peak.
- Four profile windows use the same `_speed_peak(parsed_log, window)` implementation as `artifacts/monotonicity_check_summary/run_monotonicity_check.py`.

## Step 1: Monotonic Window Filter

For each repeat, `[11,13]` is a subwindow of the original `[5,13]` tail, so `peak|v_xy|[11,13] <= peak|v_xy|[5,13]` and `rho_terminal >= rho_orig`. By this filter, archived `robust_safe` points were not exhaustively rejudged; only 10 were spot-checked.

Candidate set for exhaustive rejudgment is archived `robust_violation` plus `noise_band`:

| source | seed | arm | robust_violation | noise_band | candidate_total |
| --- | --- | --- | --- | --- | --- |
| ddmin | 0 | ddmin | 49 | 130 | 179 |
| ddmin | 1 | ddmin | 42 | 98 | 140 |
| ddmin | 2 | ddmin | 37 | 133 | 170 |
| direction_a | 0 | A | 47 | 7 | 54 |
| direction_a | 0 | B | 24 | 28 | 52 |
| direction_a | 0 | C | 32 | 20 | 52 |
| direction_a | 1 | A | 45 | 4 | 49 |
| direction_a | 1 | B | 23 | 26 | 49 |
| direction_a | 1 | C | 26 | 21 | 47 |
| direction_a | 2 | A | 40 | 5 | 45 |
| direction_a | 2 | B | 32 | 29 | 61 |
| direction_a | 2 | C | 23 | 24 | 47 |

## Step 2: Recoverability

- Exhaustive candidate points recoverable from parsed logs: 945/945.
- Extra verification objects recoverable from parsed logs (safe spotcheck + ddmin clean finals): 20/20.
- Parsed-log recoverable points audited: 965/965.
- Needs resimulation: 0.
- Estimated postprocessing time: 4775 parsed logs x 0.0061s/log = 28.98s measured wall time.
- Estimated resimulation time: 19s x 5 repeats x 0 points = 0.0s.

| audit_scope | source | seed | arm | points | parsed_recoverable | needs_resimulation |
| --- | --- | --- | --- | --- | --- | --- |
| candidate | ddmin | 0 | ddmin | 179 | 179 | 0 |
| candidate | ddmin | 1 | ddmin | 140 | 140 | 0 |
| candidate | ddmin | 2 | ddmin | 170 | 170 | 0 |
| candidate | direction_a | 0 | A | 54 | 54 | 0 |
| candidate | direction_a | 0 | B | 52 | 52 | 0 |
| candidate | direction_a | 0 | C | 52 | 52 | 0 |
| candidate | direction_a | 1 | A | 49 | 49 | 0 |
| candidate | direction_a | 1 | B | 49 | 49 | 0 |
| candidate | direction_a | 1 | C | 47 | 47 | 0 |
| candidate | direction_a | 2 | A | 45 | 45 | 0 |
| candidate | direction_a | 2 | B | 61 | 61 | 0 |
| candidate | direction_a | 2 | C | 47 | 47 | 0 |
| ddmin_clean_final | ddmin_clean_final | 0 | ddmin_clean_final | 4 | 4 | 0 |
| ddmin_clean_final | ddmin_clean_final | 1 | ddmin_clean_final | 2 | 2 | 0 |
| ddmin_clean_final | ddmin_clean_final | 2 | ddmin_clean_final | 4 | 4 | 0 |
| spotcheck_safe | direction_a | 0 | A | 1 | 1 | 0 |
| spotcheck_safe | direction_a | 0 | B | 1 | 1 | 0 |
| spotcheck_safe | direction_a | 0 | C | 2 | 2 | 0 |
| spotcheck_safe | direction_a | 1 | A | 1 | 1 | 0 |
| spotcheck_safe | direction_a | 2 | A | 3 | 3 | 0 |
| spotcheck_safe | direction_a | 2 | B | 1 | 1 | 0 |
| spotcheck_safe | direction_a | 2 | C | 1 | 1 | 0 |

## Step 3/6: Terminal Rejudgment Counts

| source | seed | arm | orig_robust_violation_count | orig_noise_band_count | terminal_robust_violation_count | terminal_noise_band_count | terminal_robust_safe_count |
| --- | --- | --- | --- | --- | --- | --- | --- |
| ddmin | 0 | ddmin | 49 | 130 | 0 | 0 | 179 |
| ddmin | 1 | ddmin | 42 | 98 | 0 | 0 | 140 |
| ddmin | 2 | ddmin | 37 | 133 | 0 | 0 | 170 |
| direction_a | 0 | A | 47 | 7 | 0 | 0 | 54 |
| direction_a | 0 | B | 24 | 28 | 0 | 0 | 52 |
| direction_a | 0 | C | 32 | 20 | 0 | 0 | 52 |
| direction_a | 1 | A | 45 | 4 | 0 | 0 | 49 |
| direction_a | 1 | B | 23 | 26 | 0 | 0 | 49 |
| direction_a | 1 | C | 26 | 21 | 0 | 0 | 47 |
| direction_a | 2 | A | 40 | 5 | 0 | 0 | 45 |
| direction_a | 2 | B | 32 | 29 | 0 | 0 | 61 |
| direction_a | 2 | C | 23 | 24 | 0 | 0 | 47 |

Headline question: any arm/ddmin/seed retaining terminal-window robust violations?

No terminal-window robust violations survived in `flagged_points.csv`.

## Step 4: ddmin Clean Finals

| seed | clean_finals | orig_robust_violation_count | terminal_robust_violation_count | terminal_noise_band_count | terminal_robust_safe_count |
| --- | --- | --- | --- | --- | --- |
| 0 | 4 | 4 | 0 | 0 | 4 |
| 1 | 2 | 2 | 0 | 0 | 2 |
| 2 | 4 | 4 | 0 | 0 | 4 |

## Step 5: Seed 1/2 Decay Profiles

| seed | arm | eval_id | theta_hash | orig_rho_mean | orig_label | peak_5_7_mean | peak_7_9_mean | peak_9_11_mean | peak_11_13_mean | terminal_rho_mean | terminal_label |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | C | 173 | f8b86ed4b82845b6 | -7.392961 | robust_violation | 8.392961 | 1.916872 | 0.362173 | 0.371135 | 0.628865 | robust_safe |
| 1 | C | 205 | af4f67fd4e6a71b1 | -7.314306 | robust_violation | 8.314306 | 1.958214 | 0.368606 | 0.377427 | 0.622573 | robust_safe |
| 1 | C | 197 | a14b2e3622d0b207 | -6.048008 | robust_violation | 7.048008 | 0.871278 | 0.332383 | 0.326619 | 0.673381 | robust_safe |
| 2 | C | 200 | fa985c1722ebf203 | -6.108523 | robust_violation | 7.108523 | 0.802275 | 0.356580 | 0.344357 | 0.655643 | robust_safe |
| 2 | C | 234 | 99ac162431974ee1 | -5.424029 | robust_violation | 6.424029 | 0.614652 | 0.329035 | 0.316147 | 0.683853 | robust_safe |
| 2 | A | 48 | 8735ddb6824db848 | -5.018125 | robust_violation | 6.018125 | 0.923124 | 0.312207 | 0.307297 | 0.692703 | robust_safe |

## Step 5: Safe Spotcheck

| seed | arm | eval_id | theta_hash | orig_rho_mean | orig_label | terminal_rho_mean | terminal_rho_std | terminal_label |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 2 | A | 14 | 3862568e88965b05 | 0.394537 | robust_safe | 0.907812 | 0.008361 | robust_safe |
| 2 | C | 193 | e7d3118370f33113 | 0.928310 | robust_safe | 0.962199 | 0.011711 | robust_safe |
| 1 | A | 5 | 50a64022bbfeca54 | 0.900629 | robust_safe | 0.945736 | 0.016662 | robust_safe |
| 2 | B | 135 | 829f406af83b9c16 | 0.145806 | robust_safe | 0.948129 | 0.013332 | robust_safe |
| 2 | A | 6 | 2bf9209ebbd91daa | 0.356151 | robust_safe | 0.961332 | 0.013406 | robust_safe |
| 0 | C | 223 | c2c1805e24e3c0cb | 0.468161 | robust_safe | 0.959190 | 0.009962 | robust_safe |
| 0 | B | 84 | ed9c9e3a32ce32f7 | 0.168557 | robust_safe | 0.957139 | 0.011397 | robust_safe |
| 2 | A | 66 | 90e548c888f6d1ea | 0.523486 | robust_safe | 0.928932 | 0.010105 | robust_safe |
| 0 | A | 51 | be275f55627378e3 | 0.416343 | robust_safe | 0.929770 | 0.019170 | robust_safe |
| 0 | C | 193 | a61b9599cff13aad | 0.372900 | robust_safe | 0.950961 | 0.012299 | robust_safe |

## Outputs

- `flagged_points.csv`: archived non-safe Direction A/ddmin points rejudged with terminal and profile metrics.
- `survivors.csv`: subset of `flagged_points.csv` with terminal `robust_violation`.
- `seed12_decay.csv`: seed 1/2 top-three archived Direction A robust violations by original rho, with four-window profiles.
- `spotcheck_safe.csv`: 10 archived Direction A robust-safe points rejudged under [11,13].
- `ddmin_clean_finals.csv`: ddmin clean minimized outputs rejudged under [11,13].
- `recoverability_audit.csv`: parsed-log availability audit.

Archive hygiene: this artifact does not copy or retain `*.ulg` files.

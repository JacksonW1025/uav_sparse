# OverDraw Island v1 Hardening Probe

VERDICT: **ISLAND-FLAKY**

The v2 island is dominated by bug_side labels caused by harness GCS_COMMAND LAND cleanup, not B_clamp or flight-controller preventive failsafe.

## Decision Criteria

| criterion | outcome | evidence |
| --- | --- | --- |
| smooth reproducible island probability | failed | Raw v2 probabilities form a small apparent island, but the B decomposition invalidates the v2 bug_side labels; no 25-seed sampling was run after the preregistered short-circuit. |
| positive evidence B was not met on overdraw runs | passed for audited core overdraw | 8 core overdraw runs have B_clamp=false, no preventive MODE/ERR/MSG, EKF FS max 0, and crash-check proxy 0.00 / 2.0 s. |
| consistent overdraw/bug_side mechanism | failed | 25 / 25 core bug_side labels are harness `GCS_COMMAND LAND`; none are B_clamp or FC failsafe. |

## B Decomposition

- Primary scope: Stage-0 v2 boundary grid, including the tight ANGLE_MAX direction check; this is the 8-overdraw-run island audit scope.
- Core v2 overdraw runs audited: 8; actual-B among them: 0.
- Core v2 bug_side runs audited: 25; bug_side caused only by harness `GCS_COMMAND LAND`: 25.
- Full sidecar audit: 51 runs; full v2 overdraw count 9.
- `MODE.Rsn=2` maps to `GCS_COMMAND` in ArduPilot `ModeReason.h`; this is test-harness cleanup, not a preventive failsafe.

| component | count |
| --- | ---: |
| B_harness_cleanup | 25 |
| B_none | 9 |

| v2 label | n | B_clamp | B_fc_failsafe | harness_GCS_LAND | B_none |
| --- | ---: | ---: | ---: | ---: | ---: |
| bug_side | 25 | 0 | 0 | 25 | 0 |
| overdraw | 8 | 0 | 0 | 0 | 8 |
| safe | 1 | 0 | 0 | 0 | 1 |

## OverDraw Positive Evidence

| run | clamp margin deg | EKF FS max | crash-check proxy max conjunctive s | actual FC B |
| --- | ---: | ---: | ---: | --- |
| stage0v2_p3_default_r0360_w00_t00_a4500_m100_s01 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_default_r0480_w00_t00_a4500_m100_s00 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_default_r0480_w00_t00_a4500_m100_s01 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_default_r0600_w00_t00_a4500_m100_s02 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_moderate_r0360_w08_t04_a4500_m100_s02 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_high_r0360_w15_t08_a4500_m100_s00 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_high_r0480_w15_t08_a4500_m100_s02 | 1.00 | 0.00 | 0.00 / 2.0 | False |
| stage0v2_p3_tightdirectioncheck_r0360_w00_t00_a3000_m100_s01 | 1.00 | 0.00 | 0.00 / 2.0 | False |

## Probability Shape

The original v2 labels form a small apparent island. After removing harness cleanup from B, the same grid reclassifies into a broad corrected-overdraw region; therefore the v2 island shape is not a coverage-hole shape.

| layer | r | n | p_overdraw_v2 | p_overdraw_corrected |
| --- | ---: | ---: | ---: | ---: |
| default | 240 | 3 | 0.00 | 1.00 |
| default | 360 | 3 | 0.33 | 1.00 |
| default | 480 | 3 | 0.67 | 1.00 |
| default | 600 | 3 | 0.33 | 1.00 |
| high | 240 | 3 | 0.00 | 1.00 |
| high | 360 | 2 | 0.50 | 1.00 |
| high | 480 | 2 | 0.50 | 1.00 |
| high | 600 | 2 | 0.00 | 1.00 |
| moderate | 240 | 3 | 0.00 | 1.00 |
| moderate | 360 | 2 | 0.50 | 1.00 |
| moderate | 480 | 3 | 0.00 | 1.00 |
| moderate | 600 | 2 | 0.00 | 1.00 |

## Artifacts

- Plot probability_vs_r: `planc/analysis/island_v1_probability_vs_r.png`
- Plot b_decomposition: `planc/analysis/island_v1_b_decomposition.png`
- Plot mechanism_pair: `planc/analysis/island_v1_mechanism_pair.png`
- Result JSON: `planc/results/island_v1_result.json`
- Preregistration: `planc/results/island_v1_prereg.json`

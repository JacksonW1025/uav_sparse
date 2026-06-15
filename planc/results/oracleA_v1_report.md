# Oracle-A v1 Hardening Probe

VERDICT: **REAL-GAP**

Hardened oracle A is met with B_preventive=0 in 9 reproducible cell(s).

## Criteria

- Hard A: SITL contact/low-floor descent, unrecovered or worsening >60 deg attitude error, or altitude loss > 15.0 m.
- Recovered transient: >60 deg/0.5s that returns before the oracle window closes is not unsafe.
- B: command clamp or flight-controller preventive failsafe before hard A; `MODE.Rsn=GCS_COMMAND` is excluded as harness.

## Summary

- Completed runs: 65; errors: 0.
- Hard-A runs: 49; clean unsafe: 49; bug-side: 0; recovered transient: 10; safe: 6.
- Hardened outcomes: `{"altitude_loss": 4, "crashed": 44, "diverged": 1, "recovered": 10, "safe": 6}`.
- Hard-A causes: `{"altitude_loss": 49, "crash_or_contact": 44, "unrecovered_divergence": 38, "worsening_divergence": 19}`.
- B_preventive before hard A: 0; B_any_window: 0; command clamp: 0.
- Command margin: max command 44.00 deg vs ANGLE_MAX 45.00 deg; minimum margin 1.00 deg.
- Escalation exhausted: False; boundary scan run: True.

## Decision Evidence

| criterion | result | evidence |
| --- | --- | --- |
| legal bounded command | True | max command angle stays below ANGLE_MAX by the configured guard. |
| hard consequence | True | hard-A count 49 across completed runs. |
| no preventive guard | True | B_preventive before hard A count 0; harness GCS_COMMAND excluded. |
| cross-seed reproducibility | True | robust r cells: `[120.0, 180.0, 240.0, 300.0, 360.0, 420.0, 480.0, 540.0, 600.0]`. |

## Cell Probabilities

| layer | model | r | n | hard_A | clean_unsafe | bug_side | recovered | safe | p_clean |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| default | m100 | 120 | 5 | 2 | 2 | 0 | 2 | 1 | 0.40 |
| default | m100 | 180 | 10 | 10 | 10 | 0 | 0 | 0 | 1.00 |
| default | m100 | 240 | 5 | 4 | 4 | 0 | 1 | 0 | 0.80 |
| default | m100 | 300 | 5 | 3 | 3 | 0 | 0 | 2 | 0.60 |
| default | m100 | 360 | 10 | 4 | 4 | 0 | 3 | 3 | 0.40 |
| default | m100 | 420 | 5 | 3 | 3 | 0 | 2 | 0 | 0.60 |
| default | m100 | 480 | 10 | 9 | 9 | 0 | 1 | 0 | 0.90 |
| default | m100 | 540 | 5 | 5 | 5 | 0 | 0 | 0 | 1.00 |
| default | m100 | 600 | 10 | 9 | 9 | 0 | 1 | 0 | 0.90 |

## Representative Trajectory

- Run: `oracleAv1_grid_default_r0600_w00_t00_a4500_m100_s03`
- Label: `clean_unsafe`
- Example clean unsafe: `oracleAv1_grid_default_r0180_w00_t00_a4500_m100_s01`; outcome `crashed`; altitude loss 43.29 m.

## Artifacts

- Preregistration: `planc/results/oracleA_v1_prereg.json`
- Result JSON: `planc/results/oracleA_v1_result.json`
- Plot outcomes_vs_r: `planc/analysis/oracleA_v1_outcomes_vs_r.png`
- Plot true_boundary: `planc/analysis/oracleA_v1_true_boundary.png`
- Plot trajectory: `planc/analysis/oracleA_v1_trajectory.png`

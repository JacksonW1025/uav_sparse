# H3 Transition-Discontinuity Probe

- outcome: `stage_a_failed_no_robust_transition_violation`
- seed: `0`
- repeats per rho: `5`
- total query calls: `325`
- elapsed wall time: `6010.5s`

## Stage 0
- gate_pass: `True`
- neutral_safe_2std: `True`
- switch_clean: `True`
- maneuver_induced_motion: `True`
- switched_run_differs_from_posctl: `True`
- transition validation: `runs/h3_transition_seed0_v1/reports/stage0_transition_validation.csv`

## Stage A
- robust transition-caused violations: `0`
- weak 1std candidates: `0`
- noise straddles not counted: `21`
- joint-random probe hits: `0`
- distinct clusters: `0`
- pair rows: `runs/h3_transition_seed0_v1/reports/stageA_pair_classification.csv`
- cluster rows: `runs/h3_transition_seed0_v1/reports/stageA_robust_clusters.csv`

# E/P Boundary Movement v1

## Mechanism Separation

- Source oracleA runs: 65.
- Mechanism counts among clean hard-A runs: `{"altitude_bleed": 21, "both": 17, "tumble": 11}`.
- Best 2D pair: `actual_rate_peak_deg_s` x `high_bank_dwell_impulse_gt_80deg_deg_s`.
- 2D AUC: 0.912; restored: True.

## E/P Grid

- Completed grid runs: 540; errors: 0.
- Command clean in all E/P cells: True.
- Flight-controller B clean in all E/P cells: True.
- E unsafe-area monotone: False; E boundary moves forward: True.
- P unsafe-area monotone: False; P boundary moves forward: True.

| E | P | n | p_clean | r50 | contract | B_preventive | clamp | min cmd margin |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| E0 | P30 | 60 | 0.767 | 120 | 0 | 0 | 0 | 1.00 |
| E0 | P45 | 60 | 0.733 | 120 | 0 | 0 | 0 | 1.00 |
| E0 | P80 | 60 | 0.867 | 120 | 0 | 0 | 0 | 1.60 |
| Ehigh | P30 | 60 | 0.767 | 120 | 0 | 0 | 0 | 1.00 |
| Ehigh | P45 | 60 | 0.817 | 120 | 0 | 0 | 0 | 1.00 |
| Ehigh | P80 | 60 | 0.950 | 120 | 0 | 0 | 0 | 1.60 |
| Elow | P30 | 60 | 0.850 | 120 | 0 | 0 | 0 | 1.00 |
| Elow | P45 | 60 | 0.767 | 120 | 0 | 0 | 0 | 1.00 |
| Elow | P80 | 60 | 0.817 | 120 | 0 | 0 | 0 | 1.60 |

## Reachability

- Completed reachability runs: 2; errors: 0.
- Non-doublet clean unsafe found: True.
- Witness: `epmovev1_reach_sidewindsustainedbankp80_a8000_w15_t08_s00`.

## Artifacts

- Mechanisms JSON: `planc/results/mechanisms_result.json`
- E/P result JSON: `planc/results/ep_movement_result.json`
- Plot boundary_surface: `planc/analysis/ep_movement_boundary_surface.png`
- Plot r_boundaries_by_cell: `planc/analysis/ep_movement_r_boundaries_by_cell.png`
- Plot contract_trigger_table: `planc/analysis/ep_movement_contract_trigger_table.png`
- Plot reachability_trajectory: `planc/analysis/ep_movement_reachability_trajectory.png`

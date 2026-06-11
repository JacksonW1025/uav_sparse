# MANTIS H* Witness Report

Status: `NO_WITNESS_FOUND_AT_HEADROOM_BOUNDARY`
Run dir: `runs/mantis_Hboundary_px4_seed0_v0`
Queries: `100`
Failed condition: `no strong violation`

## A. H-Boundary Result

- Scales tried: `[0.65, 0.6, 0.55, 0.5]`
- Lowest admissible H*: `0.5`
- First bad-headroom scale: ``
- Default-safe boundary reached: `False`

## B. H* Default Gates

- M0@P0: C_recover `safe`, C_track `safe`
- Msmall@P0: C_recover `safe`, C_track `safe`
- Strongest Mstrong@P0 max C_recover ratio: `0.11569266872746604`
- Strongest Mstrong@P0 max C_track nte: `0.7189471699423575`
- Nonlinear observable: `True`; activation rate `1.0`; max overlap `0.20400023460388184`

## C. P Boundary Under H*

- Candidates generated: `14`
- Pure-param rejected: `6`
- Small-safe retained: `8`
- Top retained candidates: `['rate_d_high_x1p188', 'rate_i_x3', 'att_p_boundary_rate_p_x1p5', 'rate_p_boundary_high_i_x1p5', 'rate_p_boundary_low_d_x1p5', 'rate_d_low_x0p35']`

## D. Witness Results

- C_recover violations: `0`
- C_track violations: `0`
- Confirmation rows: `0`
- Accepted witness count: `0`

## E. Top-Level Status

`NO_WITNESS_FOUND_AT_HEADROOM_BOUNDARY`

## F. Failed Condition

`no strong violation`

## G. Recommendation

Pivot to the ArduPilot STABILIZE witness fast path or stop the PX4 MANTIS positive-bug search.

## Plots

- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_high_x1p188_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_high_x1p188_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_high_x1p188_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_high_x1p188_manual_input.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_i_x3_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_i_x3_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_i_x3_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_i_x3_manual_input.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_low_x0p35_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_low_x0p35_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_low_x0p35_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_d_low_x0p35_manual_input.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_att_p_boundary_rate_p_x1p5_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_att_p_boundary_rate_p_x1p5_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_att_p_boundary_rate_p_x1p5_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_att_p_boundary_rate_p_x1p5_manual_input.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_high_i_x1p5_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_high_i_x1p5_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_high_i_x1p5_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_high_i_x1p5_manual_input.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_low_d_x1p5_four_arm_rates.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_low_d_x1p5_tracking_error.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_low_d_x1p5_actuator_saturation.png`
- `runs/mantis_Hboundary_px4_seed0_v0/plots/Hstar_rate_p_boundary_low_d_x1p5_manual_input.png`

## Evidence Tables

- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_headroom_boundary.csv`
- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_Hstar_boundary_candidates.csv`
- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_Hstar_witness_candidates.csv`
- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_Hstar_witness_confirmation.csv`
- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_witness_tracking.csv`
- `runs/mantis_Hboundary_px4_seed0_v0/reports/mantis_nonlinear_diagnostics.csv`

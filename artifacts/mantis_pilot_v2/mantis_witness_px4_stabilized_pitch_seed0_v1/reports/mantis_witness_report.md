# MANTIS Witness Report

Status: `NO_WITNESS_FOUND`
Run dir: `runs/mantis_witness_px4_stabilized_pitch_seed0_v1`
Queries: `100`
Note: `no C_recover/C_track violation`

## A. Witness Plan

- Contracts: `C_recover` post-neutral residual-rate and pre-registered `C_track` active-window rate-tracking.
- C_track thresholds: nte>=0.65, peak_err>=1.0 rad/s, high_err_duration>=0.25s, baseline ratio>=2.5, nonlinear overlap>=0.2s.
- C_track was pre-registered because recovery-only checks can miss poor active-window tracking that recovers quickly.
- Low headroom is admissible only when default-parameter gates remain safe.

## B. Headroom Audit

- Profiles tried: `4`
- Selected H: `H3` PX4 CA rotor thrust coefficient scaled to 0.70

## C. Boundary Search

- Boundary rows: `14`
- Small-safe retained: `4`
- Pure-param rejected: `10`

## D. Witness Results

- Candidate count: `32`
- Strong-unsafe by C_recover: `0`
- Strong-unsafe by C_track: `0`
- Confirmation rows: `0`
- Accepted witness count: `0`

## E. Visual Evidence

- no candidate plots generated

## F. Top-Level Status

`NO_WITNESS_FOUND`

## G. Failed Condition If Not Accepted

`no C_recover/C_track violation`

## Evidence Tables

- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_plan.json`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_headroom_audit.csv`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_boundary.csv`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_candidates.csv`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_confirmation.csv`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_witness_tracking.csv`
- `runs/mantis_witness_px4_stabilized_pitch_seed0_v1/reports/mantis_nonlinear_diagnostics.csv`

# PX4 control-authority cross-stack v1

**VERDICT: REPLICATES**

PX4 target: `v1.15.0` / `30e763b6780061d70a14894e3e8b06e6a656f9b8`. SITL simulator used: `jmavsim`.

## Phase A

Static audit verdict: `MPC_TILTMAX_AIR x offboard attitude SET_ATTITUDE_TARGET` is **UNCOVERED**. See `planc/results/px4_coverage_matrix.md` and prereg `planc/results/px4_ctrlauth_v1_prereg.json`.

## Phase B Runs

| roll cmd (deg) | label | max sp tilt | max achieved tilt | alt loss m | hard consequence | fd before consequence | reason |
|---:|---|---:|---:|---:|---|---|---|
| 0 | clean_safe | 0.81 | 0.74 | 0.12 | False | False | no_hard_consequence_in_oracle_window |
| 50 | clean_unsafe | 50.00 | 50.71 | 55.54 | True | False | unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation |
| 52 | clean_unsafe | 52.00 | 52.80 | 56.92 | True | False | unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation |
| 55 | clean_unsafe | 55.00 | 55.14 | 58.08 | True | False | unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation |
| 55 | clean_unsafe | 55.00 | 55.12 | 57.98 | True | False | unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation |
| 60 | contract_violated | 60.00 | 60.10 | 60.25 | True | True | failure_detector_status_true_before_hard_consequence |
| 60 | clean_unsafe | 60.00 | 60.14 | 60.39 | True | False | unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation |
| 65 | contract_violated | 65.00 | 65.17 | 61.46 | True | True | failure_detector_status_true_before_hard_consequence |
| 70 | contract_violated | 70.00 | 70.15 | 62.05 | True | True | failure_detector_status_true_before_hard_consequence |
| 90 | contract_violated | 90.00 | 90.34 | 67.15 | True | True | failure_detector_status_true_before_hard_consequence |
| 120 | contract_violated | 120.00 | 120.36 | 70.58 | True | True | failure_detector_status_true_before_hard_consequence |
| 150 | contract_violated | 150.00 | 150.23 | 74.35 | True | True | failure_detector_status_true_before_hard_consequence |

## Artifacts

- tilt_vs_command: `/mnt/nvme/px4_work/uav_sparse/planc/analysis/px4_ctrlauth_v1_tilt_vs_command.png`
- outcome_vs_roll: `/mnt/nvme/px4_work/uav_sparse/planc/analysis/px4_ctrlauth_v1_outcome_vs_roll.png`
- px4_ctrlauth_v1_r000_s00: ulog `planc/logs/px4_ctrlauth_v1_r000_s00.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r000_s00_parsed.oracle.json`
- px4_ctrlauth_v1_r050_s00: ulog `planc/logs/px4_ctrlauth_v1_r050_s00.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r050_s00_parsed.oracle.json`
- px4_ctrlauth_v1_r052_s01: ulog `planc/logs/px4_ctrlauth_v1_r052_s01.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r052_s01_parsed.oracle.json`
- px4_ctrlauth_v1_r055_s00: ulog `planc/logs/px4_ctrlauth_v1_r055_s00.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r055_s00_parsed.oracle.json`
- px4_ctrlauth_v1_r055_s02: ulog `planc/logs/px4_ctrlauth_v1_r055_s02.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r055_s02_parsed.oracle.json`
- px4_ctrlauth_v1_r060_s01: ulog `planc/logs/px4_ctrlauth_v1_r060_s01.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r060_s01_parsed.oracle.json`
- px4_ctrlauth_v1_r060_s05: ulog `planc/logs/px4_ctrlauth_v1_r060_s05.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r060_s05_parsed.oracle.json`
- px4_ctrlauth_v1_r065_s02: ulog `planc/logs/px4_ctrlauth_v1_r065_s02.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r065_s02_parsed.oracle.json`
- px4_ctrlauth_v1_r070_s03: ulog `planc/logs/px4_ctrlauth_v1_r070_s03.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r070_s03_parsed.oracle.json`
- px4_ctrlauth_v1_r090_s02: ulog `planc/logs/px4_ctrlauth_v1_r090_s02.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r090_s02_parsed.oracle.json`
- px4_ctrlauth_v1_r120_s03: ulog `planc/logs/px4_ctrlauth_v1_r120_s03.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r120_s03_parsed.oracle.json`
- px4_ctrlauth_v1_r150_s04: ulog `planc/logs/px4_ctrlauth_v1_r150_s04.ulg`, oracle `planc/logs/px4_ctrlauth_v1_r150_s04_parsed.oracle.json`

## Interpretation

At least one legal offboard attitude command exceeded `MPC_TILTMAX_AIR`, was accepted without setpoint clipping, produced a hard consequence, and stayed Tier-1 clean before the hard consequence in the ulog oracle window.

Clean witnesses in this campaign: `50, 52, 55, 55, 60` deg.

Higher roll commands also confirmed the setpoint was not clipped, but are not counted as clean witnesses when `failure_detector_status` asserted before the preregistered hard consequence.

The 0 deg baseline row ignores the commanded cleanup LAND transition after the stable OFFBOARD segment; it had no hard consequence and no failure-detector assertion.

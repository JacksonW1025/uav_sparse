VERDICT: PASS

# stabilize_acro v1 report

- Scenario: deliberate, legal operator input via supported ACRO mode; not spontaneous loss of control.
- Firmware: `Copter-4.4.1` / `e010f97906087a3a1975e1c4fcc1f88a249599ce` from `/home/car/ardupilot`.
- Reachability: RC_CHANNELS_OVERRIDE + mode changes only; SET_ATTITUDE_TARGET count = `0`; GUIDED mode set count = `0`.
- Fixed ACRO config: `ANGLE_MAX=45 deg`, `ACRO_TRAINER=0`; ANGLE_MAX is not raised or disabled.

## Decision checks
- P0.1_acro_rc_body_rate: OK
- P0.2_angle_max_not_engaged_in_acro: OK
- P0.3_input_applied: OK
- P0.4_zero_offboard_injection: OK
- P0.5_admission_guard: OK
- Robust clean_unsafe zone: OK; runs=33, targets=[55.0, 60.0, 70.0, 90.0, 120.0, 150.0], tilt_width=98.15 deg.
- Prediction gate (a): OK; accuracy=1.000.
- Severity repeatability MAE: OK; MAE=1.36 m, bound=2.50 m.
- Stabilize-vs-ACRO gate (b): OK.
- ANGLE_MAX invariance gate (c): OK.
- Final label counts: `{'clean_safe': 18, 'clean_unsafe': 39, 'ambiguous': 5}`.

## Noise and GIGO
- 55 deg repeated altitude-loss sigma: `1.6655 m`; d_margin=`4.9965 m`; mae_bound=`2.4982 m`.
- Zero-tilt hover loss mean: `0.00 m`; 55 deg hover loss mean: `22.18 m`.

## Figures
- tilt_vs_loss: `/mnt/nvme/px4_work/uav_sparse/planc/results/stabilize_acro_v1_tilt_vs_loss.png`
- stabilize_vs_acro: `/mnt/nvme/px4_work/uav_sparse/planc/results/stabilize_acro_v1_stabilize_vs_acro.png`
- anglemax_stratification: `/mnt/nvme/px4_work/uav_sparse/planc/results/stabilize_acro_v1_anglemax_stratification.png`
- gigo_hover_throttle: `/mnt/nvme/px4_work/uav_sparse/planc/results/stabilize_acro_v1_gigo_hover_throttle.png`

## Extreme Points

- Ground-contact oracle-A occurred at target layers `[120.0, 150.0]`. No preventive failsafe or CRASH_CHECK contract trigger was logged before these consequences; the clean witness band does not depend on those extreme ground-contact points.

## Limitations

- This is an operator-reachable legal-input demonstration, not an environment-only or spontaneous instability claim.
- The consequence is near deterministic in SITL; robustness is from the wide achieved-tilt range, not from claiming to predict through large noise.
- No threshold scaling law, O(log) search law, or area law is claimed.

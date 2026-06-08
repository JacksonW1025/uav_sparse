# CADET Existence Recon v0 Preregistration Contracts

Status: pre-run registration artifact. No new recon simulations were run before
this file was created.

## Frozen Shared Rules

- Feasible input set: `|u_t| <= 1.0` and `|u_t - u_{t-1}| <= R` as implemented by the existing CADET input projection.
- Evaluation window for recon labels: absolute terminal window `[11.0,13.0] s`.
- Tail profile windows to record: `[5,7]`, `[7,9]`, `[9,11]`, `[11,13]` seconds.
- Repeats: `J=5`.
- Label gate: `robust_violation` iff `rho_mean + 2*rho_std < 0`; `robust_safe` iff `rho_mean - 2*rho_std > 0`; otherwise `noise_band`.
- Survivor gate: terminal-window `robust_violation` and `max_abs < 0.9`.

## Contracts

| Stage | Mode | Property | Documentation source | Source fragment (<=15 words) | Bound parameter | Frozen threshold | A_phi |
|---|---|---|---|---|---|---|---|
| 0, 2 | PX4 Position (MC) / `px4_position` | `post_neutral_xy_velocity` | PX4 Guide main, Position Mode (Multicopter), lines 1912 and 1941: https://docs.px4.io/main/en/flight_modes_mc/position | "actively brake, level, and be locked" | `MPC_VEL_MANUAL` for manual horizontal speed scale; recon threshold keeps existing v4 value | `1.0 m/s` | `{roll,pitch}` |
| 1 | PX4 Altitude (MC) / planned `px4_altitude` | `post_neutral_climb_rate` | PX4 Guide main, Altitude Mode (Multicopter), lines 1914, 1924, 1927: https://docs.px4.io/main/en/flight_modes_mc/altitude | "Throttle (~50%) holds current altitude steady" | `MPC_Z_VEL_MAX_UP` | `0.3 m/s = 0.10 * 3.0 m/s` | `{throttle}` |

Anti-trap note: no `post_neutral_xy_velocity` contract is registered for
ALTCTL. PX4 Altitude documentation explicitly says horizontal position can move
due to wind or pre-existing momentum.

## Stage 2 Parameter Nomination

Planned POSCTL conjunction parameter: `MPC_ACC_HOR`.

Rationale frozen before any recon simulation:

- The registered property is horizontal residual velocity after centering in
  Position mode, and `derive_A_phi("post_neutral_xy_velocity")` maps this to
  `{roll,pitch}`.
- PX4 main parameter reference defines `MPC_ACC_HOR` as "Acceleration for
  autonomous and for manual modes" and says manual use applies in
  `MPC_POS_MODE` acceleration based mode.
- PX4 main Position Mode docs say the default manual translation strategy is
  acceleration based and list `MPC_ACC_HOR_MAX` as a horizontal acceleration
  parameter, but the current parameter reference says for `MPC_ACC_HOR_MAX`:
  "`MPC_POS_MODE` 4 not used, use `MPC_ACC_HOR` instead."
- Therefore the single Stage 2 sweep parameter is `MPC_ACC_HOR`, not a broad
  MPC parameter sweep.

Documented legal range for `MPC_ACC_HOR`: min `2.0`, max `15.0`, increment
`1.0`, default `3.0`, unit `m/s^2`, from PX4 Guide main Parameter Reference,
lines 29961-29968:
https://docs.px4.io/main/en/advanced_config/parameter_reference

Candidate ALTCTL secondary parameter, if Stage 2 secondary probe is reached:
`MPC_Z_VEL_P_ACC`; documented legal range min `2.0`, max `15.0`, increment
`0.1`, default `4.0`, from PX4 Guide main Parameter Reference, lines
30418-30425.

## Code-Reality Precheck

Before running Stage 0, the local oracle entry point was checked:

- `src/cadet/properties.py::compute_robustness(parsed_log, property_name, config)`
  has no explicit window argument.
- For `post_neutral_xy_velocity`, it uses all samples after `t_neutral_s` and
  returns `v_max - speed.max()`, i.e. the existing full-tail peak behavior.
- Separate archived scripts compute `[11,13]` metrics manually, but the
  preregistered instruction explicitly required `compute_robustness` to support
  an explicit window or to pass `[11,13]` explicitly.

Per the recon spec hard stop rule, Stage 0 simulation was not started.

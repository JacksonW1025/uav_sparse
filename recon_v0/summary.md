# CADET Existence Recon v0 Summary

## STOP 0 Status

Status: blocked before Stage 0 simulation.

The Stage 0 precheck found that the local oracle entry point does not support
the explicitly registered terminal window:

- `src/cadet/properties.py::compute_robustness(parsed_log, property_name, config)`
  accepts only `parsed_log`, `property_name`, and `config`.
- For `post_neutral_xy_velocity`, it computes the peak over the full tail after
  `t_neutral_s`; there is no parameter for `[11,13] s`.
- The recon instructions explicitly list "`compute_robustness` does not support
  a specified window" as a STOP condition.

No Stage 0 control simulations, saturation controls, or positive-control
parameter perturbations were run.

## STOP Questions

1. Did the Stage 0 negative control reproduce 0 survivor?

   Not evaluated. The run stopped before new simulations because the
   registered oracle API cannot accept the terminal window.

2. Did the positive control detect a violation?

   Not evaluated for the same reason.

3. Is the oracle trustworthy for this recon?

   Not established. Existing archived scripts can compute `[11,13]` peaks
   manually, but the registered `compute_robustness` API itself cannot.

## Additional Code-Reality Finding

If Stage 0 is unblocked later, Stage 1 currently has a separate likely stop:
`configs/rq1_minimal.yaml` contains `px4_position`, `px4_hold`,
`px4_transition`, `ap_loiter`, and `ap_althold`, but no PX4 ALTCTL scenario.

## Decision Table Row

No row in the preregistered decision table is reached. This is a pre-Stage-0
oracle-window API stop, not evidence for `H_mode`, `H_conjunction`, or
`H_null`.

## STOP C Precheck Status

Status: blocked before Stage 0 simulation.

Phase B was completed in commit `b8ce520`. Phase C then checked whether the
harness already supports overriding a single PX4 runtime parameter for one
simulation, as required before running the `MPC_ACC_HOR=0.5` positive control.

Actual code reality:

- `src/cadet/vehicle/mavlink_common.py::MavlinkVehicleMixin._set_param()` can
  send MAVLink `PARAM_SET`.
- `src/cadet/vehicle/px4.py::PX4Adapter.prepare()` uses `_set_param()` only for
  hard-coded setup parameters: `COM_RC_IN_MODE=1` and `MIS_TAKEOFF_ALT` from
  `scenario.takeoff_alt_m`.
- `src/cadet/config.py::ScenarioCfg` has no parameter override field.
- `configs/rq1_minimal.yaml` has PX4 simulator fields for speed, MAVLink URL,
  manual-control rate, and cleanup, but no per-run parameter override field.
- `src/cadet/query.py::run_query()` has no hook between `adapter.prepare()` and
  `adapter.run()` to inject `MPC_ACC_HOR` for a single query.
- `scripts/start_px4.sh` only forwards `HEADLESS` and
  `PX4_SIM_SPEED_FACTOR`; it does not expose a parameter file or `param set`
  injection path.

Therefore the existing harness can set fixed setup parameters internally, but
does not expose the required per-simulation override mechanism for
`MPC_ACC_HOR`. Per the Phase C STOP rule, no Stage 0 negative controls, no
saturated controls, and no illegal positive control were run. No theta-star was
selected, and oracle sensitivity under the positive control is not evaluated.

No row in the preregistered decision table is reached. This is a pre-Stage-0
parameter-override API stop, not evidence for `H_mode`, `H_conjunction`, or
`H_null`.

## STOP C Stage 0 Results

Status: Stage 0 executed and stopped before Phase D.

Phase A2 parameter override support commit: `9351b80`.
Run artifacts directory: `runs/recon_stage0_terminal_v0`.
Evaluation used `compute_robustness(..., window=(11.0, 13.0))` and recorded `[5,7]`, `[7,9]`, `[9,11]`, `[11,13]` XY-speed peaks.

### Stage 0 Rows

| point_id | MPC_ACC_HOR | legal | label | terminal_peak | margin | rho_mean | rho_std | max_abs | survivor |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| eval164 | 3.000 | True | robust_safe | 0.027506 | 0.972494 | 0.972494 | 0.007972 | 0.500000 | False |
| eval174 | 3.000 | True | robust_safe | 0.040215 | 0.959785 | 0.959785 | 0.010416 | 0.500000 | False |
| eval182 | 3.000 | True | robust_safe | 0.045782 | 0.954218 | 0.954218 | 0.006460 | 0.500000 | False |
| eval199 | 3.000 | True | robust_safe | 0.046625 | 0.953375 | 0.953375 | 0.008706 | 0.500000 | False |
| eval234 | 3.000 | True | robust_safe | 0.050138 | 0.949862 | 0.949862 | 0.015341 | 0.500000 | False |
| G01 | 3.000 | True | robust_safe | 0.321245 | 0.678755 | 0.678755 | 0.020213 | 1.000000 | False |
| G02 | 3.000 | True | robust_safe | 0.300011 | 0.699989 | 0.699989 | 0.016905 | 1.000000 | False |
| eval234_MPC_ACC_HOR_0p5 | 0.500 | False | robust_safe | 0.017593 | 0.982407 | 0.982407 | 0.014884 | 0.500000 | False |

### STOP C Questions

1. Negative control 0 survivor: `True` (0 survivor rows among default-parameter C1 rows).
2. Theta-star: `eval234` selected by minimum default terminal margin among the five interior theta files; default margin `0.949862` with terminal_peak `0.050138`.
3. Positive control detected robust violation: `False`.
4. Positive-control readback targets: `[0.5, 0.5, 0.5, 0.5, 0.5]`; readbacks: `[0.5, 0.5, 0.5, 0.5, 0.5]`; reboot_required flags: `[False, False, False, False, False]`.
5. Oracle credible for Stage 2 gate: `False`.
   C2 did not fire. Because readback verification ran on every repeat, distinguish this as oracle/vehicle-response non-sensitivity if readbacks equal 0.5; otherwise parameter injection failure.

### Decision Table Row

Stage 0 is a control/oracle gate only. Phase D is not entered until this STOP C result is reviewed.

## Amendment 02 Step 1 STOP

Status: stopped before constructing `theta**` or running any Amendment 02 recon
data.

Amendment 02 was committed in `0727a1e`. Step 1 then checked current SITL
parameters and the active PX4 source path.

Actual findings:

- Current SITL readback: `MPC_POS_MODE=4`.
- Current SITL also read back `MPC_ACC_HOR=0.5`, meaning the prior
  positive-control override persisted in PX4 parameter storage. Future default
  runs need an explicit reset/cleanup mechanism before they can be interpreted
  as default-parameter runs.
- In `MPC_POS_MODE=4`, `FlightModeManager.cpp` selects
  `ManualAcceleration`.
- `ManualAcceleration` calls `StickAccelerationXY::generateSetpoints()`.
- `StickAccelerationXY` computes commanded acceleration and zero-stick drag
  using `MPC_ACC_HOR`, and applies `MPC_JERK_MAX` to the acceleration slew.
- `MPC_DEC_HOR_SLOW` is absent from the current PX4 parameter metadata.
- `MPC_ACC_HOR_MAX` is present but the current PX4 main Parameter Reference and
  local source metadata say mode 4 does not use it and should use
  `MPC_ACC_HOR` instead.

Therefore Step 1 did not identify a valid corrected deceleration-side scan
parameter among the Amendment 02 candidates. The code reality conflicts with
the requested assumption that the current-mode brake authority is
`MPC_DEC_HOR_SLOW` or `MPC_ACC_HOR_MAX`.

No `theta**_sat`, no `theta**_sub`, no repaired positive control, no corrected
Stage 2, and no Stage 1 inventory were run.

## Amendment 03 Stage 2 + Stage 1 Inventory

Status: Amendment 03 executed through the requested MPC_ACC_HOR scan and then
unconditionally continued to Stage 1 inventory.

Amendment 03 pre-data correction commit: `b607662`.
Run artifacts directory: `runs/recon_amendment03_v0`.
Generated recon files:

- `recon_v0/theta_build_profiles.csv`
- `recon_v0/stage2_conjunction.csv`
- `recon_v0/survivors.csv`
- `recon_v0/stage1_inventory.csv`

### Pollution-Fix Check

Every PX4 run explicitly set and read back both `MPC_ACC_HOR` and
`MPC_JERK_MAX`. The default reset check used `eval234` with
`{MPC_ACC_HOR: 3.0, MPC_JERK_MAX: 8.0}`:

| check | Stage 0 eval234 terminal_peak | new terminal_peak | delta | readback |
| --- | ---: | ---: | ---: | --- |
| explicit default reset | 0.050138 | 0.047642 | 0.002496 | `MPC_ACC_HOR=3.0`, `MPC_JERK_MAX=8.0` |

This passes the contamination screen and confirms the prior persisted
`MPC_ACC_HOR=0.5` state was not used for new default runs.

### Maneuvers

Both maneuvers are pitch-only, remain in F, and project with zero change:

- `theta_build_sat`: pitch windows
  `[0.25, 0.50, 0.75, 1.00, 1.00, 1.00, 1.00, 0.75, 0.50, 0.25]`.
- `theta_build_sub`: pitch windows
  `[0.25, 0.50, 0.75, 0.85, 0.85, 0.85, 0.85, 0.60, 0.35, 0.10]`.

### MPC_ACC_HOR Scan

Fixed `MPC_JERK_MAX=8.0`. Terminal window stayed `[11,13]` and threshold stayed
`1.0 m/s`.

| MPC_ACC_HOR | legal | maneuver | seed | max_abs | t_neutral speed | terminal_peak | rho_mean | rho_std | label | survivor |
| ---: | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 0.5 | False | sat | 0 | 1.00 | 1.505045 | 0.470306 | 0.529694 | 0.048653 | robust_safe | False |
| 2.0 | True | sub | 0 | 0.85 | 2.531908 | 0.113996 | 0.886004 | 0.021262 | robust_safe | False |
| 2.0 | True | sat | 0 | 1.00 | 3.731571 | 0.155086 | 0.844914 | 0.013599 | robust_safe | False |
| 3.0 | True | sub | 0 | 0.85 | 2.911656 | 0.157595 | 0.842405 | 0.011662 | robust_safe | False |
| 3.0 | True | sat | 0 | 1.00 | 4.653485 | 0.201315 | 0.798685 | 0.027530 | robust_safe | False |
| 5.0 | True | sub | 0 | 0.85 | 3.166246 | 0.201648 | 0.798352 | 0.025658 | robust_safe | False |
| 5.0 | True | sat | 0 | 1.00 | 5.786466 | 0.294639 | 0.705361 | 0.018338 | robust_safe | False |
| 10.0 | True | sub | 0 | 0.85 | 3.931628 | 0.252428 | 0.747572 | 0.183808 | robust_safe | False |
| 10.0 | True | sat | 0 | 1.00 | 7.054561 | 1.212636 | -0.212636 | 0.448400 | noise_band | False |
| 15.0 | True | sub | 0 | 0.85 | 3.415161 | 1.338109 | -0.338109 | 0.093963 | robust_violation | True |
| 15.0 | True | sat | 0 | 1.00 | 9.454447 | 4.697109 | -3.697109 | 0.342907 | robust_violation | False |

The illegal `0.5` positive control did not fire. Its parameter readback was
correct on every repeat, so this is not a parameter-injection failure. The
realized t_neutral speed was only `1.505045 m/s`, so the result is specifically
that this full-stick, low-acceleration/low-drag run still decayed below the
terminal threshold.

The legal seed-0 marginal curve turns negative only at `MPC_ACC_HOR=15.0` with
the non-saturated `theta_build_sub`. That seed-0 row was immediately
reproduced:

| point | seed | terminal_peak | rho_mean | rho_std | label | survivor |
| --- | ---: | ---: | ---: | ---: | --- | --- |
| `MPC_ACC_HOR_15p0_sub` | 0 | 1.338109 | -0.338109 | 0.093963 | robust_violation | True |
| `MPC_ACC_HOR_15p0_sub_seed1_repro` | 1 | 1.099989 | -0.099989 | 0.225045 | noise_band | False |
| `MPC_ACC_HOR_15p0_sub_seed2_repro` | 2 | 0.987087 | 0.012913 | 0.360178 | noise_band | False |

Decision: a legal, non-saturated seed-0 survivor exists at the upper documented
legal value `MPC_ACC_HOR=15.0`, but the strict seed 1/2 reproduction gate did
not pass. Therefore `H_conjunction` is not accepted as a reproducible result
from this scan.

### Stage 1 Inventory

Smoke method: one zero-input run per mode candidate, current harness, seed 0.
PX4 runs explicitly reset `MPC_ACC_HOR=3.0` and `MPC_JERK_MAX=8.0`.

| scenario | action in harness | run status | observed modes | doc contract for centered/neutral behavior |
| --- | --- | --- | --- | --- |
| `px4_hold` | start in Position, switch to `Hold`/`LOITER` after t_neutral | pass | `POSCTL,LOITER` | PX4 Hold causes MC to stop/hover at current position and altitude; candidate property is `post_neutral_xy_velocity`. Source: https://docs.px4.io/main/en/flight_modes_mc/hold |
| `px4_transition` | start in Position, repeatedly request `Hold`/`LOITER` from `t_switch_s=5.0`; observed transition at `5.5s` | pass | `POSCTL,LOITER` | Same PX4 Hold contract after transition; candidate property is `post_neutral_xy_velocity`. Source: https://docs.px4.io/main/en/flight_modes_mc/hold |
| `ap_althold` | run in ArduPilot `ALT_HOLD` throughout | pass | `ALT_HOLD` | ArduPilot AltHold maintains altitude at mid throttle; roll/pitch directly control lean angles, so no official horizontal velocity-to-zero contract is registered. Candidate vertical contract would be altitude/climb-rate, not XY velocity. Source: https://ardupilot.org/copter/docs/altholdmode.html |

PX4 docs also distinguish Altitude mode from Position/Hold by saying released
sticks level and maintain altitude, but do not actively brake or hold horizontal
position: https://docs.px4.io/main/en/flight_modes_mc/

STOP: Stage 1 inventory is complete. Await manual scenario selection.

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

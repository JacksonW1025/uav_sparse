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

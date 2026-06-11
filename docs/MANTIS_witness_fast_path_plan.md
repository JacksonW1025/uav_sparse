# MANTIS Witness Fast Path Plan

This witness round is a targeted search for the first valid MANTIS witness, not
a broad benchmark campaign. All accepted claims must come from freshly
pre-registered reruns from this round; old logs may only guide what to test next.

## Witness Definition

A near-worst but valid witness configuration is a triple `(H, P, M)` where:

- `H` is a low-headroom condition that is still admissible under default
  parameters.
- `H` must be default-safe: `M0@P0@H`, `Msmall@P0@H`, and the strong maneuver set
  at `P0@H` must pass the registered safety contracts.
- `P` must be small-input-safe under `H`: `M0@P@H` and `Msmall@P@H` must pass.
- `M` must be default-parameter-safe under `H`: `Mstrong@P0@H` must pass.
- Only the combined arm `Mstrong@P@H` may violate.

Low headroom is allowed only as an environmental or simulator configuration
stress, such as reduced actuator authority, increased mass, reduced thrust
coefficient, or similar reversible SITL-only changes. It is not itself a bug
unless the default-parameter gates above pass.

## Invalid Versus Accepted Outcomes

- Pure parameter bad: `M0@P@H` or `Msmall@P@H` violates. This is rejected even if
  `Mstrong@P@H` also violates.
- Pure input bad: `Mstrong@P0@H` violates. This is rejected because the strong
  maneuver is unsafe without the parameter change.
- Bad low-headroom configuration: default-parameter gates fail under `H`, mode
  harness fails because of `H`, or the vehicle crashes, lands, or failsafes due
  to `H` alone. This is rejected as `BAD_HEADROOM`.
- Accepted MANTIS witness: `Mstrong@P0@H`, `M0@P@H`, and `Msmall@P@H` are safe;
  `Mstrong@P@H` violates `C_recover` or `C_track`; nonlinear activation is
  observable and overlaps the relevant violation window; and the result is
  repeat-robust.

Saturation alone is not a violation. A pure bad PID, pure aggressive input, or
unreadable or illegal parameter setting is not a MANTIS witness.

## Pre-Registered Contracts

### C_recover: Post-Neutral Residual-Rate Contract

`C_recover` is the existing post-neutral residual-rate contract:

- Applies to roll and pitch.
- Terminal residual-rate threshold: `0.35 rad/s`.
- Uses the existing terminal window, non-decay logic, and robust repeat rule
  already implemented for the MANTIS pilot workflow.

### C_track: Active-Window Rate-Tracking Contract

`C_track` is added for this witness round and applies only when both PX4 ULog
topics are available:

- `vehicle_rates_setpoint`
- `vehicle_angular_velocity`

For target axis `a in {roll, pitch}`:

```text
e_a(t) = omega_sp_a(t) - omega_a(t)
```

The active window starts at the first non-neutral target-axis input or rate
setpoint and ends at release-to-neutral. The neutral tail is excluded. If the
rate setpoint is unavailable, `C_track` is not used for an accepted witness and
the result must report `C_track_unavailable`.

Metrics:

- `rms_sp = RMS(omega_sp_a)` over the active window.
- `rms_err = RMS(e_a)` over the active window.
- `nte = rms_err / max(rms_sp, 0.25 rad/s)`.
- `peak_err = max(|e_a|)`.
- `high_err_duration_s = duration(|e_a| >= 1.0 rad/s)`.
- `saturation_error_overlap_s = duration(|e_a| >= 1.0 rad/s and nonlinear
  saturation or limit diagnostic is active)`.

Pre-registered `C_track` violation:

```text
C_track_violation is true iff all hold:
  nte >= 0.65
  peak_err >= 1.0 rad/s
  high_err_duration_s >= 0.25 s
  nte >= 2.5 * median_nte_of_same_Mstrong_at_P0_H
  saturation_error_overlap_s >= 0.20 s OR explicit limit/saturation flag active
    during the high-error window
```

`C_track` must also be safe for the control arms:

- `Msmall@P@H`
- `Mstrong@P0@H`

These thresholds are fixed before the witness reruns and must not be tuned after
observing results.

## Why C_track Is Needed

The recovery contract can miss cases where the controller tracks poorly during
the active strong input but recovers quickly after release. `C_track` covers that
active-window failure mode while keeping the differential gates intact: the same
strong input must be safe under default parameters, and the same parameter must
be safe under neutral and small inputs.

## Nonlinear Diagnostic Rule

Nonlinear activation is necessary but not sufficient for an accepted witness.
The violation must still be a contract violation, not saturation alone.

For `C_recover`, nonlinear evidence must be present before or during the
post-neutral failure window.

For `C_track`, nonlinear activation must overlap the high-error active window:
`saturation_error_overlap_s >= 0.20 s`, or an explicit limit/saturation flag must
be active during the high-error window.

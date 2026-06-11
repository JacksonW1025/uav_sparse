# MANTIS Pilot Plan

MANTIS means Maneuver-Aware Nonlinear Testing of Inner-loop Stability.

It is not the old CADET POSCTL outer-loop trigger story. CADET fixed tuning and
searched feasible pilot inputs that violated a post-neutral outer-loop contract.
MANTIS changes the object under test: it asks whether a rate/attitude tuning
configuration that is accepted by the controller and safe under hover and small
inputs can fail recovery after a stronger but still feasible maneuver pushes the
inner loop into nonlinear limits.

## Bug Definition

An accepted MANTIS bug requires all four differential arms:

- `Safe(M_strong, P0)`
- `Safe(M0, P)`
- `Safe(M_small, P)`
- `Violation(M_strong, P)`
- `NonlinearActivated(M_strong, P)`

`P0` is the default parameter set. `P` is the tested rate/attitude tuning. `M0`
is hover/no-input, `M_small` is a small step or gentle doublet, and `M_strong`
is a stronger step, doublet, or pulse train. Saturation or clipping alone is not
the bug; it is only the diagnostic gate that distinguishes nonlinear activation
from an ordinary bad parameter or ordinary bad input.

## Contracts

The pilot uses pre-registered post-neutral residual-rate contracts:

- `post_neutral_roll_rate`: terminal peak roll rate below `0.35 rad/s`.
- `post_neutral_pitch_rate`: terminal peak pitch rate below `0.35 rad/s`.
- `post_neutral_yaw_rate`: terminal peak yaw rate below `0.261799388 rad/s`.

Tier 1 checks the terminal tail window. Tier 2 requires non-decay evidence by
terminal/start ratio or full-tail slope. Repeated decisions use the existing
2-sigma robust rule.

## Nonlinear Activation

MANTIS requires evidence that the strong maneuver under `P` activated a nonlinear
limit: actuator/motor saturation, mixer clipping, integrator windup, rate/angle
limit, thrust-headroom depletion, or similar telemetry. Current MAVLink parsed
logs expose attitude, angular rates, position, velocity, mode, and manual inputs,
but not those nonlinear diagnostics. Until raw ULog/BIN nonlinear parsing is
implemented, promising four-arm patterns must be reported as
`CONDITIONAL_CANDIDATE_NOT_ACCEPTED`.

## Maneuvers

Primary bug-oracle maneuvers are release-based:

- `M0`: hover/no-input.
- `M_small`: small roll/pitch step or doublet at amplitude `0.20`.
- `M_strong`: stronger roll/pitch steps, doublets, or pulse trains at amplitudes
  `0.5`, `0.7`, and `0.9`.

Chirp is scout-only. It can help diagnose rate response, but it is not a primary
accepted-bug oracle unless separately pre-registered.

Actual excitation is not inferred from commanded theta alone. Each run records
actual peak roll/pitch/yaw rate during the active window and manual input energy.
If rate setpoint telemetry is unavailable, the report states that actual rate and
manual input are being used as stress proxies.

## Modal Sparsity

The pilot keeps the search axis-local:

- roll contract uses roll maneuvers and roll rate/attitude gains;
- pitch contract uses pitch maneuvers and pitch rate/attitude gains.

Cross-axis energy is a diagnostic, not the primary search dimension.

## Go/No-Go

Go requires:

- tests pass;
- dry-run produces `mantis_plan.json` and empty report tables;
- simulator root and MAVLink are available;
- parameter default readback succeeds;
- default strong maneuvers are safe under `P0`;
- at least one parameter remains safe for `M0` and `M_small`;
- a strong maneuver creates a strong-unsafe sliver;
- nonlinear activation is observable and activated for acceptance.

No-Go is reported without overclaiming if any required arm, mode/param harness,
repeat robustness, or nonlinear observability gate fails.

# Transition Handoff Contract Preregistration v2

Date: 2026-06-08

Status: pre-data. This preregistration is written before any v2 transition-handoff
exploratory or confirmatory run.

## Generalizable Template

The experiment is parameterized by a transition pair
`(perturb_mode -> observe_mode)`. For each pair, the test uses the same feasible
pilot maneuver in two arms and changes only whether the mode-transition command
is issued at `t_switch`.

Long-term target set, to be added only after the Position->Hold prerequisite
passes and after pair-specific documentation-contract review plus harness support:

- PX4 `Position->Hold`
- PX4 `Altitude->Position`
- PX4 `Position->Altitude`
- PX4 `Mission/RTL interruption`
- ArduPilot `ALT_HOLD` family, reusing the already runnable `ap_althold` path

This round instantiates and runs only pair #1: PX4 `Position->Hold`.

## Pair #1 Contract

- Pair: PX4 `Position -> Hold/LOITER`
- Contract property: `post_transition_xy_velocity`
- Expected behavior: after the transition settles, horizontal velocity should
  approach zero.
- Threshold: `1.0 m/s`, frozen as `0.10 * MPC_VEL_MANUAL(10)`.
- Terminal decision window: absolute `[11.0, 13.0] s`.
- Diagnostic subwindows: `[5,7]`, `[7,9]`, `[9,11]`, `[11,13]`.
- Documentation source: https://docs.px4.io/main/en/flight_modes_mc/hold
- Documentation fragment: "stop and hover at its current GPS position and altitude"

Only the Hold documentation contract is used. No claim may rely on non-contractual
implementation expectations.

## Arms

The two arms use the same maneuver, tick by tick. The only behavioral difference
is the transition command:

- SW: scenario `px4_transition`; at `t_switch`, the harness requests PX4
  Hold/LOITER.
- NS: scenario `px4_position`; no transition command is sent.

Maneuver family:

- Use feasible pilot input in `[0, t_switch]` to build horizontal velocity.
- Return to neutral by the fastest feasible ramp allowed by `F`, so the input is
  neutral at `t_switch`.
- From `t_switch` onward all pilot inputs remain neutral.
- SW additionally sends the Hold/LOITER command at `t_switch`.

Audit hard constraint: any non-zero pilot stick after `t_switch` invalidates the
run for this experiment, because it can kick autonomous Hold back to Position.

## Frozen Feasible Set And Parameters

- Do not relax feasible set `F`: input limits, per-window rate limits, and
  human-sendable manual-control sequences remain fixed.
- Input limits and slew limits are those in the active harness configuration.
- Every PX4 run must explicitly set and read back:
  - `MPC_ACC_HOR = 3.0`
  - `MPC_JERK_MAX = 8.0`
- `J = 5` for robust decisions.
- `robust_violation` iff `rho_mean + 2*sigma < 0`.
- `robust_safe` iff `rho_mean - 2*sigma > 0`.
- Otherwise the point is `noise_band`.

## Stress Validity Gate

The measured horizontal speed at the observed transition time is the stress
delivered to the handoff logic.

- `V_stress = 2.0 m/s`, frozen as `2 * threshold = 0.2 * MPC_VEL_MANUAL`.
- Any `H_null` claim is valid only for points with measured
  `velocity_at_transition_mps >= V_stress`.
- If the reachable maximum `velocity_at_transition_mps` under `F` and the audit
  constraint is below `V_stress`, the outcome is
  `STRUCTURAL_INCONCLUSIVE`, not `H_null`.

## Hypotheses

These outcomes are mutually exclusive and exhaustive for this round:

- `H_transition`: There exists a maneuver such that SW is terminal
  `robust_violation`, NS is terminal `robust_safe`, the result reproduces across
  seeds 0/1/2, and the point has
  `velocity_at_transition_mps >= V_stress`. This is a true transition-handoff bug.
- `H_velocity_artifact`: Where SW crosses the threshold, NS with the same
  maneuver also crosses it. This is velocity accumulation, not a handoff bug.
- `H_null`: With `velocity_at_transition_mps >= V_stress`, there is no
  reproducible SW threshold crossing. The transition handoff is robust under the
  tested contract.
- `STRUCTURAL_INCONCLUSIVE`: The maximum reachable
  `velocity_at_transition_mps` under `F` and the audit constraint is below
  `V_stress`. The experiment did not stress the handoff enough; stop for human
  decision before changing parameters.

## Prerequisite Go/No-Go Table

| Result | Meaning | Next step |
|---|---|---|
| `H_transition` with cross-seed reproduction, differential attribution, and `>= V_stress` | True transition bug | GO: generalize to more transition pairs and ArduPilot |
| Residual rises monotonically with `velocity_at_transition` and remains differentially attributable, but does not cross `1.0 m/s` | Trend present, threshold not reached | GO, cautious: push higher-speed pairs or conditions |
| `H_null` with `>= V_stress` and robust safety | Handoff also robust | NO-GO: repositioning discussion with negative result and methodology |
| `STRUCTURAL_INCONCLUSIVE` | Feasible input cannot stress the handoff enough | HOLD: human decides whether to relax default parameters to build speed |

## Phase T1: Exploratory Mapping, Seed 0

This phase is exploratory and must be labeled `exploratory=True`.

Step 1, stress first:

- Explicitly set and read back `MPC_ACC_HOR=3.0`, `MPC_JERK_MAX=8.0`.
- Construct a family of late-build, fastest-return-to-neutral maneuvers.
- Sweep amplitude, including saturated `max_abs=1.0` attempts where feasible-set
  projection permits; record saturation status.
- Sweep `t_switch`, biased near `5.0 s` to maximize build duration while
  preserving neutral-at-switch.
- Objective: maximize measured `velocity_at_transition_mps`.
- If the reachable maximum is below `V_stress`, stop and report
  `STRUCTURAL_INCONCLUSIVE`. Do not enter Step 2.

Step 2, differential mapping, only if Step 1 reaches `V_stress`:

- For the subset with `velocity_at_transition_mps >= V_stress`, run SW and NS
  once each on seed 0 with identical maneuvers.
- Record `velocity_at_transition_mps`, `transition_observed_t_s`, terminal peak,
  four subwindows, mode timeline, saturation status, and label.
- Mark attributable candidates where SW terminal peak exceeds `1.0 m/s` and NS
  terminal peak is below `1.0 m/s`.
- Report the residual-vs-transition-speed relationship for SW and NS.

T1 stop rule:

- Attributable candidate found: proceed to T2.
- SW crossing only when NS also crosses: tentatively `H_velocity_artifact`; use
  T2 only for cross-seed confirmation.
- `>= V_stress` and SW all safe: tentative `H_null`; stop and report.
- Reachable maximum `< V_stress`: `STRUCTURAL_INCONCLUSIVE`; stop and report.

## Phase T2: Confirmatory, Conditional

T2 is allowed only if T1 finds at least one attributable candidate. Before T2
data collection, write and commit
`recon_v0/transition_confirmatory_prereg_v2.md`, freezing the criterion band
learned from T1, such as `velocity_at_transition >= V*` and
`t_switch in [a,b]`.

Confirmatory gate:

- For candidate points inside the frozen band, run SW and NS for seeds 0/1/2,
  `J=5` each.
- Accept `H_transition` only if SW is `robust_violation` for all three seeds,
  NS is `robust_safe` for all three seeds, and the point has
  `velocity_at_transition_mps >= V_stress`.

Hit-rate check:

- Compare `N` points sampled inside the criterion band with `N` uniformly random
  feasible `(amplitude, t_switch)` points over the round's feasible grid.
- The criterion band has non-obvious value only if the attributable-violation
  rate inside the band is much larger than the uniform rate.

## Required Outputs

All outputs are written under `recon_v0/`:

- `transition_exploratory_v2.csv`
- `transition_confirmatory_prereg_v2.md`, only if T2 is reached
- `transition_confirmatory_v2.csv`, only if T2 is reached
- `transition_hitrate_v2.csv`, only if T2 is reached
- `survivors.csv`, updated only if T2 accepts survivors
- `summary.md`, updated with the STOP answer, go/no-go placement, and reachable
  maximum `velocity_at_transition_mps`

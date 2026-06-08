# Transition Amendment 04

Date: 2026-06-08

Status: pre-data for Amendment 04. Earlier v2 T1 stress rows, if present in the
workspace, are pre-A04 exploratory data and are not part of this amendment's
data collection. No A04 data may be run before this file is committed.

## Amendment 04: longer feasible build + J=1 stress map

Discipline is unchanged:

- Feasible set `F` is not relaxed.
- No jump inputs, no over-rate changes, and no non-zero pilot stick at or after
  `t_switch`.
- PX4 parameters remain pinned to the defaults `MPC_ACC_HOR=3.0` and
  `MPC_JERK_MAX=8.0`, with explicit set and readback on every run.
- `V_stress=2.0 m/s`, the `1.0 m/s` terminal threshold, differential
  attribution, anti-trap documentation constraint, exploratory-to-confirmatory
  separation, and cross-seed confirmatory gate all remain frozen.

## 1. Stress-map repeat count

`velocity_at_transition_mps` is treated as an approximately deterministic
kinematic stress measurement. The exploratory stress envelope map changes from
`J=5` to `J=1`.

`J=5` remains required for differential seed-0 labels and for confirmatory
robust labels.

## 2. Stress source

The stress source is changed from a short build window to a longer feasible build
window:

1. Ramp at the feasible per-window rate toward near-full stick.
2. Hold the input to let horizontal speed approach `MPC_VEL_MANUAL`.
3. Return to neutral at the fastest feasible rate.
4. Be neutral at `t_switch`.
5. In SW only, request Hold/LOITER at `t_switch`.

The sweep increases `t_switch` and the corresponding build window, e.g.
`t_switch in {5,6,8,10}` subject to harness limits. This is the only allowed
aggressive change. We do not increase `MPC_ACC_HOR` or any other motion
parameter, because that would reintroduce the speed-accumulation confound and can
make the NS arm fail for the same maneuver.

## 3. Relative terminal window

The terminal decision window is translated with the event:

- SW window: `[t_switch + 6, t_switch + 8]`
- NS window: the same `[t_switch + 6, t_switch + 8]`, using the nominal
  `t_switch` as the reference event for neutral release.

This preserves the original semantics of `[11,13] = t_neutral + 6..8` when
`t_switch=5`. The simulation horizon must be extended to cover the relative
terminal window plus margin.

Diagnostic subwindows are also relative:

- `[t_switch + 0, t_switch + 2]`
- `[t_switch + 2, t_switch + 4]`
- `[t_switch + 4, t_switch + 6]`
- `[t_switch + 6, t_switch + 8]`

## 4. Target inherited-speed band

The target inherited-speed band is:

`velocity_at_transition_mps in [V_stress=2.0, NS safe upper bound]`.

The lower bound proves the handoff was actually stressed. The upper bound keeps
the same-maneuver NS arm safe so that any SW-only terminal violation can still be
attributed to the transition handoff rather than generic velocity accumulation.

Existing default-parameter Position data indicates Position can still stop from
roughly `4.7 m/s`, so the band is expected to be broad, but the A04 exploratory
run must measure the NS safe upper bound rather than assume it.

## 5. Unchanged outcome logic

T1 exploratory stopping rules remain:

- Stress still below `V_stress`: `STRUCTURAL_INCONCLUSIVE`; stop, no `H_null`.
- Stress reaches `V_stress` and SW/NS both safe: tentative `H_null`; stop.
- Stress reaches `V_stress` and SW/NS both violate: tentative
  `H_velocity_artifact`; stop.
- Stress reaches `V_stress` and SW is robust violation while same-maneuver NS is
  robust safe: attributable candidate; proceed to T2 preregistration.

T2 remains unchanged: write the confirmatory preregistration first, freeze the
criterion band, run SW and NS for seeds 0/1/2 with `J=5`, require SW all
`robust_violation`, NS all `robust_safe`, and
`velocity_at_transition_mps >= V_stress`.

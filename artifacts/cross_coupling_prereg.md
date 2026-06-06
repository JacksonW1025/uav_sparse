# Cross-coupling residual-rate preregistration

Date: 2026-06-06. This file freezes the local cross-coupling protocol before
any cross-coupling probe data are generated in this run.

Provenance note: `artifacts/cross_coupling_prereg.md` was requested by the
operator but was absent at task start. This copy materializes the operator's
provided protocol text before running Step A. Reports must preserve this
provenance gap and must not hide it as a pre-existing artifact.

Additional local implementation note: after an aborted initial runner start, the
horizontal natural-pair labels in this generated copy were corrected to match the
operator's direct-channel/diagonal intent for horizontal outputs
(`vx<-roll`, `vy<-pitch`). This does not change measured `G`, thresholds,
repeats, or inputs, but it means conclusions from this run must be labeled
`exploratory-hypothesis`, not `confirmatory-protocol`.

## Frozen ground

- Scenario: PX4 `px4_position` only.
- Seed: 0.
- Input parameterization: D=40, 10 windows x 0.5 s x channels
  `{roll,pitch,yaw,throttle}`, followed by the 8 s neutral tail.
- Stick limit for this protocol: `min_value=-1.0`, `max_value=1.0`.
- Repeats per point: J=5.
- Robust gate and Tier definitions are inherited from
  `artifacts/residual_rate_prereg.md`: two sigma; Tier 1 terminal window
  `[11.0,13.0] s`; Tier 2 non-decay over the full tail.
- Property threshold binding is inherited from
  `artifacts/residual_rate_prereg.md`; the only violation target output is
  `post_neutral_xy_velocity` with threshold `1.0 m/s`.
- Delta probe: `delta_probe=0.2`.
- Interior/saturation bins and support threshold are inherited from
  `artifacts/residual_rate_prereg.md`.
- No threshold, support threshold, candidate threshold, Tier rule, or budget may
  be changed after seeing cross-coupling data.

## Step A: empirical interaction matrix and RGA analogue

Outputs are signed terminal residual rates:

- `vx`: mean `vx_mps` on absolute terminal window `[11.0,13.0] s`.
- `vy`: mean `vy_mps` on absolute terminal window `[11.0,13.0] s`.
- `vz`: mean `vz_mps` on absolute terminal window `[11.0,13.0] s`.
- `yaw_rate`: mean `yaw_rate_rps` on absolute terminal window
  `[11.0,13.0] s`; if absent, derive it from unwrapped `yaw_rad`.

Inputs are `{roll,pitch,yaw,throttle}`. For each input channel, apply a
mid-segment envelope with base internal amplitude `0.5`, perturb it with
`+delta_probe` and `-delta_probe`, and return to neutral. The fixed local
envelope is windows 3 through 6 inclusive, i.e. nominal active interval
`[1.5,3.5) s`; `project_theta` may add ramp-limited neighboring values.

For each output/input pair, compute

`G[output,input] = (mean_signed_output_plus - mean_signed_output_minus) /
(2 * delta_probe)`.

Also report the per-output normalized absolute mass share
`abs(G_ij) / sum_j abs(G_ij)`. Compute the RGA analogue
`Lambda = G * transpose(inv(G))` with elementwise multiplication. If exact
inversion fails, the run is not a confirmatory RGA decision.

Natural pairings for the candidate rules are:

- `vx`: `roll`.
- `vy`: `pitch`.
- `vz`: `throttle`.
- `yaw_rate`: `yaw`.

Cross-coupling candidate rule: candidate `(input j -> output i)` if `i` is
`vx` or `vy`, `j` is not the natural pairing for `i`, and either the normalized
absolute mass share is at least `0.20` or `abs(Lambda_ij)` is at least `0.30`.

Interaction candidate rule: for each 2-channel input set `{j1,j2}`, inspect the
RGA submatrix on the columns `{j1,j2}` and the rows naturally paired with
`{j1,j2}`. Mark the pair as a candidate if any inspected entry has
`abs(Lambda) >= 1.5` or `Lambda <= 0`.

Step A go/no-go:

- No cross-coupling candidates and no interaction candidates: stop and report
  the cross-channel bug as not present under this structure.
- Any candidate: H-RGA-1 holds. Stop at the Step A go decision and report the
  candidate set before running Step B unless the operator authorizes Step B in
  the same run.

## Step B: mild cross-coupling violation probe

Only Step A candidates are eligible. For each cross-coupling candidate input set
and each interaction candidate pair, run channel-directed envelope, duration
sweep, and amplitude bisection on the candidate channels only. Target an
interior, returned-to-neutral, Tier 1 robust `post_neutral_xy_velocity`
violation, and record Tier 2. Use a matched-budget uniform random arm.

H-RGA-2 holds only if a candidate input set produces an interior, returned to
neutral, Tier 1 robust `post_neutral_xy_velocity` violation and the supporting
point is channel-pure for that candidate set. For interaction candidates, the
combined input must violate while each single channel at the same per-channel
amplitude does not robustly violate, so the evidence is attributed to the
interaction rather than a single channel.

If H-RGA-2 holds, stop and report. Seeds 1/2, ddmin, and distinct-cost analyses
are not authorized by this preregistration.

## Output contract

Write `artifacts/cross_coupling_summary/cross_coupling_report.md` plus:

- `interaction_matrix.csv`
- `rga.csv`
- `candidates.csv`
- `arm_purity_tier1.csv`
- `arm_purity_tier2.csv`
- `signatures.csv`

Every conclusion must be labeled `confirmatory-protocol` or
`exploratory-hypothesis`, `PX4`, property name, and Tier level. Cross-platform
claims require ArduPilot. Archives must not include `*.ulg`.

# Residual-rate migration preregistration

Date: 2026-06-05. This file freezes the confirmatory PX4 `px4_position`
protocol before any residual-rate evaluation is run.

## Frozen ground

- Scenario: PX4 `px4_position` only.
- Input parameterization: D=40, 10 windows x 0.5 s x channels `{roll,pitch,yaw,throttle}`, followed by the 8 s neutral tail.
- Stick limit for this protocol: `min_value=-1.0`, `max_value=1.0`.
- Repeats per point: J=5.
- Robust gate: two sigma. For robustness `rho`, robust violation is `rho_mean + 2*rho_std < 0`; robust safe is `rho_mean - 2*rho_std > 0`; otherwise noise band.
- Interior/saturation bins: `INTERIOR_MAX_ABS=0.5`; `SATURATED_MIN_ABS=0.9`; `support_abs_threshold=0.1`.
- Arm budget: 80 J=5 points per arm.
- H-1 perturbation: `delta_probe=0.2`.
- Nonzero seed replication, if Phase 2 is later authorized, must use the existing explicit `--allow-nonzero-seed` flag. No threshold or budget changes are allowed across seeds.

## Properties

Two new rate properties are confirmatory targets:

- `post_neutral_climb_rate`: residual vertical speed after stick neutral. Active channel from control allocation: `derive_A_phi=throttle`. The simulator log field is `vz_mps`; the metric uses `abs(vz_mps)`.
- `post_neutral_yaw_rate`: residual yaw angular speed after stick neutral. Active channel from control allocation: `derive_A_phi=yaw`. The simulator log field is the ATTITUDE yaw speed about body/local z, recorded as `yaw_rate_rps`; the metric uses `abs(yaw_rate_rps)`.

Control-allocation derivation: in PX4 Position mode, roll/pitch drive horizontal acceleration/velocity; collective throttle is scaled to vertical velocity through `MPC_Z_VEL_MAX_UP/DN`; yaw stick is scaled to yaw-speed setpoint through `MPC_MAN_Y_MAX`. Therefore `A_xy_velocity={roll,pitch}`, `A_climb_rate={throttle}`, and `A_yaw_rate={yaw}`.

## PX4 parameter binding and thresholds

The SITL target used by CADET is `make px4_sitl jmavsim`, which selects airframe `10017_jmavsim_iris`. That airframe sources `rc.mc_defaults` and does not override the rate/deadzone parameters below. CADET's `PX4Adapter.prepare()` only sets `COM_RC_IN_MODE` and `MIS_TAKEOFF_ALT`, so the following compiled PX4 defaults are the run configuration for the relevant parameters:

| Axis/property | PX4 parameter(s) | Value in run config | Source |
| --- | --- | ---: | --- |
| stick hold deadzone | `MPC_HOLD_DZ` | `0.1` | `/home/car/PX4-Autopilot/build/px4_sitl_default/parameters.json`; defined in `multicopter_position_mode_params.c` |
| horizontal manual speed | `MPC_VEL_MANUAL` | `10.0 m/s` | same |
| vertical manual ascent speed | `MPC_Z_VEL_MAX_UP` | `3.0 m/s` | same |
| vertical manual descent speed | `MPC_Z_VEL_MAX_DN` | `1.5 m/s` | same |
| yaw manual rate | `MPC_MAN_Y_MAX` | `150 deg/s = 2.617993878 rad/s` | same |

Fixed threshold rule: use the hold-deadzone rate equivalent as the lower bound and the same fixed fraction of the documented max manual rate as the decision threshold. The fixed fraction is `MPC_HOLD_DZ = 0.10`.

- Existing `post_neutral_xy_velocity`: threshold `0.10 * MPC_VEL_MANUAL = 1.0 m/s`. This exactly matches the existing `xy_velocity` threshold; no existing result is changed, only the source is documented.
- New `post_neutral_climb_rate`: threshold `0.10 * max(MPC_Z_VEL_MAX_UP, MPC_Z_VEL_MAX_DN) = 0.10 * 3.0 = 0.3 m/s`. The descent-side deadzone equivalent is `0.15 m/s`; the axis threshold uses the larger documented absolute manual rate.
- New `post_neutral_yaw_rate`: threshold `0.10 * MPC_MAN_Y_MAX = 15 deg/s = 0.261799388 rad/s`.

No threshold may be changed after seeing residual-rate data.

## Tier definitions

Every evaluated point records both tiers. The Tier 1 robustness value is the property robustness used for search/bracketing.

- Tail start: `t_neutral=5.0 s`.
- Tail-start comparison window: `[5.0, 7.0] s`.
- Terminal window: `[11.0, 13.0] s`.
- Full tail window for slope: `[5.0, 13.0] s`.

Tier 1, braking failure:

- Per repeat, compute `terminal_peak_abs_rate = max(abs(rate))` on `[11.0,13.0]`.
- `rho_tier1 = threshold - terminal_peak_abs_rate`.
- Tier 1 robust violation iff `rho_tier1_mean + 2*rho_tier1_std < 0`.

Tier 2, non-convergent braking failure:

- Tier 2 requires Tier 1 robust violation.
- Per repeat, compute `tail_start_peak_abs_rate = max(abs(rate))` on `[5.0,7.0]`.
- Ratio non-decay margin: `terminal_peak_abs_rate - 0.80 * tail_start_peak_abs_rate`.
- Slope non-decay margin: ordinary least-squares slope of `abs(rate)` over `[5.0,13.0]`; units are property units per second.
- Non-decay is robust iff either `ratio_margin_mean - 2*ratio_margin_std >= 0` or `slope_margin_mean - 2*slope_margin_std >= 0`.
- Tier 2 robust violation iff Tier 1 is robust and robust non-decay holds.

## Phase 0

For each property, run a cheap saturated-channel sanity check with J=5:

- `post_neutral_climb_rate`: zero anchor, full positive throttle, full negative throttle.
- `post_neutral_yaw_rate`: zero anchor, full positive yaw, full negative yaw.

If no saturated predicted-channel point violates Tier 1, stop that property and report it as not violable under this protocol.

## Phase 1, seed 0

H-1 channel-mass probe:

- Use the Phase 0 violating sign on the predicted channel to bracket a Tier 1 boundary from zero to full-stick.
- At the selected boundary point, perturb every D=40 group by `+delta_probe` and `-delta_probe`, each with J=5.
- Compute directional sensitivity from the Tier 1 terminal-rate metric and aggregate mass by channel and by channel/sign.
- H-1 confirms migration iff the predicted channel is the top channel and its mass share is at least `0.50`.

H-2 direct synthesis:

- Run the three matched arms with 80 J=5 points per arm:
  - Arm A: uniform random feasible points.
  - Arm B: channel-unaware random endpoint plus internal bracketing.
  - Arm C: channel-directed envelopes on `A_phi`, with duration sweep and distinct signatures.
- Arm C signature is `channel set + active window band + sign`; amplitude is excluded.
- H-2 confirms at a given Tier iff Arm A interior violations are `0`, Arm B channel-pure interior ratio is `<=0.10`, and Arm C has `>0` interior violations with channel-pure interior ratio `>=0.90`.
- Internal-violation existence is reported separately for Tier 1 and Tier 2.

Go/no-go per property:

- If H-1 and H-2 both confirm at least under Tier 1, the property may proceed to Phase 2.
- If H-1 is falsified, the criterion does not migrate to that predicted channel; stop and report negative.
- If H-2 has no interior violations in both tiers, the channel has only saturated violations; stop and report that the bug class does not migrate to that channel.

## Phase 2, not run until authorized

Only properties that pass Phase 1 are eligible. If later authorized, run seeds 1 and 2 plus the ddmin baseline with the existing nonzero-seed unlock. Report channel-pure interior violations and per-distinct cost separately for Tier 1 and Tier 2. Distinct maneuver identity is `channel set + active window band + sign`; amplitude is excluded. Do not claim quantity dominance.

## Output contract

Write `artifacts/residual_rate_summary/residual_rate_report.md` plus:

- `channel_mass.csv`
- `arm_purity_tier1.csv`
- `arm_purity_tier2.csv`
- `distinct_costs.csv`
- `signatures.csv`

Every conclusion must be labeled `confirmatory`, `PX4`, property name, and Tier level. Cross-platform claims require ArduPilot. Archives must not include `*.ulg`.

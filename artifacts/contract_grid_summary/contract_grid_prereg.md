# Contract–Input–Bug grid preregistration v0

Date: freeze and commit BEFORE any grid evaluation is run. Scope: PX4
`px4_position`, seed 0, Phase-0 screening. This file widens the unit of study
from a single (contract, input, channel) point to a small grid of documented
flight-mode contracts crossed with feasible pilot-maneuver classes. Every
contract is derived from official PX4 documentation and bound to a PX4
parameter (PGFuzz-style provenance). Thresholds are fixed here, before data.

## Frozen ground (unchanged from prior protocols)
- Scenario: PX4 `px4_position`. D=40 (10 windows x 0.5 s x {roll,pitch,yaw,throttle}); stick limit +/-1.0; per-step rate limit as in `project_theta`.
- J=5 repeats. Robust 2-sigma gate: robust violation iff `rho_mean + 2*rho_std < 0`; robust safe iff `rho_mean - 2*rho_std > 0`; else noise band.
- `INTERIOR_MAX_ABS=0.5`, `SATURATED_MIN_ABS=0.9`, `support_abs_threshold=0.1`.
- `t_neutral = 5.0 s`. Tail = [5,13] s. No `*.ulg` in archives.

## PX4 parameter binding (read actual compiled values, record them)
The Agent reads these from `/home/car/PX4-Autopilot/build/px4_sitl_default/parameters.json` (same source as `residual_rate_prereg.md`) and records the value used. Known values: `MPC_HOLD_DZ=0.1`, `MPC_VEL_MANUAL=10.0`, `MPC_Z_VEL_MAX_UP=3.0`, `MPC_Z_VEL_MAX_DN=1.5`, `MPC_MAN_Y_MAX=2.617993878`. Read and record: `MPC_XY_VEL_MAX`, `MPC_MAN_TILT_MAX` (documented default 35 deg).

## Log fields required
- Velocity: `vx_mps`, `vy_mps`, `v_xy = hypot(vx,vy)`, `vz_mps`.
- Yaw rate: `yaw_rate_rps`.
- Attitude: `roll_rad`, `pitch_rad`; tilt-from-level `tilt_rad = acos(cos(roll_rad)*cos(pitch_rad))`.

## Contracts (documentation-derived, parameter-bound)
Each contract C-i has a documentation source, a bound parameter, an STL-style
robustness `rho` (>=0 satisfied, <0 violated), and a frozen threshold.

- **C1 Brake — residual rate to zero after centering.** Doc: centered sticks make the vehicle actively brake to a stop. Metric: terminal window `[11,13] s` peak absolute rate. `rho = threshold - peak`. Thresholds (0.10 x documented max manual rate): `xy_velocity 1.0 m/s` (`0.10*MPC_VEL_MANUAL`), `climb_rate 0.3 m/s` (`0.10*MPC_Z_VEL_MAX_UP`), `yaw_rate 0.261799 rad/s` (`0.10*MPC_MAN_Y_MAX`).
- **C2 Level — residual attitude to level after centering.** Doc: centered sticks level the vehicle. Metric: terminal window `[11,13] s` peak `tilt_rad`. `rho = level_threshold - peak_tilt`. Threshold: `0.10 * MPC_MAN_TILT_MAX = 0.10 * 35 deg = 3.5 deg = 0.061087 rad`.
- **C3 Envelope — documented ceiling not exceeded during sustained input.** Doc: full deflection ramps to the documented max manual velocity; tilt is bounded by the max-tilt parameter. Metric: active window `[2,5] s` peak. `rho = ceiling - peak`. Ceilings (read from params, exact documented hard limits, not a fraction): horizontal speed ceiling `MPC_XY_VEL_MAX`; tilt ceiling `MPC_MAN_TILT_MAX`; climb-rate ceiling `MPC_Z_VEL_MAX_UP` (up) / `MPC_Z_VEL_MAX_DN` (down). Violation = peak exceeds ceiling robustly.
- **C4 Coupling — no cross-axis residual rate after centering under coordinated/cross input.** Doc: roll/pitch govern horizontal, throttle governs vertical, yaw governs yaw rate; the centered-stick brake-and-hold guarantee applies to all axes. Metric: terminal window `[11,13] s` peak absolute rate of each axis, evaluated under an input whose commanded channel(s) are NOT the natural driver of that axis. `rho` and thresholds identical to C1, applied to the cross axis.

## Input taxonomy (feasible, rate-limited pilot maneuvers)
- **I1 Pulse-return.** A mid-band envelope on the commanded channel(s) over the active phase, then return to neutral for the 8 s tail. Evaluation window: terminal `[11,13] s` (post-neutral).
- **I2 Step-hold.** Sustained deflection on the commanded channel held through the active phase `[0,5] s` (no return within the active phase). Evaluation window: active `[2,5] s` (during sustained input).
- **I3 Coordinated.** Two channels deflected together (pulse-return form). Evaluation window: terminal `[11,13] s` (post-neutral).

## Phase-0 grid (cells to screen this pass)
Each cell is one (input class, contract, commanded channel-set). For each cell, run the cheap screen: a zero anchor (all-neutral, J=5) plus representative strong feasible probe(s) on the commanded channel-set (single-channel cells: saturated +/- on that channel; I2 cells: sustained high deflection; I3 cells: both channels at high feasible deflection, with sign variants), J=5 each, robust 2-sigma gate.

| cell | input | contract | commanded channels | axis measured | window |
| --- | --- | --- | --- | --- | --- |
| G01 | I1 | C1 Brake | roll | xy_velocity | [11,13] |
| G02 | I1 | C1 Brake | pitch | xy_velocity | [11,13] |
| G03 | I1 | C1 Brake | throttle | climb_rate | [11,13] |
| G04 | I1 | C1 Brake | yaw | yaw_rate | [11,13] |
| G05 | I1 | C2 Level | roll | tilt | [11,13] |
| G06 | I1 | C2 Level | pitch | tilt | [11,13] |
| G07 | I2 | C3 Envelope | roll | v_xy, tilt | [2,5] |
| G08 | I2 | C3 Envelope | pitch | v_xy, tilt | [2,5] |
| G09 | I2 | C3 Envelope | throttle | climb_rate | [2,5] |
| G10 | I3 | C4 Coupling | throttle (alone) | xy_velocity | [11,13] |
| G11 | I3 | C4 Coupling | roll+throttle | xy_velocity, climb_rate | [11,13] |
| G12 | I3 | C4 Coupling | pitch+throttle | xy_velocity, climb_rate | [11,13] |

## Per-cell record and go/no-go
For every cell record: commanded channel-set, probe descriptor, `peak` per measured axis, `rho_mean`, `rho_std`, `rho_mean + 2*rho_std`, and the robust label (`robust_violation` / `robust_safe` / `noise_band`). A cell is marked `violable` iff at least one of its probes is a robust violation under the frozen threshold. Cells marked `violable` are eligible for Phase 1 (full three-arm + interior bisection, later, on authorization). Report every cell's status; report values for all probes regardless of label.

## Discipline
- Thresholds are frozen above and bound to documented PX4 parameters; do not change any threshold after seeing data.
- This file must be committed to the repository before data collection to qualify conclusions as `confirmatory-protocol, PX4, seed 0, Phase-0`. If committed after, label `exploratory-hypothesis`.
- Phase 1 / cross-seed / ddmin / ArduPilot are out of scope for this pass.

## Output contract
Write `artifacts/contract_grid_summary/contract_grid_report.md` plus:
- `params_used.csv` (every parameter value read and the threshold derived from it)
- `grid_cells.csv` (one row per cell: status + go/no-go eligibility)
- `probe_points.csv` (one row per probe: peak per axis, rho_mean, rho_std, label)
Every conclusion labeled with provenance, PX4, seed 0, Phase-0, and the contract id.
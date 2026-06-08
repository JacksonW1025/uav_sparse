# Transition Hold/Loiter Setpoint Initialization Feasibility

Date: 2026-06-08

PX4 source: `/home/car/PX4-Autopilot`

PX4 commit: `30e763b67800`

Scope: pure source and official-doc reading only. No simulation, no harness/oracle construction, and no runtime-code changes.

## Decision

**NOT_VIABLE**

Current PX4 has no official Hold heading/yaw-stability contract strong enough to support a yaw oracle, and the current source does not expose a tunable small-velocity yaw guard. The position side is also structurally guarded against a stale loiter reference: manual Hold creates a fresh loiter/braking-stop item, recent reposition is time-gated, and AUTO mission-end loiter uses the current loiter setpoint only when it is already a loiter setpoint.

Recommendation under the task rule: do not build a new Hold/Loiter transition harness for this target; treat this as a negative source-only feasibility result and move to relocation/methodology discussion.

## Q1: Contract Provenance

Official PX4 Hold Mode documentation supports a position/stop-hold contract:

- Quote: "stop and hover at its current GPS position and altitude."
- URL: https://docs.px4.io/main/en/flight_modes_mc/hold

The multicopter flight-mode overview gives the same position contract:

- Quote: "Vehicle stops and hovers at its current position and altitude"
- URL: https://docs.px4.io/main/en/flight_modes_mc/

Heading/yaw contract: **not established**. The official Hold Mode page contains no explicit `yaw` or `heading` promise. Therefore, yaw/heading cannot be used as the primary anti-trap contract for "no uncommanded yaw" in this target. It can only be a diagnostic signal unless a stronger official contract is found elsewhere.

## Q2: Setpoint Initialization Source Reading

### Mode Switch Path

The current path for Hold/Loiter is:

- Commander changes finished takeoff/mission to `NAVIGATION_STATE_AUTO_LOITER`: `/home/car/PX4-Autopilot/src/modules/commander/Commander.cpp:1988-2004`.
- Navigator maps `NAVIGATION_STATE_AUTO_LOITER` to `_loiter`: `/home/car/PX4-Autopilot/src/modules/navigator/navigator_main.cpp:762-773`.
- FlightModeManager runs `FlightTaskAuto` for auto-enabled states: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/FlightModeManager.cpp:188-194`.
- Task switching passes the prior trajectory setpoint, then activates the new task: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/FlightModeManager.cpp:372-399`.

### Yaw Initialization

`FlightTaskAuto::activate()` initializes yaw to current yaw and yawspeed to zero, resets position smoothing from the last setpoint/current state, and initializes `_yaw_sp_prev` from the last setpoint yaw if finite, otherwise current yaw:

- `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:43-74`

When the triplet is invalid, `FlightTaskAuto::_evaluateTriplets()` falls back to current position and current yaw:

- `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:360-365`

When a valid triplet exists, yaw is resolved in this order:

- finite triplet yaw wins: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:518-520`
- otherwise `_set_heading_from_mode()` is used: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:522-523`
- if no valid yaw/yawspeed remains, generate heading along trajectory if possible, otherwise keep the previous yaw setpoint/current yaw: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:194-199`

`_set_heading_from_mode()`:

- `MPC_YAW_MODE` 0/4 points toward the current waypoint; 1/2 use home; 3 leaves heading to trajectory-specific generation.
- Outside target acceptance radius it computes heading from the target vector; once inside, it locks yaw to current yaw to prevent excessive yawing.
- Source: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:530-583`

Velocity-derived yaw has two hardcoded source guards:

- `_generateHeadingAlongTraj()` only uses velocity if `vel_sp_xy > 0.1 m/s` and target distance is `> 2 m`: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:769-781`.
- `_compute_heading_from_2D_vector()` has a lower hardcoded vector guard `1e-3`: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:698-711`.

The relevant parameters are not that small-velocity threshold:

- `MPC_YAW_MODE` only selects heading behavior, legal values 0-4: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:165-177`.
- `MPC_YAWRAUTO_MAX` only limits yaw setpoint rate: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:135-148` and `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:308-324`.
- `MIS_YAW_ERR` is a waypoint heading acceptance tolerance, not the velocity guard: `/home/car/PX4-Autopilot/src/modules/navigator/mission_params.c:123-133`.

Status of #11960-like mechanism: the current source still has a velocity-to-heading path, but the "small velocity" entry condition for along-trajectory yaw is now a hardcoded `0.1 m/s` plus `2 m` target-distance guard. The remaining `1e-3` guard is a non-parameterized helper fallback. Thus this is not a parameter-by-maneuver conjunction trigger. If a yaw issue remained, it would be a pure input/source-guard issue, and Q1 does not establish an official heading contract for it.

### Position Initialization

Manual Hold activation:

- Recent reposition triplet, younger than 500 ms, goes through `reposition()`.
- Otherwise manual Hold calls `set_loiter_position()`.
- Source: `/home/car/PX4-Autopilot/src/modules/navigator/loiter.cpp:51-65`.

`Loiter::set_loiter_position()` behavior:

- If already on an orbit loiter pattern and within acceptance plus loiter radius, it preserves that current loiter setpoint.
- Else for rotary-wing vehicles it calls `setLoiterItemFromCurrentPositionWithBreaking()`.
- Else it uses current position.
- It then invalidates previous/next setpoints and publishes a new current triplet.
- Source: `/home/car/PX4-Autopilot/src/modules/navigator/loiter.cpp:91-125`.

The braking-stop helper is explicit:

- `setLoiterItemFromCurrentPositionWithBreaking()` calls `calculate_breaking_stop()` and sets yaw to `NAN`: `/home/car/PX4-Autopilot/src/modules/navigator/mission_block.cpp:770-778`.
- `calculate_breaking_stop()` says multirotors account for braking distance "otherwise the vehicle will overshoot and go back"; it predicts distance from current horizontal velocity using `MPC_JERK_AUTO` and `MPC_ACC_HOR`: `/home/car/PX4-Autopilot/src/modules/navigator/navigator_main.cpp:1476-1489`.

Reposition is not a stale-reference backdoor in this path:

- The reposition triplet must be valid and younger than 500 ms.
- `reposition()` copies that commanded setpoint, then clears the reposition triplet.
- Source: `/home/car/PX4-Autopilot/src/modules/navigator/loiter.cpp:129-154`.

AUTO mission-end loiter is separate from manual Hold:

- Mission end uses the current loiter setpoint only if the current triplet is valid and already a loiter; otherwise it uses current position.
- It invalidates previous/next and marks the mission finished.
- Source: `/home/car/PX4-Autopilot/src/modules/navigator/mission_base.cpp:485-508`.

FlightTaskAuto then projects the navigator triplet defensively:

- If current triplet latitude/longitude are invalid, it locks XY to current local position.
- If projected target XY/Z is invalid, it replaces it with current position.
- If previous setpoint is invalid, previous waypoint becomes current position.
- For loiter, next waypoint equals target.
- Source: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:392-461`.

Status of #20477-like mechanism: current code does not show a general stale-loiter-reference path. The only reuse of a previous loiter setpoint is a narrow "already on orbit loiter" branch, and recent reposition is intentionally time-gated. A moving manual-Hold vehicle can receive a loiter target ahead of the switch-time position, but that is the current braking-stop design, not an accidental stale reference.

## Q3: Trigger Classification

Yaw side:

- Feasible inherited state: a maneuver can enter Hold/Loiter with small lateral speed or a changing velocity direction.
- Current guard: along-trajectory yaw ignores velocity below `0.1 m/s` and when within `2 m` of target.
- Parameter classification: no legal parameter controls these two guard thresholds. `MPC_YAW_MODE=3` can choose along-trajectory yaw, but it does not make the small-speed threshold tunable.
- Contract status: official Hold docs do not explicitly promise heading preservation.
- Result: **not viable** as a doc-backed violation; if pursued anyway, it would be a pure input/source-guard hypothesis, not a parameter-by-maneuver conjunction.

Position side:

- Feasible inherited state: switching while moving changes the braking-stop point.
- Current guard/design: rotary-wing Hold intentionally computes a braking-stop loiter target; normal stale triplets are not reused except for explicit orbit/reposition cases.
- Candidate parameters: `MPC_JERK_AUTO` and `MPC_ACC_HOR` influence braking-stop prediction and trajectory constraints, but they are symmetric motion-planning/controller constraints, not setpoint-initialization reference selectors. `NAV_ACC_RAD` is an acceptance radius, not a reference source.
- Contract status: official docs support stop-and-hover at current position, but the source intentionally interprets moving-entry Hold through a predicted braking stop to avoid overshoot-and-return. A strict "must hold exact switch-time coordinate" oracle would likely flag intended behavior, not a clean current-commit bug.
- Result: **not viable** for a reproducible #20477-like wrong-reference trigger in current source.

## Final Classification

No trigger satisfies all required conditions:

- official contract成立: only position side成立; yaw side未成立；
- feasible inherited-state excitation: yes in principle for moving entry;
- static/common non-trigger: yes, but;
- current commit unguarded and reproducibly wrong: no;
- parameter-by-maneuver conjunction: no tunable yaw guard and no separable position-reference selector.

Therefore the target is **NOT_VIABLE** for a new thin harness/oracle campaign.

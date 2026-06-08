# AUTO brake-overshoot feasibility check

Date: 2026-06-08
PX4 source checkout: `/home/car/PX4-Autopilot`, commit `30e763b678`.
Method: documentation + source reading only. No simulation, no harness edits, no runtime data.

## Decision

**NOT_VIABLE** for the proposed thin experiment.

Reason: the multicopter AUTO endpoint path is source-level preemptive, not a sparse "high speed plus weakened brake permission" mechanism. The trajectory generator uses the remaining distance to the target together with the configured acceleration and jerk limits to reduce the allowed velocity before the endpoint. Lowering the legal acceleration/jerk limits therefore lowers the planned approach speed and starts braking earlier; it does not create a planner-level inability to stop at the final waypoint. I also did not find a legal, static-benign, cruise-separable parameter that only reduces endpoint braking permission while leaving the high-cruise excitation intact.

## Q1: contract provenance

Contract is usable, but it is a hold/loiter stop-and-hover contract rather than an exact "no geometric overshoot past waypoint center" promise.

PX4 official docs:

- Mission Mode (MC): "If flying the vehicle will hold." URL: https://docs.px4.io/main/en/flight_modes_mc/mission
- Mission Mode (MC): "loiter is implemented as hover". URL: https://docs.px4.io/main/en/flight_modes_mc/mission
- Hold Mode (MC): "stop and hover at its current GPS position and altitude." URL: https://docs.px4.io/main/en/flight_modes_mc/hold

Source agrees with the docs:

- Mission completion logs "Mission finished, loitering" if still airborne: `/home/car/PX4-Autopilot/src/modules/navigator/mission_base.cpp:419`.
- End-of-mission creates a loiter mission item when not landed, publishes it as current, invalidates previous/next, and marks mission finished: `/home/car/PX4-Autopilot/src/modules/navigator/mission_base.cpp:485`.
- Commander switches from `AUTO_MISSION` to `AUTO_LOITER` after mission_result.finished: `/home/car/PX4-Autopilot/src/modules/commander/Commander.cpp:1987`.

Implication: terminal-window horizontal velocity is a reasonable contract proxy. Exact final waypoint center overshoot should remain diagnostic only, because mission acceptance radius and final loiter-at-current-position behavior mean the docs do not promise zero geometric overshoot relative to the waypoint center.

## Q2: AUTO lateral deceleration mechanism

Judgment: **preemptive**.

Evidence path:

- In AUTO position/loiter/takeoff cases, `FlightTaskAuto` sets the current target as the position setpoint and leaves velocity setpoint NaN, then calls `PositionSmoothing::generateSetpoints(...)` on `{previous, current, next}` waypoints: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:144` and `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:173`.
- If the mission target is loiter or there is no valid next waypoint, `_triplet_next_wp` is set equal to `_triplet_target`: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:459` and `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:467`.
- `PositionSmoothing::_getMaxXYSpeed()` builds dynamic limits from acceptance radius, `max_acc_xy`, `max_jerk`, cruise speed, and trajectory gain, then calls `computeXYSpeedFromWaypoints`: `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:85`.
- `computeXYSpeedFromWaypoints()` propagates speed backwards through the waypoint list. For the final waypoint, the exit speed is initialized to 0 and the start speed is limited by distance to target: `/home/car/PX4-Autopilot/src/lib/motion_planning/TrajectoryConstraints.hpp:111`.
- `computeStartXYSpeedFromWaypoints()` uses `computeMaxSpeedFromDistance(config.max_jerk, config.max_acc_xy, start_to_target, speed_at_target)`: `/home/car/PX4-Autopilot/src/lib/motion_planning/TrajectoryConstraints.hpp:95`.
- `computeMaxSpeedFromDistance()` explicitly solves maximum speed from remaining distance, acceleration, jerk, and final speed: `/home/car/PX4-Autopilot/src/lib/mathlib/math/TrajMath.hpp:48`.

Tracking-gap check:

- The smoothing layer also slows trajectory integration when the drone is behind the trajectory, stopping integration as horizontal tracking error approaches `MPC_XY_ERR_MAX`: `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:271`.
- This is a guard against setpoint outrunning the vehicle, not a route to induce endpoint residual speed by lowering AUTO jerk/acceleration limits.

Therefore this target is not a clean "reaction-only braking after arrival" setup at the planner level. A physical tracking defect is never impossible in the real vehicle, but this source path does not expose the requested sparse parameter x excitation conjunction.

## Q3: deceleration permission parameter candidates

### `MPC_JERK_AUTO`

- Legal range/default from source: min `1`, max `80`, default `4` m/s^3: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:90`.
- Used by AUTO smoothing as max jerk: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:807`.
- It enters the endpoint speed-from-distance calculation through `PositionSmoothing` and `TrajectoryConstraints`: `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:91` and `/home/car/PX4-Autopilot/src/lib/motion_planning/TrajectoryConstraints.hpp:96`.
- Static benign: likely yes for hover/low speed.
- Cruise-separable: partly yes on long legs, because cruise is still `MPC_XY_CRUISE` or mission speed.
- Can legally cause insufficient endpoint braking: **no**. The same lower jerk is used by the preemptive speed limit, so planned approach speed is reduced before the endpoint.

### `MPC_ACC_HOR`

- Legal range/default from source: min `2`, max `15`, default `3` m/s^2: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:76`.
- Used by AUTO smoothing as horizontal max acceleration: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:805`.
- It directly enters speed-from-distance endpoint planning: `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:94` and `/home/car/PX4-Autopilot/src/lib/motion_planning/TrajectoryConstraints.hpp:96`.
- Static benign: likely yes for hover/low speed.
- Cruise-separable: **no for this hypothesis**. It is the symmetric AUTO acceleration/deceleration design constraint; lowering it also lowers allowable approach speed and agility.
- Can legally cause insufficient endpoint braking: **no** for the same preemptive-planning reason.

### `MPC_XY_TRAJ_P`

- Legal range/default from source: min `0.1`, max `1`, default `0.5`: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:105`.
- Used as horizontal trajectory gain and turn-speed radius scale: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:801` and `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:97`.
- Static benign: likely yes.
- Cruise-separable: not a cruise knob.
- Can legally cause insufficient endpoint braking: **no**. For the final waypoint where next equals target, endpoint exit speed is still 0; this gain is mainly a trajectory/turn shaping term, not a brake permission.

### `MPC_XY_ERR_MAX`

- Legal range/default from source: min `0.1`, max `10`, default `2`: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:116`.
- Used to slow or stop trajectory integration when the vehicle lags behind the trajectory: `/home/car/PX4-Autopilot/src/lib/motion_planning/PositionSmoothing.cpp:271`.
- Static benign: likely yes.
- Cruise-separable: yes, but not a deceleration authority parameter.
- Can legally cause insufficient endpoint braking: **no**. Lower values make the trajectory wait earlier; they do not reduce endpoint braking permission.

### `NAV_ACC_RAD`

- Legal range/default from source: min `0.05`, max `200`, default `10` m: `/home/car/PX4-Autopilot/src/modules/navigator/navigator_params.c:57`.
- Used as default waypoint acceptance radius and can be overridden by mission item acceptance radius: `/home/car/PX4-Autopilot/src/modules/navigator/navigator_main.cpp:1181` and `/home/car/PX4-Autopilot/src/modules/navigator/mission_block.cpp:331`.
- Mission waypoint reached uses distance <= acceptance radius for rotary-wing waypoint items: `/home/car/PX4-Autopilot/src/modules/navigator/mission_block.cpp:395`.
- Static benign: not clean, because it changes the arrival event/contract semantics.
- Cruise-separable: yes, but it is not a deceleration authority knob.
- Can legally cause insufficient endpoint braking: **no**. It changes when a waypoint is accepted; it does not make the final-waypoint speed-from-distance planner ignore acceleration/jerk constraints.

### `MPC_XY_CRUISE` / mission item speed

- Legal range/default from source for `MPC_XY_CRUISE`: min `3`, max `20`, default `5` m/s: `/home/car/PX4-Autopilot/src/modules/mc_pos_control/multicopter_autonomous_params.c:34`.
- `FlightTaskAuto` uses mission triplet cruising speed if valid, otherwise `MPC_XY_CRUISE`, capped by `MPC_XY_VEL_MAX`: `/home/car/PX4-Autopilot/src/modules/flight_mode_manager/tasks/Auto/FlightTaskAuto.cpp:373`.
- This is the excitation knob, not a permission knob.

## Final recommendation

Do **not** build the AUTO mission harness for this target as currently defined.

The intended conjunction needs "high cruise remains high" plus "endpoint braking permission becomes insufficient." PX4's AUTO endpoint planner explicitly couples the candidate permission parameters into the remaining-distance speed limit, so lowering them makes the planned approach more conservative rather than under-braked. `NAV_ACC_RAD` changes arrival semantics, and `MPC_XY_TRAJ_P` / `MPC_XY_ERR_MAX` are trajectory-shaping or tracking-lag guards, not endpoint deceleration permission.

Suggested next target per the instruction: transition yaw / position-initialization, where a genuine mode-switch initialization or setpoint discontinuity can plausibly produce a parameter x excitation conjunction without being neutralized by the AUTO distance-to-speed planner.

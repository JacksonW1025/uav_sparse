# PX4 v1.15.0 Tilt/Attitude-Limit Coverage Matrix

Phase A static audit for `px4_ctrlauth_v1`.

Pinned PX4 target: `v1.15.0`, SHA `30e763b6780061d70a14894e3e8b06e6a656f9b8`, local tree `/mnt/nvme/px4_work/PX4-Autopilot`.

## Static Verdict

`MPC_TILTMAX_AIR x offboard attitude SET_ATTITUDE_TARGET` is **UNCOVERED by static source audit**.

The limit is wired into multicopter position control: `MPC_TILTMAX_AIR` default is 45 deg and is documented as the maximum for velocity/acceleration controlled modes (`src/modules/mc_pos_control/multicopter_position_control_limits_params.c:83-96`). `MulticopterPositionControl` chooses `MPC_TILTMAX_AIR` in flight and calls `PositionControl::setTiltLimit()` (`src/modules/mc_pos_control/MulticopterPositionControl.cpp:475-478`). `PositionControl` then applies `ControlMath::limitTilt(...)` before producing thrust/attitude (`src/modules/mc_pos_control/PositionControl/PositionControl.cpp:214-220`) and publishes the generated `vehicle_attitude_setpoint` (`src/modules/mc_pos_control/MulticopterPositionControl.cpp:535-539`).

The offboard attitude path does not enter that controller. MAVLink `SET_ATTITUDE_TARGET` publishes `offboard_control_mode.attitude=true` and directly publishes `vehicle_attitude_setpoint.q_d` when in OFFBOARD (`src/modules/mavlink/mavlink_receiver.cpp:1541-1607`). Commander maps `offboard_control_mode.attitude` to attitude/rates/allocation only, not position/velocity/acceleration control (`src/modules/commander/ModeUtil/control_mode.cpp:122-161`). `mc_att_control` copies the quaternion setpoint into `AttitudeControl` (`src/modules/mc_att_control/mc_att_control_main.cpp:289-318`). `AttitudeControl` limits generated rate setpoints by `MC_*RATE_MAX`, but does not clamp attitude-setpoint tilt (`src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:55-107`).

## Mechanisms

| Mechanism | Scope in code | Static interpretation |
|---|---|---|
| `MPC_TILTMAX_AIR` | `mc_pos_control` / `PositionControl`; applied when trajectory setpoints are converted to thrust vector and attitude. | Covers position/velocity/acceleration controlled paths, including offboard position/velocity/acceleration after MAVLink publishes `trajectory_setpoint`. Does not cover direct attitude or body-rate setpoints. |
| `MPC_TILTMAX_LND` | Same `PositionControl` tilt slot during takeoff/landing ramp before flight state. | Covers takeoff/landing position-control-generated attitude only. Not a direct offboard attitude clamp. |
| `MPC_MAN_TILT_MAX` | Stabilized/manual attitude generation in `mc_att_control`; stick vector limited before quaternion setpoint publication (`src/modules/mc_att_control/mc_att_control_main.cpp:142-194`). | Covers manual Stabilized-style attitude setpoint generation. Does not cover MAVLink direct attitude setpoints. |
| `MC_ROLLRATE_MAX`, `MC_PITCHRATE_MAX`, `MC_YAWRATE_MAX` | `AttitudeControl::setRateLimit()` and rate-setpoint constrain (`src/modules/mc_att_control/mc_att_control_main.cpp:93-99`; `src/modules/mc_att_control/AttitudeControl/AttitudeControl.cpp:104-107`). | Limits attitude-controller output rates, not commanded attitude tilt. A slow but large attitude target can still be accepted. |
| Acro/manual rate limits (`MC_ACRO_*`, rate controller) | Manual acro stick maps to rate setpoints in `mc_rate_control`; rate controller consumes `vehicle_rates_setpoint`. | Governs rates/torques, not absolute lean angle. |

## Interface Matrix

Legend: `COVERED` = this mechanism is on the command path and can constrain the relevant command before actuator allocation. `N/A` = mechanism does not apply to this interface. `RATE_ONLY` = only rate output is limited; attitude setpoint is not clipped. `DIRECT` = bypasses position-control tilt limiting.

| Command interface | Primary PX4 setpoint path | `MPC_TILTMAX_AIR` | `MPC_TILTMAX_LND` | `MPC_MAN_TILT_MAX` | `MC_*RATE_MAX` | Static note |
|---|---|---:|---:|---:|---:|---|
| Manual Stabilized / Altitude stick attitude | `mc_att_control::generate_attitude_setpoint()` | N/A | N/A | COVERED | RATE_ONLY | Manual stick tilt vector is capped by `_man_tilt_max`; not the position-control air tilt parameter. |
| Manual Position stick | FlightTask trajectory -> `mc_pos_control` -> `PositionControl` | COVERED | COVERED during ramp/land | Indirect/manual task constraints | RATE_ONLY | Final generated attitude from position control is tilt-limited. |
| Auto/Mission position trajectory | Navigator/FlightTask trajectory -> `mc_pos_control` | COVERED | COVERED during ramp/land | N/A | RATE_ONLY | Generated attitude goes through `PositionControl::limitTilt`. |
| Offboard position | MAVLink `SET_POSITION_TARGET_*` -> `trajectory_setpoint` -> `mc_pos_control` | COVERED | COVERED during ramp/land | N/A | RATE_ONLY | `mavlink_receiver.cpp:1045-1068` publishes trajectory setpoints for OFFBOARD. |
| Offboard velocity | MAVLink `SET_POSITION_TARGET_*` -> `trajectory_setpoint` -> `mc_pos_control` | COVERED | COVERED during ramp/land | N/A | RATE_ONLY | Velocity setpoints still enter position control. |
| Offboard acceleration | MAVLink `SET_POSITION_TARGET_*` -> `trajectory_setpoint` -> `mc_pos_control` | COVERED | COVERED during ramp/land | N/A | RATE_ONLY | Acceleration setpoints are limited by thrust-vector tilt in `PositionControl`; FORCE_SET is rejected. |
| **Offboard attitude quaternion** | MAVLink `SET_ATTITUDE_TARGET` -> `vehicle_attitude_setpoint` -> `mc_att_control` | **N/A / UNCOVERED** | N/A | N/A | RATE_ONLY | Candidate gap. Direct `q_d` publication bypasses `mc_pos_control`; no tilt clamp found in `mc_att_control`. |
| Offboard body rate | MAVLink `SET_ATTITUDE_TARGET` -> `vehicle_rates_setpoint` -> `mc_rate_control` | N/A / UNCOVERED | N/A | N/A | Depends on upstream; direct rate setpoint is consumed by rate controller | Direct rate path has no absolute tilt ceiling. Not the Phase B primary candidate. |
| Offboard thrust/torque or direct actuator | Offboard allocation/direct actuator paths | N/A | N/A | N/A | N/A | Outside tilt/attitude-setpoint governance; not used in this experiment. |
| Acro rate | Manual acro stick -> `vehicle_rates_setpoint` -> `mc_rate_control` | N/A | N/A | N/A | Acro/rate governance | Absolute attitude can exceed position/manual tilt limits by design of rate control. |

## Pre-Registered Candidate

Candidate uncovered grid cell:

- Limit: `MPC_TILTMAX_AIR = 45 deg`.
- Interface: OFFBOARD `SET_ATTITUDE_TARGET` with attitude quaternion active and body-rate fields ignored.
- Dynamic test: command roll setpoints `{60, 90, 120, 150}` deg at 20-50 Hz while keeping OFFBOARD active; parse ulog `vehicle_attitude`, `vehicle_attitude_setpoint`, `vehicle_local_position`, and `vehicle_status`.
- COVERED outcome: setpoint or achieved tilt is clipped/rejected at or below `MPC_TILTMAX_AIR`, or PX4 prevents the command path through commander/offboard validation/failsafe.
- REPLICATES outcome: setpoint is accepted, achieved tilt exceeds `MPC_TILTMAX_AIR` without clamp, a hard consequence occurs, and Tier-1 cleanliness holds.

This matrix is a Phase A result. Dynamic Phase B must not reinterpret offboard loss, invalid-setpoint failsafe, or commander rejection as clean evidence.

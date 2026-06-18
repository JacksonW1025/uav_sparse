# geofence_attitude Phase A Coverage Matrix

Pinned target: ArduCopter `Copter-4.4.1`, SHA `e010f97906087a3a1975e1c4fcc1f88a249599ce`. Static audit source tree: `/tmp/ardupilot-Copter-4.4.1`.

## Static Cell Record

| Field | Record |
|---|---|
| Scenario | `geofence_attitude`, phase A static, v1 |
| Mechanism + promise | Horizontal geofence promises that guided waypoints/destinations outside the fence are rejected before navigation uses them, and that actual fence breaches trigger `FENCE_ACTION`. |
| Interface + dispatch + supported? | MAVLink `SET_ATTITUDE_TARGET` is advertised as supported (`ArduCopter/GCS_Mavlink.cpp:1451-1459`). In guided mode it dispatches to `ModeGuided::set_angle` (`GCS_Mavlink.cpp:1122-1192`), then `angle_control_run` calls `AC_AttitudeControl::input_quaternion` or `input_rate_bf_roll_pitch_yaw` (`ArduCopter/mode_guided.cpp:963-967`). |
| Static coverage bit | **Clamp component = UNCOVERED. Reactive fallback = PRESENT, interface-independent, boundary-after-the-fact.** |
| Why UNCOVERED | `check_destination_within_fence` is called by guided destination setters when a position destination exists (`ArduCopter/mode_guided.cpp:329-339`, `421-430`, `563-574`). `set_angle` only stores quaternion/body rates and thrust/climb rate (`mode_guided.cpp:626-653`). The angle run path calls attitude control directly and has no XY fence admission or `AC_Avoid::adjust_velocity` (`mode_guided.cpp:911-977`). |
| If UNCOVERED, M witness sketch | Start inside an enabled horizontal fence in GUIDED. Stream legal `SET_ATTITUDE_TARGET` with a unit quaternion leaning outward, or set `attitude_ignore` and provide body rates, while keeping thrust valid. There is no position target field for destination admission. The vehicle can build horizontal velocity and cross the boundary before `AC_Fence::check()` reacts. |
| Expected Oracle-A consequence | Boundary breach or entry into a configured dangerous space/obstacle band outside the horizontal fence, attributable to the operator's supported attitude/rate input, not to environment-only drift. |
| By-design + firmware source | Destination rejection is by design for waypoint-like targets: `AC_Fence::check_destination_within_fence` checks a supplied `Location` against enabled alt/circle/polygon fences (`libraries/AC_Fence/AC_Fence.cpp:590-628`). Reactive breach is by design: `AC_Fence::check` checks actual fences and `Copter::fence_check` maps new breaches to RTL/Land/SmartRTL/Brake (`libraries/AC_Fence/AC_Fence.cpp:542-588`; `ArduCopter/fence.cpp:23-82`). |
| Risk communicated? | Fence params document `FENCE_ACTION`, `FENCE_RADIUS`, `FENCE_MARGIN`, and enabled types (`libraries/AC_Fence/AC_Fence.cpp:44-93`). In the audited source, no local API warning was found that `SET_ATTITUDE_TARGET` bypasses the destination admission check; the bypass follows from the command topology. |
| Static verdict | `UNCOVERED` for the command-entry clamp component on attitude/rate. Not a static clean-unsafe verdict because the reactive failsafe may still prevent a chosen hard consequence in dynamic runs. |
| E role | Dynamic E must include explicit boundary/obstacle geometry. Wind alone is insufficient: the consequence is crossing a horizontal fence or entering a danger band outside it. |

## Interface Matrix

| Interface / command family | Route in Copter 4.4.1 | Supported? | Destination admission `check_destination_within_fence` | XY stop-at-fence avoidance | Reactive `AC_Fence::check` | Coverage interpretation |
|---|---|---:|---:|---:|---:|---|
| Guided position target, local/global with position active | `GCS_Mavlink.cpp:1283-1291`, `1369-1390` -> `ModeGuided::set_destination*` | yes | COVERED when position destination exists | position controller path | yes | Position destination is rejected if outside fence. |
| Guided position+velocity target | `set_destination_posvelaccel` | yes | COVERED for the destination point (`mode_guided.cpp:563-574`) | position/velocity controller path | yes | Destination point checked; velocity component is not an independent fence admission. |
| Guided velocity-only / acceleration-only | `GCS_Mavlink.cpp:1286-1289`, `1385-1388` -> `set_velaccel` / `set_accel` | yes | N/A, no destination point | COVERED by `AC_Avoid::adjust_velocity` when enabled/default (`mode_guided.cpp:785-789`; `AC_Avoid.cpp:185-214`) | yes | Not the candidate cell; has velocity limiter path, but no destination admission. |
| Auto `NAV_GUIDED` waypoint | `ModeAuto::do_guided` calls `mode_guided.set_destination(dest)` (`ArduCopter/mode_auto.cpp:763-779`) | yes | COVERED through guided destination setter | position controller path | yes | Auto-guided waypoint inherits destination admission. |
| Rally point validity | `AP_Rally_Copter::is_valid` calls `check_destination_within_fence` (`ArduCopter/AP_Rally.cpp:22-29`) | yes | COVERED | N/A | yes | Rally point outside fence is invalid. |
| **Guided attitude quaternion** | `SET_ATTITUDE_TARGET` -> `set_angle` -> `input_quaternion` (`GCS_Mavlink.cpp:1122-1192`; `mode_guided.cpp:626-653`, `963-967`) | yes | **UNCOVERED** | No XY `adjust_velocity` in angle path | yes | Candidate gap: no command-entry horizontal fence clamp. |
| **Guided body-rate** | `SET_ATTITUDE_TARGET` with attitude ignored -> `set_angle` -> `input_rate_bf_roll_pitch_yaw` (`mode_guided.cpp:963-967`; `AC_AttitudeControl.cpp:421-455`) | yes | **UNCOVERED** | No XY `adjust_velocity` in angle path | yes | Same gap for body-rate variant. |

## Reactive Fallback Matrix

| Component | Static finding |
|---|---|
| Input | Circle fence uses actual AHRS home-relative NE position (`libraries/AC_Fence/AC_Fence.cpp:505-515`). Polygon fence uses current AHRS `Location` (`libraries/AC_Fence/AC_PolyFence_loader.cpp:207-214`). |
| Call point/frequency | `Copter::three_hz_loop` is scheduled at 3 Hz and calls `fence_check` (`ArduCopter/Copter.cpp:180`, `591-605`). `ArduCopter/fence.cpp:8` says 1 Hz, but scheduler is the operative static source. |
| Margin | `FENCE_MARGIN` default 2 m is configured (`libraries/AC_Fence/AC_Fence.cpp:87-93`) and used by `AC_Avoid` stop-at-fence velocity limiting (`libraries/AC_Avoidance/AC_Avoid.cpp:708-777`), but `check_fence_circle` breaches at `_home_distance >= _circle_radius` (`AC_Fence.cpp:511-523`). |
| Action | New breaches trigger configured `FENCE_ACTION`: default RTL-or-Land, plus Land, SmartRTL, Brake variants (`ArduCopter/fence.cpp:30-78`; `libraries/AC_Fence/AC_Fence.h:19-30`). |
| Momentum response | Brake and RTL use position/waypoint controllers to generate thrust vectors and reduce/redirect motion (`ArduCopter/mode_brake.cpp:10-69`; `ArduCopter/mode_rtl.cpp:127-185`). This is a real fallback but not an instantaneous pre-controller clamp. |
| Dynamic prediction | Because the fallback sees actual boundary breach after it occurs and then changes mode at finite rate, predict clean-unsafe hard witness if the dynamic Oracle-A danger space begins at or close outside the fence. If dynamic shows zero practical window before hard consequence, recode as C-ABSENT/covered by reaction. |

## Honest Boundaries

- This is an operator-reachable supported-input case, not an environment-spontaneous escape claim.
- Static Phase A does not decide clean-unsafe; it decides the clamp coverage bit and preregisters the dynamic prediction.
- No area law, scale law, or robustness area claim is made here.

# Direction-A Necessity Test: Channel-Agnostic ddmin

Scope: `px4_position`, seed 0, property `post_neutral_xy_velocity`.
Starts: 10 cached Arm B robust violations; clean yield: 4/10 (0.400).

## Decision

Headline: **channel-direction necessary / clearly better in this single-seed probe**

median final support stayed above 8 (14.5); clean yield was below majority (4/10); cost ratio was much larger than Arm C (22.106)

## Starting Points

| trigger | bucket | Arm B eval | stage | max|theta| | support | channels | rho mean | rho std | theta hash |
| ---: | --- | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0 | arm_b_interior | 97 | random_endpoint | 0.451955 | 28 | pitch,roll,throttle,yaw | -0.129791 | 0.024550 | 1ab5c0d0f90b9060 |
| 1 | arm_b_interior | 107 | scale_bisection | 0.494727 | 32 | pitch,roll,throttle,yaw | -0.455709 | 0.020747 | 4f9e4e8096aeac56 |
| 2 | arm_b_interior | 116 | scale_bisection | 0.426143 | 26 | pitch,roll,throttle,yaw | -0.078110 | 0.005913 | 9fd92bf79a81d85e |
| 3 | arm_b_interior | 119 | scale_bisection | 0.415489 | 25 | pitch,roll,throttle,yaw | -0.027395 | 0.006426 | 1875150e5456d715 |
| 4 | arm_b_interior | 147 | scale_bisection | 0.458877 | 31 | pitch,roll,throttle,yaw | -0.576164 | 0.009529 | b6f37638eecd435d |
| 5 | arm_b_interior | 150 | scale_bisection | 0.401517 | 27 | pitch,roll,throttle,yaw | -0.239881 | 0.024488 | f683a5684a3befee |
| 6 | arm_b_interior | 151 | scale_bisection | 0.372837 | 27 | pitch,roll,throttle,yaw | -0.099429 | 0.008710 | a640eec192e42a4d |
| 7 | arm_b_densest_moderate | 113 | random_endpoint | 0.681829 | 32 | pitch,roll,throttle,yaw | -1.519877 | 0.027778 | e42eeb10fbb8320d |
| 8 | arm_b_densest_moderate | 105 | random_endpoint | 0.659636 | 32 | pitch,roll,throttle,yaw | -1.232666 | 0.021362 | 86299434d1ec5381 |
| 9 | arm_b_densest_moderate | 89 | random_endpoint | 0.738535 | 29 | pitch,roll,throttle,yaw | -1.140124 | 0.064878 | 9ab8912bc8b9fd52 |

## Minimized Triggers

| trigger | clean | support | channels | max|theta| | rho mean | rho std | 2sigma margin | J=5 points | memo hits | budget | theta hash |
| ---: | --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 0 | False | 17 | pitch,roll,throttle,yaw | 0.451955 | -0.037384 | 0.016161 | -0.005063 | 40 | 7 | ok | 92a47d3c9d085bfb |
| 1 | True | 8 | pitch,roll | 0.474178 | -0.042352 | 0.019819 | -0.002714 | 40 | 3 | exhausted | 5da5a141db383307 |
| 2 | False | 17 | pitch,roll,throttle | 0.426143 | -0.061604 | 0.021024 | -0.019556 | 40 | 7 | ok | 7e2e9c13e4a03267 |
| 3 | False | 24 | pitch,roll,throttle,yaw | 0.382258 | -0.039666 | 0.007227 | -0.025211 | 40 | 7 | ok | 6f126656b1cf4f9a |
| 4 | False | 12 | pitch,roll | 0.455292 | -0.172168 | 0.078186 | -0.015796 | 40 | 1 | exhausted | 0d1bbfdd234aab75 |
| 5 | False | 27 | pitch,roll,throttle,yaw | 0.388970 | -0.157850 | 0.035935 | -0.085980 | 40 | 0 | exhausted | d28470e4a10b5fb6 |
| 6 | False | 17 | pitch,roll,throttle | 0.372837 | -0.086356 | 0.014189 | -0.057978 | 40 | 7 | ok | 9b2db2b1862a540d |
| 7 | True | 6 | pitch,roll | 0.521130 | -0.136462 | 0.015266 | -0.105930 | 38 | 3 | ok | 42c8211a023bf73f |
| 8 | True | 5 | pitch,roll | 0.637215 | -0.057951 | 0.012770 | -0.032411 | 40 | 1 | ok | 4d6376be0a5f63ec |
| 9 | True | 4 | roll | 0.675067 | -0.116065 | 0.042868 | -0.030329 | 35 | 1 | ok | eed06199809c4868 |

## Distributions

- ddmin final support: min=4.000, p25=6.500, median=14.500, p75=17.000, max=27.000
- Arm C interior support: min=4.000, p25=6.000, median=6.000, p75=8.000, max=8.000
- ddmin final max|theta|: min=0.373, p25=0.398, median=0.454, p75=0.509, max=0.675
- Arm C interior max|theta|: min=0.406, p25=0.453, median=0.477, p75=0.500, max=0.500
- ddmin final channels: {'pitch,roll': 4, 'pitch,roll,throttle': 2, 'pitch,roll,throttle,yaw': 3, 'roll': 1}
- Arm C interior channels: {'pitch': 5, 'pitch,roll': 8, 'roll': 5}

## Cost

- ddmin J=5 points used: 393
- ddmin J=5 points per clean trigger: 98.250
- Arm C amortized J=5 points per interior violation: 4.444 (80/18)
- cost ratio ddmin/Arm C: 22.106

## Robustness

Every accepted ddmin step and every reported minimized trigger satisfies the fixed J=5 two-sigma robust-violation gate.

## Artifacts

- pre_registration: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/pre_registration.json`
- starting_points: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/starting_points.csv`
- minimized_triggers: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/minimized_triggers.csv`
- ddmin_point_evaluations: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/ddmin_point_evaluations.csv`
- ddmin_query_repeats: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/ddmin_query_repeats.csv`
- ddmin_decisions: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/ddmin_decisions.csv`
- summary: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/direction_a_ddmin_summary.json`
- report: `runs/direction_a_ddmin_px4_position_seed0_v1/reports/direction_a_ddmin_report.md`

Successful PX4 queries: 1965.
Timeout retries: 12.
Query attempts including timeout retries: 1977.
Elapsed wall time: 38542.6s.

Single seed/scenario probe only; replicate across seeds before any paper claim.

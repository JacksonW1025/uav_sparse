# Control-Authority Tier B Re-anchor + Tier C Verification (v2)

**VERDICT: B=RE-ANCHORED; C=C-ABSENT**

On the documented-supported GUIDED quaternion interface, a commanded attitude target > ANGLE_MAX is accepted unclamped (demanded attitude exceeds ANGLE_MAX), produces a real hard consequence, and the flight controller raises no preventive failsafe (interface-aware Tier-1 = 0). The contract-clean unsafe zone re-anchors on solid ground. Part C ABSENT: no run with ATT.DesRoll <= ANGLE_MAX under the legal wind/turbulence upper bound produced an unrecovered hard consequence with B = 0 in SITL; the demonstration rests on Tier B.

## Interface (the re-anchor)

- Part B/C ride the **supported** GUIDED `SET_ATTITUDE_TARGET` path: `type_mask=0x07` (ignore all three body-rate fields) + a **unit attitude quaternion**. A non-zero quaternion routes (`mode_guided.cpp:964-967`) to `AC_AttitudeControl::input_quaternion` (`AC_AttitudeControl.cpp:231-266`), which applies only `ang_vel_limit` -- **no ANGLE_MAX clamp**. This is exactly the mask ArduPilot's GUIDED dev docs prescribe; the predecessor oracleA_v1 used the doc-discouraged body-rate fields.
- Part C is split by vertical authority. `C_attitude` uses `GUID_OPTIONS=0`, so `thrust=0.5` is zero climb-rate and the z-controller holds altitude from a 90 m start. `C_altitude` uses `GUID_OPTIONS=8` (direct throttle) and is reported as the suspect secondary group.
- Interface-aware **B**: command attitude > ANGLE_MAX is **not** a contract clamp on this interface (by design, angle_max_scope_v1) -- scored under Tier-2. `CRASH_FAILSAFE`/`CRASH_CHECK` are post-impact consequence detectors, excluded from preventive B. So B = a genuine preventive failsafe before the hard consequence.

## S1 interface fidelity

| probe | group | commanded | DesRoll peak | demanded lean peak | achieved peak | DesRoll valid? | outcome |
| --- | --- | ---: | ---: | ---: | ---: | :--: | --- |
| tierBCv2_fid_fidB_quat_roll90_g090_w00_t00_a4500_m100_s00 | B | 90 | 89.99 | 89.99 | 90.07 | False | altitude_loss |
| tierBCv2_fid_fidC_quat_roll44_g044_w00_t00_a4500_m100_s00 | C_attitude | 44 | 43.99 | 43.99 | 44.02 | True | safe |

- S1 gate: `PASS`.
- ACRO_TRAINER=0 corroboration: ran (achieved roll peak 174.87 deg, exceeds ANGLE_MAX = True)
- Legal-wind upper-bound check (0 deg level hover at 15 m/s wind / 8 m/s turb): outcome `safe` -> within envelope (used as high pressure).

## Part B re-anchor (command > ANGLE_MAX, supported interface)

Verdict **B = RE-ANCHORED**. Robust clean cells (deg): `[90.0, 120.0, 150.0]`.

| cmd target (deg) | n | reanchor_clean | DesRoll>ANGLE_MAX | DesRoll crossed before hard A | Tier-1 clean | blocked_by_FS | no_consequence | max demanded peak (deg) | outcomes |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 60 | 3 | 0 | 3 | 3 | 3 | 0 | 3 | 59.99 | `{"safe": 3}` |
| 90 | 3 | 3 | 3 | 3 | 3 | 0 | 0 | 89.99 | `{"altitude_loss": 3}` |
| 120 | 3 | 3 | 3 | 3 | 3 | 0 | 0 | 119.99 | `{"altitude_loss": 3}` |
| 150 | 3 | 3 | 3 | 3 | 3 | 0 | 0 | 149.99 | `{"altitude_loss": 3}` |

## Part C verification (ATT.DesRoll <= ANGLE_MAX throughout + legal wind/turbulence)

Verdict **C = C-ABSENT**. DesRoll-over-limit anywhere: `False` (must be false for valid C cells). C-attitude cells: `[]`; C-altitude cells: `[]`.

| group | cmd (deg) | pressure | wind | turb | n | C-attitude | C-altitude | recovered | safe | blocked_FS | invalid | DesRoll<=AMAX | max DesRoll | max att err | max alt loss |
| --- | ---: | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | :--: | ---: | ---: | ---: |
| C_altitude | 35 | alt_nohold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 1.85 | 0.00 |
| C_altitude | 35 | alt_nohold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 2.66 | 0.00 |
| C_altitude | 35 | alt_nohold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 1.58 | 0.00 |
| C_altitude | 40 | alt_nohold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 2.20 | 0.00 |
| C_altitude | 40 | alt_nohold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 2.69 | 0.00 |
| C_altitude | 40 | alt_nohold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 1.85 | 0.00 |
| C_altitude | 44 | alt_nohold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 2.04 | 0.00 |
| C_altitude | 44 | alt_nohold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 2.65 | 0.00 |
| C_altitude | 44 | alt_nohold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 2.24 | 0.00 |
| C_attitude | 35 | att_hold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 1.49 | 0.00 |
| C_attitude | 35 | att_hold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 2.77 | 0.00 |
| C_attitude | 35 | att_hold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 34.99 | 1.74 | 0.00 |
| C_attitude | 40 | att_hold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 1.81 | 0.00 |
| C_attitude | 40 | att_hold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 2.58 | 0.00 |
| C_attitude | 40 | att_hold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 39.99 | 2.07 | 0.00 |
| C_attitude | 44 | att_hold_calm | 0 | 0 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 2.31 | 0.00 |
| C_attitude | 44 | att_hold_high | 15 | 8 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 3.27 | 0.00 |
| C_attitude | 44 | att_hold_low | 8 | 4 | 3 | 0 | 0 | 0 | 3 | 0 | 0 | True | 43.99 | 2.08 | 0.00 |

## Interface-aware contract cleanliness (Tier-1 / Tier-2)

- Anti-leakage audit: Tier-1 input fields disjoint from oracle-A consequence set -> leakage_detected = **False** (T1a is N/A on the attitude interface and excluded).
- Part B: Tier-1 (contract) hits = 0/12; Tier-2 (result) hits = 9/12.
- Part C: Tier-1 (contract) hits = 0/54; Tier-2 (result) hits = 0/54.
- Tier-1 policies scored on the attitude interface: T1b (no preventive failsafe), T1c (configured-limit compliance), T1d (mode-transition legitimacy). T1a (ANGLE_MAX command clamp) is reported **not-applicable**.

## Artifacts

- Preregistration: `planc/results/tierBC_v2_prereg.json`
- Result JSON: `planc/results/tierBC_v2_result.json`
- Plot partB_demanded_vs_anglemax: `planc/analysis/tierBC_v2_partB_demanded_vs_anglemax.png`
- Plot partB_trajectory: `planc/analysis/tierBC_v2_partB_trajectory.png`
- Plot partC_outcome_vs_wind: `planc/analysis/tierBC_v2_partC_outcome_vs_wind.png`
- Plot partC_altitude_loss: `planc/analysis/tierBC_v2_partC_altitude_loss.png`
- Plot partC_trajectory: `planc/analysis/tierBC_v2_partC_trajectory.png`
- Plot tier_decomposition: `planc/analysis/tierBC_v2_tier_decomposition.png`

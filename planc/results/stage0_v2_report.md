# Stage-0 v2 Control Authority Probe

VERDICT: **GO**

Robust OverDraw exists: at least one P3 cell is A inside, B outside in >=2 seeds.

## Preregistered Criteria

- Oracle A: attitude tracking error > 60.0 deg for 0.5 s, or crash/ground-hit before cleanup.
- Oracle B: preventive failsafe/mode intervention, or command angle touching ANGLE_MAX.
- Command path: SET_ATTITUDE_TARGET in GUIDED_NOGPS after GUIDED takeoff; r is bounded-attitude doublet ramp rate in deg/s.
- Seed group: [0, 1, 2] (ArduCopter 4.4.1 exposes SIM_WIND_TURB but no writable turbulence seed parameter; seed labels are fixed repetition IDs.)

## P1 Path Fidelity

Conclusion: **True** - direct attitude/rate path met the 90% fidelity preregistration.

| run | RATE desired/cmd | RATE actual/cmd | ATT target/cmd diagnostic | error |
| --- | ---: | ---: | ---: | --- |
| stage0v2_fidelitymid_fidelity_r0180_w00_t00_a4500_m100_s00 | 1.081 | 0.998 | 3.487 |  |
| stage0v2_fidelityhigh_fidelity_r0300_w00_t00_a4500_m100_s00 | 1.011 | 1.006 | 1.612 |  |

## P2 Pressure

Completed: 15 runs; A count: 14; authority-limited count: 15; max error: 223.72 deg.
First A: `stage0v2_p2_default_r0120_w00_t00_a4500_m100_s01` at `{"angle_max_cd": 4500.0, "layer": "default", "model": "m100", "r_deg_s": 120.0, "role": "p2", "seed": 1, "turbulence_m_s": 0.0, "wind_m_s": 0.0}`.

## P3 Boundary Separation

Counts: `{"blocked": 5, "bug_side": 25, "overdraw": 8, "safe": 1}`.
Stable overdraw cells: `[{"A": 3, "bug_side": 1, "layer": "default", "overdraw": 2, "r_deg_s": 480.0, "total": 3}]`.
OverDraw scale: 8 runs; 1 stable cells.
Boundary interpretation: The first A and B entry thresholds can coincide while B is non-monotone in r; the observed separation is an OverDraw island, not a simple first-threshold offset.
Stable bug-side cells: `[{"A": 3, "bug_side": 3, "layer": "default", "overdraw": 0, "r_deg_s": 240.0, "total": 3}, {"A": 3, "bug_side": 2, "layer": "default", "overdraw": 1, "r_deg_s": 360.0, "total": 3}, {"A": 3, "bug_side": 2, "layer": "default", "overdraw": 1, "r_deg_s": 600.0, "total": 3}, {"A": 3, "bug_side": 3, "layer": "moderate", "overdraw": 0, "r_deg_s": 240.0, "total": 3}, {"A": 3, "bug_side": 3, "layer": "moderate", "overdraw": 0, "r_deg_s": 480.0, "total": 3}, {"A": 2, "bug_side": 2, "layer": "moderate", "overdraw": 0, "r_deg_s": 600.0, "total": 2}, {"A": 3, "bug_side": 3, "layer": "high", "overdraw": 0, "r_deg_s": 240.0, "total": 3}, {"A": 2, "bug_side": 2, "layer": "high", "overdraw": 0, "r_deg_s": 600.0, "total": 2}]`.

| layer | A boundary r | B boundary r | separation | overdraw r values |
| --- | ---: | ---: | ---: | --- |
| default | 240.0 | 240.0 | 0.0 | [360.0, 480.0, 600.0] |
| high | 240.0 | 240.0 | 0.0 | [360.0, 480.0] |
| moderate | 240.0 | 240.0 | 0.0 | [360.0] |
| tight_direction_check | 360.0 | 360.0 | 0.0 | [360.0] |

## P4 Monotonicity Pre-read

Evaluated: **True** - authority signals entered the limited region.

| layer | throttle near-limit monotone | AOutSlew monotone | rate-out monotone |
| --- | --- | --- | --- |
| default | False | True | False |
| high | False | False | False |
| moderate | False | True | False |
| tight_direction_check | None | None | None |

## Evidence Artifacts

- Preregistration: `planc/results/stage0_v2_prereg.json`
- Result JSON: `planc/results/stage0_v2_result.json`
- Partial runs: `planc/results/stage0_v2_partial.json`
- Plot boundary_A: `planc/analysis/stage0_v2_boundary_A.png`
- Plot boundary_separation: `planc/analysis/stage0_v2_boundary_separation.png`
- Plot authority_evidence: `planc/analysis/stage0_v2_authority_evidence.png`
- Parsed logs: `planc/logs/stage0v2_*_parsed.csv` and `*_parsed.oracle.json`
- Raw DataFlash: `planc/logs/stage0v2_*.BIN` (local workspace evidence, ignored by Git)

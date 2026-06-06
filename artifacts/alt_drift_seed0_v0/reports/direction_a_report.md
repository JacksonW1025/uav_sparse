# Direction-A Discriminating Probe

Scope: `px4_position`, seed 0, property `post_neutral_alt_drift`.
Matched budget: N=80 J=5 points per arm, 1200 successful PX4 queries total.

## Arm Outcomes

| arm | J=5 points | robust violations | interior | moderate | saturated | safe | noise band | 2sigma gate rejects | gentlest max|theta| | support | active channels |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A | 80 | 1 | 0 | 0 | 1 | 79 | 0 | 0 | 0.995 | 32 | pitch,roll,throttle,yaw |
| B | 80 | 0 | 0 | 0 | 0 | 78 | 2 | 1 |  |  |  |
| C | 80 | 10 | 0 | 6 | 4 | 59 | 11 | 6 | 0.812 | 8 | throttle |

## Robust-Violation Amplitude Percentiles

- Arm A: min=0.995, p25=0.995, median=0.995, p75=0.995, p90=0.995, p95=0.995, max=0.995.
- Arm B: no robust violations.
- Arm C: min=0.812, p25=0.828, median=0.875, p75=1.000, p90=1.000, p95=1.000, max=1.000.

## Gentlest Robust Violation

- arm: `C`
- theta hash: `ab680a7d086a96ea`
- theta path: `artifacts/alt_drift_seed0_v0/thetas/C_00239_ab680a7d086a96ea.npy`
- max|theta|: 0.812500 (moderate)
- support size |theta|>0.1: 8
- active channels: `throttle`
- cross-property rho means: post_neutral_xy_velocity=0.954036, post_neutral_xy_drift=1.957330, post_neutral_alt_drift=-0.067692

Theta (D=40 group order):

```json
[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.75, 0.0, 0.0, 0.0, 0.8125, 0.0, 0.0, 0.0, 0.8125, 0.0, 0.0, 0.0, 0.8125, 0.0, 0.0, 0.0, 0.8125, 0.0, 0.0, 0.0, 0.8125]
```

## Interior Violations

No interior robust violations were found.

## Decision Inputs

- Arm A interior robust violations: 0
- Arm A robust violations total: 1
- Interior-targeting value condition: `False`
- Channel-reduction gentler than Arm B: `False`
- Channel-reduction cleaner than Arm B: `False`
- Strict no-interior-Arm-A confirmation flag: `False`

The qualitative terms in the decision rule (`few`, `readily`, `clearly`) are left as exact counts and distributions here; no thresholds were tuned after seeing data.

## Artifacts

- pre_registration: `artifacts/alt_drift_seed0_v0/reports/pre_registration.json`
- point_evaluations: `artifacts/alt_drift_seed0_v0/reports/point_evaluations.csv`
- query_repeats: `artifacts/alt_drift_seed0_v0/reports/query_repeats.csv`
- arm_metrics: `artifacts/alt_drift_seed0_v0/reports/arm_metrics.csv`
- robust_violations: `artifacts/alt_drift_seed0_v0/reports/robust_violations.csv`
- interior_violations: `artifacts/alt_drift_seed0_v0/reports/interior_violations.csv`
- summary: `artifacts/alt_drift_seed0_v0/reports/direction_a_summary.json`
- report: `artifacts/alt_drift_seed0_v0/reports/direction_a_report.md`
- groups: `artifacts/alt_drift_seed0_v0/groups.csv`

Successful PX4 queries: 1200.
Timeout retries: 8.
Query attempts including timeout retries: 1208.
Elapsed wall time: 23865.3s.

Single seed/scenario probe only; replicate across seeds before any paper claim.

Stop point: three arms plus classifier only.

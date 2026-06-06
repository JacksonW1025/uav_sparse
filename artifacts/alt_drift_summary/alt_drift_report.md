# alt_drift Phase 1 report

Context: confirmatory PX4 alt_drift, scenario px4_position.

## Phase 0

Decision: confirm. Saturated positive throttle is feasible and violates alt_drift under the frozen 2-sigma gate.

## H-alt-1

Decision: confirm. Top channel: throttle. Throttle share: 0.910364. A_alt: throttle.

| channel | weight | share |
| --- | --- | --- |
| throttle | 8.26076 | 0.910364 |
| yaw | 0.33403 | 0.0368112 |
| pitch | 0.246296 | 0.0271427 |
| roll | 0.233038 | 0.0256816 |

## H-alt-2

Decision: falsify. Thresholds: Arm A interior=0; Arm B channel pure <=0.10; Arm C interior>0 and channel pure >=0.90.

| seed | arm | point_count | robust_violation_count | interior_robust_violation_count | channel_pure_interior_count | channel_pure_ratio | channel_pure_denominator | A_alt |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | A | 80 | 1 | 0 | 0 | 0 | 0 | throttle |
| 0 | B | 80 | 0 | 0 | 0 | 0 | 0 | throttle |
| 0 | C | 80 | 10 | 0 | 0 | 0 | 0 | throttle |

Runner arm metrics:

| arm | j5_point_count | robust_violation_count | interior_robust_violation_count | noise_band_count | negative_mean_rejected_by_2sigma_gate_count |
| --- | --- | --- | --- | --- | --- |
| A | 80 | 1 | 0 | 0 | 0 |
| B | 80 | 0 | 0 | 2 | 1 |
| C | 80 | 10 | 0 | 11 | 6 |

## Go/no-go

Phase 1 go/no-go: no_go_phase2. Phase 2 was not run.

## Artifacts

- Seed run dir: `artifacts/alt_drift_seed0_v0`
- Summary dir: `artifacts/alt_drift_summary`
- CSV: `artifacts/alt_drift_summary/channel_mass.csv`, `artifacts/alt_drift_summary/arm_purity.csv`, `artifacts/alt_drift_summary/signatures_seed0.csv`

Caveat: this is confirmatory PX4 alt_drift only; cross-platform claims require ArduPilot.

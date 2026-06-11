# Residual-rate migration report

Context: confirmatory, PX4, scenario `px4_position`. Phase 2 and ddmin were not run.

## One-line judgments

- confirmatory PX4 post_neutral_climb_rate: no Tier 1 saturated throttle violation; stop.
- confirmatory PX4 post_neutral_yaw_rate: no Tier 1 saturated yaw violation; stop.
- confirmatory PX4 overall: neither throttle nor yaw residual-rate migration is confirmed at Tier 1; multi-channel method-paper premise is not established by this test.

## Phase 0

| property | Tier 1 saturated violable | Tier 2 saturated violable | best label |
| --- | --- | --- | --- |
| `post_neutral_climb_rate` | False | False | `` |
| `post_neutral_yaw_rate` | False | False | `` |

## H-1 Channel Mass

| property | seed | channel | weight | share | status |
| --- | --- | --- | --- | --- | --- |
| post_neutral_climb_rate | 0 | throttle |  |  | not_run_phase0_stop |
| post_neutral_yaw_rate | 0 | yaw |  |  | not_run_phase0_stop |

## H-2 Tier 1

| property | tier | seed | arm | point_count | robust_violation_count | interior_robust_violation_count | channel_pure_interior_count | channel_pure_ratio | channel_pure_denominator | A_phi | status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| post_neutral_climb_rate | Tier 1 | 0 | A | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_climb_rate | Tier 1 | 0 | B | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_climb_rate | Tier 1 | 0 | C | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 1 | 0 | A | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 1 | 0 | B | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 1 | 0 | C | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |

## H-2 Tier 2

| property | tier | seed | arm | point_count | robust_violation_count | interior_robust_violation_count | channel_pure_interior_count | channel_pure_ratio | channel_pure_denominator | A_phi | status |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| post_neutral_climb_rate | Tier 2 | 0 | A | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_climb_rate | Tier 2 | 0 | B | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_climb_rate | Tier 2 | 0 | C | 0 | 0 | 0 | 0 | 0.000000 | 0 | throttle | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 2 | 0 | A | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 2 | 0 | B | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |
| post_neutral_yaw_rate | Tier 2 | 0 | C | 0 | 0 | 0 | 0 | 0.000000 | 0 | yaw | not_run_phase0_stop |

## Go/no-go

| property | H-1 | H-2 Tier 1 | H-2 Tier 2 | Phase 2 |
| --- | --- | --- | --- | --- |
| `post_neutral_climb_rate` | not_run_phase0_stop | not_run_phase0_stop | not_run_phase0_stop | no_go_phase2 |
| `post_neutral_yaw_rate` | not_run_phase0_stop | not_run_phase0_stop | not_run_phase0_stop | no_go_phase2 |

## Data and parameter gaps

- Phase 2 seed 1/2 and ddmin baseline were not run by instruction.
- Cross-platform claims require ArduPilot.
- Thresholds are bound to local PX4 SITL defaults recorded in `artifacts/residual_rate_prereg.md`.

## Artifacts

- Summary dir: `artifacts/residual_rate_summary`
- CSV: `artifacts/residual_rate_summary/channel_mass.csv`, `artifacts/residual_rate_summary/arm_purity_tier1.csv`, `artifacts/residual_rate_summary/arm_purity_tier2.csv`, `artifacts/residual_rate_summary/distinct_costs.csv`, `artifacts/residual_rate_summary/signatures.csv`

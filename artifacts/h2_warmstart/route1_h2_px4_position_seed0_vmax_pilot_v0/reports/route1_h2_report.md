# Route-1 H2 Cross-Condition Warm-Start Pilot

Scope: `px4_position`, seed 0, property `post_neutral_xy_velocity`.
Cross-condition axis: `post_neutral_xy_velocity.v_max_mps` = [1.0, 0.9, 0.8].

Warm-start uses the empirically measured property-conditioned active channels: roll, pitch.

## Campaign Query Ratios

| cold baseline | cold c1 | warm c2+c3 | warm campaign | cold all 3 | ratio | speedup |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| structured | 122 | 100 | 222 | 366 | 0.607 | 1.65 |
| uniform | 52 | 100 | 152 | 169 | 0.899 | 1.11 |
| descent | 51 | 100 | 151 | 153 | 0.987 | 1.01 |

## Warm Boundaries

| condition | v_max | status | rho mean | rho std | queries | theta hash |
| ---: | ---: | --- | ---: | ---: | ---: | --- |
| 1 | 1.000 | reused_pointV | -0.075493 | 0.123369 | 5 | 3248eac8d31a9542 |
| 2 | 0.900 | complete | 0.001311 | 0.035376 | 50 | 6e9dced3879a94c4 |
| 3 | 0.800 | complete | 0.004531 | 0.013798 | 50 | b3b2b82e2a4b1ef6 |

## Boundary Displacement

| from v_max | to v_max | L2 | Linf | relative L2 |
| ---: | ---: | ---: | ---: | ---: |
| 1.000 | 0.900 | 0.207280 | 0.049870 | 0.124 |
| 0.900 | 0.800 | 0.072168 | 0.017363 | 0.048 |

## Cold Boundaries

| method | condition | v_max | status | rho mean | rho std | queries | theta hash |
| --- | ---: | ---: | --- | ---: | ---: | ---: | --- |
| structured | 1 | 1.000 | complete | -0.001615 | 0.021254 | 122 | cdaf417ec7a31b72 |
| structured | 2 | 0.900 | complete | -0.002230 | 0.019879 | 122 | 81374d981167d125 |
| structured | 3 | 0.800 | complete | -0.000083 | 0.013130 | 122 | 213b1b3f6f2a96d6 |
| uniform | 1 | 1.000 | complete | 0.005430 | 0.008919 | 52 | 99ffc5f90c3f0ef6 |
| uniform | 2 | 0.900 | complete | 0.006642 | 0.021391 | 65 | 17a3a20a33546fba |
| uniform | 3 | 0.800 | complete | -0.008138 | 0.025544 | 52 | 816887fe2d94bbc3 |
| descent | 1 | 1.000 | complete | 0.011456 | 0.043540 | 51 | d7b7ac4b7c126be5 |
| descent | 2 | 0.900 | complete | -0.003022 | 0.058397 | 51 | ba90d44d309250db |
| descent | 3 | 0.800 | complete | -0.003173 | 0.006574 | 51 | 011b6bf7b139db37 |

## Active-Channel Stability

| condition | v_max | top2 channels | top80 channels | channel shares | roll+pitch top2 | queries |
| ---: | ---: | --- | --- | --- | --- | ---: |
| 1 | 1.000 | pitch,roll | pitch,roll | pitch:0.480,roll:0.440,yaw:0.053,throttle:0.028 | True | 400 |
| 3 | 0.800 | pitch,roll | pitch,roll | pitch:0.467,roll:0.463,yaw:0.036,throttle:0.034 | True | 400 |

## Decision Inputs

- speedup >= 2x for all cold baselines: `False`
- minimum cold/warm speedup: `1.01`
- roll+pitch top2 at measured conditions: `True`

Caveat: v_max tightening moves the boundary largely along the input-scale direction; a follow-on initial-state axis is needed before making a stronger method claim.

## Run Notes

- final completed process wall time: 24610.6s (6.84h)
- summed per-query wall time from `query_evaluations.csv`: 31525.9s (8.76h)
- one earlier interrupted pass was stopped after finding a uniform-baseline J=5 bracket handling issue; the fixed run reused cached completed evaluations, so the final process wall time is lower than total calendar time
- uniform condition 2 had one boundary-noise bracket failure: a single-query safe endpoint (`rho=0.028799`) became unsafe under J=5 (`rho_mean=-0.017231`); the runner continued sampling and localized a valid J=5 straddle
- observed several recovered PX4 POSCTL mode-switch retries (`last mode=LOITER`); all recovered under retry and the campaign completed
- Point V was reused as the disclosed v_max=1.0 boundary anchor, but its current J=5 mean was noisy (`rho_mean=-0.075493`, `rho_std=0.123369`)

## Artifacts

- report: `runs/route1_h2_px4_position_seed0_vmax_pilot_v0/reports/route1_h2_report.md`
- summary: `runs/route1_h2_px4_position_seed0_vmax_pilot_v0/reports/route1_h2_summary.json`
- query_evaluations: `runs/route1_h2_px4_position_seed0_vmax_pilot_v0/reports/query_evaluations.csv`
- boundaries: `runs/route1_h2_px4_position_seed0_vmax_pilot_v0/reports/boundary_results.csv`

Total counted queries: 1593.
Elapsed wall time: 24610.6s.

Stop point: 3-condition pilot only.

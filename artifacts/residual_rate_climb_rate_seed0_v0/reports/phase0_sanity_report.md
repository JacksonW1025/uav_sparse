# Residual-rate Phase 0

Scope: confirmatory, PX4, `post_neutral_climb_rate`, `px4_position`.

Tier 1 saturated predicted-channel violable: `False`.
Tier 2 saturated predicted-channel violable: `False`.
Best Tier 1 label: ``.

| label | sign | max abs theta | channels | Tier 1 | Tier 2 | rho mean | rho std | terminal peak mean |
| --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: |
| zero_anchor | 0 | 0.000 |  | robust_safe | not_tier1_violation | 0.291679 | 0.002109 | 0.008321 |
| throttle_pos_full | 1 | 1.000 | throttle | robust_safe | not_tier1_violation | 0.273848 | 0.004652 | 0.026152 |
| throttle_neg_full | -1 | 1.000 | throttle | robust_safe | not_tier1_violation | 0.287377 | 0.004418 | 0.012623 |

Artifacts:

- pre_registration_copy: `artifacts/residual_rate_climb_rate_seed0_v0/reports/residual_rate_prereg.md`
- phase0_sanity: `artifacts/residual_rate_climb_rate_seed0_v0/reports/phase0_sanity.csv`
- phase0_query_repeats: `artifacts/residual_rate_climb_rate_seed0_v0/reports/phase0_query_repeats.csv`
- summary: `artifacts/residual_rate_climb_rate_seed0_v0/reports/phase0_sanity_summary.json`
- report: `artifacts/residual_rate_climb_rate_seed0_v0/reports/phase0_sanity_report.md`

# Residual-rate Phase 0

Scope: confirmatory, PX4, `post_neutral_yaw_rate`, `px4_position`.

Tier 1 saturated predicted-channel violable: `False`.
Tier 2 saturated predicted-channel violable: `False`.
Best Tier 1 label: ``.

| label | sign | max abs theta | channels | Tier 1 | Tier 2 | rho mean | rho std | terminal peak mean |
| --- | ---: | ---: | --- | --- | --- | ---: | ---: | ---: |
| zero_anchor | 0 | 0.000 |  | robust_safe | not_tier1_violation | 0.246883 | 0.001344 | 0.014917 |
| yaw_pos_full | 1 | 1.000 | yaw | robust_safe | not_tier1_violation | 0.246543 | 0.003350 | 0.015256 |
| yaw_neg_full | -1 | 1.000 | yaw | robust_safe | not_tier1_violation | 0.244695 | 0.002447 | 0.017104 |

Artifacts:

- pre_registration_copy: `artifacts/residual_rate_yaw_rate_seed0_v0/reports/residual_rate_prereg.md`
- phase0_sanity: `artifacts/residual_rate_yaw_rate_seed0_v0/reports/phase0_sanity.csv`
- phase0_query_repeats: `artifacts/residual_rate_yaw_rate_seed0_v0/reports/phase0_query_repeats.csv`
- summary: `artifacts/residual_rate_yaw_rate_seed0_v0/reports/phase0_sanity_summary.json`
- report: `artifacts/residual_rate_yaw_rate_seed0_v0/reports/phase0_sanity_report.md`

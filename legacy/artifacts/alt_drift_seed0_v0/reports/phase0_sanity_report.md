# alt_drift Phase 0 Sanity

Scope: confirmatory, PX4, alt_drift, px4_position.

Feasible saturated-throttle violation: **True**.
Best violation label: `throttle_pos_full`.
Best 2sigma margin: `-1.274073`.

| label | class | max|theta| | channels | rho mean | rho std | 2sigma margin |
| --- | --- | ---: | --- | ---: | ---: | ---: |
| zero_anchor | robust_safe | 0.000 |  | 0.990106 | 0.003018 | 0.996143 |
| throttle_pos_full | robust_violation | 1.000 | throttle | -1.362963 | 0.044445 | -1.274073 |
| throttle_neg_full | robust_safe | 1.000 | throttle | 0.721964 | 0.006547 | 0.735058 |

Decision: stop after Phase 0 until the user authorizes Phase 1.

No `*.ulg` files are retained in this artifact directory.

# Cross-coupling Residual-rate Step A

Scope: `exploratory-hypothesis`, PX4, `post_neutral_xy_velocity`, Step A structural matrix.

Protocol provenance: `artifacts/cross_coupling_prereg.md` was absent at task start; a local copy was generated from the operator protocol before collecting cross-coupling data. This is a data gap for `confirmatory-protocol` labeling. A generated horizontal natural-pair label was also corrected after an aborted initial runner start; measured G values are unchanged by that label correction, but all conclusions here are labeled `exploratory-hypothesis`.

Step A go/no-go: **go_step_b_candidates_present**.
H-RGA-1: `True`.
Cross-coupling candidates: `2`.
Interaction candidates: `2`.
RGA inverse method: `exact_inverse`; condition number: `98.6525`.

G matrix, signed terminal residual sensitivity:

| output \ input | roll | pitch | yaw | throttle |
| --- | ---: | ---: | ---: | ---: |
| vx | 0.098508 | -0.010353 | -0.022990 | 0.040790 |
| vy | 0.034694 | 0.123608 | 0.009710 | 0.026441 |
| vz | 0.009877 | 0.001907 | 0.018241 | 0.007958 |
| yaw_rate | 0.000912 | -0.002995 | 0.000015 | -0.001337 |

G row-normalized absolute mass share:

| output \ input | roll | pitch | yaw | throttle |
| --- | ---: | ---: | ---: | ---: |
| vx | 0.570594 | 0.059970 | 0.133164 | 0.236271 |
| vy | 0.178419 | 0.635668 | 0.049935 | 0.135977 |
| vz | 0.260041 | 0.050200 | 0.480244 | 0.209514 |
| yaw_rate | 0.173372 | 0.569617 | 0.002842 | 0.254170 |

RGA analogue Lambda:

| output \ input | roll | pitch | yaw | throttle |
| --- | ---: | ---: | ---: | ---: |
| vx | 0.450083 | 0.028738 | 0.143155 | 0.378023 |
| vy | 0.263342 | 1.122232 | 0.015063 | -0.400637 |
| vz | 0.014554 | -0.015956 | 0.840066 | 0.161337 |
| yaw_rate | 0.272021 | -0.135014 | 0.001716 | 0.861277 |

Candidates:

| id | type | input set | output | criterion | share | lambda/max | triggered entries |
| --- | --- | --- | --- | --- | ---: | ---: | --- |
| C000 | cross_coupling | throttle | vx | row_abs_share>=0.20;abs(lambda)>=0.30 | 0.236271 | 0.378023 | vx:throttle=0.378023 |
| C001 | cross_coupling | throttle | vy | abs(lambda)>=0.30 | 0.135977 | -0.400637 | vy:throttle=-0.400637 |
| C002 | interaction | pitch+yaw |  | any relevant lambda abs>=1.5 or <=0 |  | 1.122232 | yaw_rate:pitch=-0.135014 |
| C003 | interaction | pitch+throttle |  | any relevant lambda abs>=1.5 or <=0 |  | 1.122232 | vy:throttle=-0.400637;vz:pitch=-0.0159559 |

Step B status: not run in this Step A go/no-go pass.

Artifacts:

- pre_registration_copy: `artifacts/cross_coupling_summary/cross_coupling_prereg.md`
- groups: `artifacts/cross_coupling_summary/groups.csv`
- point_evaluations: `artifacts/cross_coupling_summary/stepA_point_evaluations.csv`
- query_repeats: `artifacts/cross_coupling_summary/stepA_query_repeats.csv`
- interaction_matrix: `artifacts/cross_coupling_summary/interaction_matrix.csv`
- rga: `artifacts/cross_coupling_summary/rga.csv`
- candidates: `artifacts/cross_coupling_summary/candidates.csv`
- arm_purity_tier1: `artifacts/cross_coupling_summary/arm_purity_tier1.csv`
- arm_purity_tier2: `artifacts/cross_coupling_summary/arm_purity_tier2.csv`
- signatures: `artifacts/cross_coupling_summary/signatures.csv`
- summary: `artifacts/cross_coupling_summary/cross_coupling_summary.json`
- report: `artifacts/cross_coupling_summary/cross_coupling_report.md`

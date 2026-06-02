# Direction-A Discriminating Probe

Scope: `px4_position`, seed 0, property `post_neutral_xy_velocity`.
Matched budget: N=80 J=5 points per arm, 1200 successful PX4 queries total.

## Arm Outcomes

| arm | J=5 points | robust violations | interior | moderate | saturated | safe | noise band | 2sigma gate rejects | gentlest max|theta| | support | active channels |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| A | 80 | 47 | 0 | 33 | 14 | 26 | 7 | 3 | 0.577 | 26 | pitch,roll,throttle,yaw |
| B | 80 | 24 | 7 | 14 | 3 | 28 | 28 | 15 | 0.373 | 27 | pitch,roll,throttle,yaw |
| C | 80 | 32 | 18 | 6 | 8 | 28 | 20 | 10 | 0.406 | 8 | pitch,roll |

## Robust-Violation Amplitude Percentiles

- Arm A: min=0.577, p25=0.736, median=0.830, p75=0.927, p90=0.972, p95=0.984, max=1.000.
- Arm B: min=0.373, p25=0.486, median=0.606, p75=0.720, p90=0.875, p95=0.947, max=0.989.
- Arm C: min=0.406, p25=0.469, median=0.500, p75=0.812, p90=1.000, p95=1.000, max=1.000.

## Gentlest XY-Velocity Violation

- arm: `B`
- theta hash: `a640eec192e42a4d`
- theta path: `runs/direction_a_px4_position_seed0_v0/thetas/B_00151_a640eec192e42a4d.npy`
- max|theta|: 0.372837 (interior)
- support size |theta|>0.1: 27
- active channels: `pitch,roll,throttle,yaw`
- cross-property rho means: xy_velocity=-0.099429, xy_drift=0.933891, alt_drift=0.989813

Theta (D=40 group order):

```json
[-0.02409588504602479, -0.029167514645372913, -0.0836840469659702, 0.017137981057006448, -0.11024346361459175, 0.022744845401512066, -0.06717062618627509, 0.04892095011479434, -0.16750349530294206, 0.07034715242968606, -0.11695705838936551, 0.08255787592404226, -0.17483432168678592, 0.13126090823834474, -0.12471023984049878, 0.03915381377262643, -0.21045344411106684, 0.2168432750633691, -0.14426394696948094, 0.05993209328977874, -0.2898058008368766, 0.26244429066724706, -0.24286969784144882, 0.10716865033909075, -0.37283742204050824, 0.3052835611878094, -0.1742630307470407, 0.08562710217973145, -0.2840688905219072, 0.26860061903686544, -0.2686903100175018, 0.17381055754655694, -0.30618262850258116, 0.3024647008833733, -0.19818834918649011, 0.08226352959351542, -0.30148833793355767, 0.3269126957318767, -0.14490253304946593, 0.14224287763401383]
```

## Interior Violations

| arm | eval | max|theta| | support | active channels | rho mean | rho std | theta hash |
| --- | ---: | ---: | ---: | --- | ---: | ---: | --- |
| B | 97 | 0.451955 | 28 | pitch,roll,throttle,yaw | -0.129791 | 0.024550 | 1ab5c0d0f90b9060 |
| B | 107 | 0.494727 | 32 | pitch,roll,throttle,yaw | -0.455709 | 0.020747 | 4f9e4e8096aeac56 |
| B | 116 | 0.426143 | 26 | pitch,roll,throttle,yaw | -0.078110 | 0.005913 | 9fd92bf79a81d85e |
| B | 119 | 0.415489 | 25 | pitch,roll,throttle,yaw | -0.027395 | 0.006426 | 1875150e5456d715 |
| B | 147 | 0.458877 | 31 | pitch,roll,throttle,yaw | -0.576164 | 0.009529 | b6f37638eecd435d |
| B | 150 | 0.401517 | 27 | pitch,roll,throttle,yaw | -0.239881 | 0.024488 | f683a5684a3befee |
| B | 151 | 0.372837 | 27 | pitch,roll,throttle,yaw | -0.099429 | 0.008710 | a640eec192e42a4d |
| C | 164 | 0.500000 | 6 | pitch,roll | -0.215798 | 0.027387 | 0556702a8ad39731 |
| C | 168 | 0.468750 | 6 | pitch,roll | -0.101591 | 0.041266 | 7307155a039ce0db |
| C | 174 | 0.500000 | 6 | pitch | -0.275694 | 0.018787 | f7d1cf8cc272f027 |
| C | 178 | 0.468750 | 6 | pitch | -0.120277 | 0.016176 | afbdfb6c0cf0f9b4 |
| C | 179 | 0.453125 | 6 | pitch | -0.041013 | 0.019494 | 5847a4b75d46778f |
| C | 182 | 0.500000 | 8 | pitch,roll | -0.605722 | 0.031542 | 36c4ad75ca8b372b |
| C | 185 | 0.437500 | 8 | pitch,roll | -0.280525 | 0.038954 | 32018296437e814e |
| C | 186 | 0.406250 | 8 | pitch,roll | -0.092292 | 0.021206 | 115d1049daf89b7a |
| C | 191 | 0.500000 | 4 | roll | -0.142648 | 0.021386 | 520f34858a740487 |
| C | 196 | 0.484375 | 4 | roll | -0.048891 | 0.011719 | 9dfde040ab231b58 |
| C | 199 | 0.500000 | 8 | pitch,roll | -0.635278 | 0.037294 | a98d7690b189a629 |
| C | 202 | 0.437500 | 8 | pitch,roll | -0.306472 | 0.041298 | 65ea075e8f5028c3 |
| C | 203 | 0.406250 | 8 | pitch,roll | -0.096880 | 0.038692 | c22e3f18f193458b |
| C | 226 | 0.500000 | 4 | pitch | -0.121563 | 0.025406 | 07cb13c99e811d06 |
| C | 231 | 0.484375 | 4 | pitch | -0.059233 | 0.026915 | a03b4fa95d47c883 |
| C | 234 | 0.500000 | 6 | roll | -0.282551 | 0.024340 | aec082d57a938c50 |
| C | 238 | 0.468750 | 6 | roll | -0.134650 | 0.015616 | 06e3f5b3f479d040 |
| C | 239 | 0.453125 | 6 | roll | -0.038870 | 0.011477 | 58fb2fe932525f32 |

## Decision Inputs

- Arm A interior robust violations: 0
- Arm A robust violations total: 47
- Interior-targeting value condition: `True`
- Channel-reduction gentler than Arm B: `False`
- Channel-reduction cleaner than Arm B: `True`
- Strict no-interior-Arm-A confirmation flag: `True`

The qualitative terms in the decision rule (`few`, `readily`, `clearly`) are left as exact counts and distributions here; no thresholds were tuned after seeing data.

## Artifacts

- pre_registration: `runs/direction_a_px4_position_seed0_v0/reports/pre_registration.json`
- point_evaluations: `runs/direction_a_px4_position_seed0_v0/reports/point_evaluations.csv`
- query_repeats: `runs/direction_a_px4_position_seed0_v0/reports/query_repeats.csv`
- arm_metrics: `runs/direction_a_px4_position_seed0_v0/reports/arm_metrics.csv`
- robust_violations: `runs/direction_a_px4_position_seed0_v0/reports/robust_violations.csv`
- interior_violations: `runs/direction_a_px4_position_seed0_v0/reports/interior_violations.csv`
- summary: `runs/direction_a_px4_position_seed0_v0/reports/direction_a_summary.json`
- report: `runs/direction_a_px4_position_seed0_v0/reports/direction_a_report.md`
- groups: `runs/direction_a_px4_position_seed0_v0/groups.csv`

Successful PX4 queries: 1200.
Timeout retries: 3.
Query attempts including timeout retries: 1203.
Elapsed wall time: 23602.3s.

Single seed/scenario probe only; replicate across seeds before any paper claim.

Stop point: three arms plus classifier only.

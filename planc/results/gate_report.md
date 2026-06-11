# planc gate report: ArduPilot geofence high-speed overshoot

## planc narrative

`planc` is a specification-gap test: under a fixed legal flight-control configuration P, it looks for sparse feasible pilot inputs M and environment conditions E that enter an externally anchored unsafe region while the controller still satisfies its own contract. This gate is a constructed existence test, not a search algorithm.

## gate design

Stack: ArduPilot native ArduCopter SITL, direct `arducopter` binary, TCP MAVLink `tcp:127.0.0.1:5760`. ArduPilot commit `e010f97906`.

P: circular fence enabled (`FENCE_TYPE=2`), `FENCE_RADIUS=100 m`, `FENCE_ACTION=1` (RTL-and-Land), `WPNAV_SPEED=1500 cm/s` for high-speed arms. D uses `WPNAV_SPEED=500 cm/s`. All set parameters were read back and recorded per run.

E: nominal no wind for B; tailwind with `SIM_WIND_DIR=270`, `SIM_WIND_SPD=10 m/s`, `SIM_WIND_TURB=0` for A/C/D. The target bearing is 90 deg, so 270 deg is wind coming from the west and pushing along the outbound path.

M: a single GUIDED global position target 350 m outside the fence for witness arms; C sends one center hover target.

Oracle: B calibrates `overshoot_nominal`; `hard_boundary = R + overshoot_nominal + max(0.20 * overshoot_nominal, 3 m)`. Unsafe means `max_distance > hard_boundary`. Contract-clean means the expected fence breach/action is present for witness arms and no other failsafe or contract violation is logged.

## hard-boundary calibration

- R: 100.00 m
- overshoot_nominal: 0.00 m
- buffer: 3.00 m
- hard_boundary: 103.00 m
- calibration validity: invalid for the intended overshoot oracle because the nominal witness did not cross the fence; the value above is recorded only as the mechanical formula result.

## results

| run | arm | max distance m | max overshoot m | hard boundary m | margin m | contract clean | contract/failsafe findings | verdict |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| B_nominal_r1 | B_nominal | 0.02 | 0.00 | 103.00 | -102.98 | False | NAVIGATION:DEST_OUTSIDE_FENCE | safe_contract_dirty |
| A_witness_r1 | A_witness | 0.28 | 0.00 | 103.00 | -102.72 | False | NAVIGATION:DEST_OUTSIDE_FENCE | safe_contract_dirty |
| A_witness_r2 | A_witness | 0.28 | 0.00 | 103.00 | -102.72 | False | NAVIGATION:DEST_OUTSIDE_FENCE | safe_contract_dirty |
| A_witness_r3 | A_witness | 0.28 | 0.00 | 103.00 | -102.72 | False | NAVIGATION:DEST_OUTSIDE_FENCE | safe_contract_dirty |
| C_hover_r1 | C_hover | 0.28 | 0.00 | 103.00 | -102.72 | True | none | safe_contract_clean |
| D_conservative_r1 | D_conservative | 0.28 | 0.00 | 103.00 | -102.72 | False | NAVIGATION:DEST_OUTSIDE_FENCE | safe_contract_dirty |

## key findings

- Gate conclusion: **GATE FAILED / NO-GO**.
- Reason: Witness GUIDED targets were rejected by ArduPilot as DEST_OUTSIDE_FENCE before a fence crossing occurred: B_nominal_r1, A_witness_r1, A_witness_r2, A_witness_r3, D_conservative_r1.
- Arm A N=3 consistency: False with max-distance spread 0.00 m.
- B safe: False; C safe: True; D safe: False.

## reproducibility

Each run starts a fresh ArduCopter SITL process with `--wipe`, fixed home, fixed speedup, and per-run parameter readback assertions. Arm A runs three repetitions. Raw DataFlash logs, parsed CSV, parameter readbacks, and plots are saved under `planc/logs/` and `planc/analysis/`.

## limitations

Geofence is close to PGFUZZ/RVFuzzer territory, so the distinction is narrow: this gate does not claim a bug when the fence action is correctly triggered. It asks whether the correct action still permits externally unsafe overshoot. SITL fidelity is limited; this gate proves only scenario existence, not generality. The hard boundary is calibrated from nominal no-wind behavior rather than chosen arbitrarily.

## GO/NO-GO

**NO-GO: GATE FAILED.** This scenario did not establish the required unsafe and contract-clean witness with safe controls. Revisit the scenario or oracle before building the full pipeline.

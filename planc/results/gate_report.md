# planc gate report: ArduPilot geofence high-speed overshoot

## v1 to v2 patch

v1 used a GUIDED position target outside the fence, which ArduPilot rejected before motion with `NAVIGATION:DEST_OUTSIDE_FENCE`. v2 changes the witness M to a streamed GUIDED local-NED velocity setpoint. The input has no destination, so aggressiveness is in legal conditions (speed and tailwind), not in an inadmissible command.

## planc narrative

`planc` is a specification-gap test: under a fixed legal flight-control configuration P, it looks for sparse feasible pilot inputs M and environment conditions E that enter an externally anchored unsafe region while the controller still satisfies its own contract. This gate is a constructed existence test, not a search algorithm.

## gate design

Stack: ArduPilot native ArduCopter SITL, direct `arducopter` binary, TCP MAVLink `tcp:127.0.0.1:5760`. ArduPilot commit `e010f97906`.

P: circular fence enabled (`FENCE_TYPE=2`), `FENCE_RADIUS=100 m`, `FENCE_ACTION=1` (RTL-and-Land), `WPNAV_SPEED=2000 cm/s` for high-speed arms. D uses `WPNAV_SPEED=500 cm/s`. All set parameters were read back and recorded per run.

E: nominal no wind for B; tailwind with `SIM_WIND_DIR=270`, `SIM_WIND_SPD=15 m/s`, `SIM_WIND_TURB=0` for A/C/D. The target bearing is 90 deg, so 270 deg is wind coming from the west and pushing along the outbound path.

M: witness arms stream one constant GUIDED local-NED velocity setpoint at 10 Hz along the outbound bearing; C streams zero velocity. The stream stops when a fence breach or configured fence action is observed.

Oracle: B calibrates `overshoot_nominal`; `hard_boundary = R + overshoot_nominal + max(0.20 * overshoot_nominal, 3 m)`. Unsafe means `max_distance > hard_boundary`. Contract-clean means the expected fence breach/action is present for witness arms and no other failsafe or contract violation is logged.

## hard-boundary calibration

- R: 100.00 m
- overshoot_nominal: 5.54 m
- buffer: 3.00 m
- hard_boundary: 108.54 m

## results

| run | arm | max distance m | max overshoot m | hard boundary m | margin m | contract clean | contract/failsafe findings | verdict |
| --- | --- | ---: | ---: | ---: | ---: | --- | --- | --- |
| B_nominal_r1 | B_nominal | 105.54 | 5.54 | 108.54 | -3.00 | True | none | safe_contract_clean |
| A_witness_r1 | A_witness | 111.21 | 11.21 | 108.54 | 2.67 | True | none | unsafe_contract_clean |
| A_witness_r2 | A_witness | 109.94 | 9.94 | 108.54 | 1.39 | True | none | unsafe_contract_clean |
| A_witness_r3 | A_witness | 109.83 | 9.83 | 108.54 | 1.29 | True | none | unsafe_contract_clean |
| C_hover_r1 | C_hover | 0.59 | 0.00 | 108.54 | -107.95 | True | none | safe_contract_clean |
| D_conservative_r1 | D_conservative | 104.04 | 4.04 | 108.54 | -4.50 | True | none | safe_contract_clean |

## key findings

- Gate conclusion: **GATE PASSED**.
- Reason: Arm A was unsafe and contract-clean across N=3, while B/C/D were safe and contract-clean.
- Arm A N=3 consistency: True with max-distance spread 1.38 m.
- B safe: True; C safe: True; D safe: True.
- RC override fallback was not used; all witness runs used admitted GUIDED velocity setpoints.

## reproducibility

Each run starts a fresh ArduCopter SITL process with `--wipe`, fixed home, fixed speedup, and per-run parameter readback assertions. Arm A runs three repetitions. Raw DataFlash logs, parsed CSV, parameter readbacks, and plots are saved under `planc/logs/` and `planc/analysis/`.

## limitations

Geofence is close to PGFUZZ/RVFuzzer territory, so the distinction is narrow: this gate does not claim a bug when the fence action is correctly triggered. It asks whether the correct action still permits externally unsafe overshoot. SITL fidelity is limited; this gate proves only scenario existence, not generality. The hard boundary is calibrated from nominal no-wind behavior rather than chosen arbitrarily.

## GO/NO-GO

**GO: GATE PASSED.** The constructed witness demonstrates unsafe and contract-clean behavior, while B/C/D remain safe. Proceed to the full planc pipeline.

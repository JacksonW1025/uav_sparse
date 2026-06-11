# planc Gate: ArduPilot Geofence Overshoot

`planc` (plan C) is a working name for a flight-control specification-gap test.
For a fixed, legal flight-control configuration `P`, it scans environment
conditions `E` and pilot inputs `M` for feasible, sparse, normal-looking control
actions that enter an externally anchored unsafe region without violating the
flight controller's own documented contract. The aim is not to find a firmware
bug. It is to find SOTIF-style behavior that is unsafe but still contract-clean.

This gate is an existence test, not the full method. It uses construction rather
than search: one sparse witness input and one extreme-but-legal environment are
checked with two oracles.

## Stack

This gate uses ArduPilot's native ArduCopter SITL. It is headless, quick, and
supports native `SIM_*` environment injection including wind. PX4 ports are left
for later work.

## v1 to v2

v1 used a GUIDED position target outside the fence. ArduPilot rejected that
command at admission time with `NAVIGATION:DEST_OUTSIDE_FENCE`, so the aircraft
never exercised the physical geofence crossing path. v2 replaces that witness
with a streamed GUIDED local-NED velocity setpoint. The input has no destination;
the aggressive part is the legal high speed and tailwind condition.

## Scenario

The scenario is high-speed geofence overshoot:

- `P`: circular fence enabled, radius 100 m, `FENCE_ACTION=RTL`, high
  `WPNAV_SPEED`.
- `E`: no-wind nominal control, and a 15 m/s tailwind witness condition.
- `M`: one constant streamed GUIDED velocity setpoint along a fixed bearing.

The hard unsafe boundary is calibrated from the no-wind witness arm:

`hard_boundary = R + overshoot_nominal + buffer`

where `overshoot_nominal = max_distance_nominal - R` and `buffer` is 20% of the
nominal overshoot with a small minimum. A run is unsafe only if it exceeds this
calibrated hard boundary. A contract-clean run must show the fence breach and
configured action, with no other failsafe, crash, parameter, EKF, battery, RC, or
GCS violation.

## Arms

- Arm B: no wind, same sparse witness. This calibrates the hard boundary.
- Arm A: 15 m/s tailwind, high speed, same sparse witness. This runs three
  repetitions.
- Arm C: 15 m/s tailwind, zero-velocity hover near the fence center.
- Arm D: 15 m/s tailwind, same sparse witness, conservative legal speed.

Run all arms with:

```bash
python3 planc/src/run_gate.py
```

Results are written to `planc/results/`, raw DataFlash logs and parsed CSV files
to `planc/logs/`, and plots to `planc/analysis/`.

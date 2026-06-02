# Direction-A ddmin, PX4 Position Seed 0

RQ: necessity baseline testing whether channel-agnostic Arm B violations can be
delta-debugged into the same clean roll/pitch trigger class found by CADET.

Runner:

```bash
python -m sparsepilot.runners.direction_a_ddmin \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --probe-dir runs/direction_a_px4_position_seed0_v0 \
  --run-dir runs/direction_a_ddmin_px4_position_seed0_v1
```

Tracked here:

- `reports/*.csv`
- `reports/direction_a_ddmin_summary.json`
- `reports/pre_registration.json`
- final minimized trigger theta files, one per start

Not tracked here: raw ULog files, parsed per-query logs, intermediate theta
files, and query caches.

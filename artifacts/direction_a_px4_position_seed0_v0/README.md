# Direction-A Probe, PX4 Position Seed 0

RQ: core CADET/Direction-A evidence comparing uniform random, channel-agnostic
boundary probing, and channel-directed roll/pitch probing for
`post_neutral_xy_velocity`.

Runner:

```bash
python -m sparsepilot.runners.direction_a_probe \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/direction_a_px4_position_seed0_v0
```

Tracked here:

- `reports/*.csv`
- `reports/direction_a_summary.json`
- `reports/pre_registration.json`
- Arm C interior trigger theta files
- Arm B theta files selected as ddmin starting points

Not tracked here: raw ULog files, parsed per-query logs, and query caches.

# RQ2 ddmin Necessity Baseline: direction_a_ddmin_px4_position_seed0_v1

Claim: post-hoc channel-agnostic delta debugging cannot match channel-directed
search in this seed-0 probe.

Runner: `cadet.runners.direction_a_ddmin`.
Command:

```bash
python -m cadet.runners.direction_a_ddmin \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --probe-dir runs/direction_a_px4_position_seed0_v0 \
  --run-dir runs/direction_a_ddmin_px4_position_seed0_v1
```

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。**

Included small artifacts are report CSV/JSON files, pre-registration, and the
ten final minimized theta files. Raw logs and full query caches are excluded.

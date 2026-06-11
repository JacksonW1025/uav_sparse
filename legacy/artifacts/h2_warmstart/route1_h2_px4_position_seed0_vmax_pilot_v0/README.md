# H2 Warm-Start Pilot: route1_h2_px4_position_seed0_vmax_pilot_v0

Claim: cross-condition warm starts do not save enough queries in this seed-0
v_max pilot.

Runner: `cadet.runners.route1_h2_campaign`.
Command:

```bash
python -m cadet.runners.route1_h2_campaign \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --theta-v artifacts/margin_stage1_redo_v1/theta_V.npy \
  --run-dir runs/route1_h2_px4_position_seed0_vmax_pilot_v0
```

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。**

Included small artifacts are campaign report CSV/JSON files plus canonical
boundary theta files. Raw logs and query caches are excluded.

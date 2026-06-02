# Boundary Anisotropy: margin_stage1_redo_v1

Claim: the boundary normal is channel-anisotropic at the δ=0.2 redo point.

Runner: `cadet.runners.margin_stage1_redo`.
Command:

```bash
python -m cadet.runners.margin_stage1_redo \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_redo_v1
```

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。**

Included small artifacts are report CSV/JSON files and the Point V theta
anchors. Raw `*.ulg` logs and per-query caches are excluded.

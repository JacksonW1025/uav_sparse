# Margin Stage 1 Redo Anchors

RQ: boundary sensitivity is channel-anisotropic near the seed-0 PX4/POSCTL
Point V boundary.

Runner:

```bash
python -m cadet.runners.margin_stage1_redo \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_redo_v1
```

Tracked here:

- `theta_V.npy`, used as the H2 warm-start anchor
- `theta_117.npy`, used as an H3 prior candidate

Not tracked here: stage reports, raw ULog files, parsed per-query logs, and
query caches.

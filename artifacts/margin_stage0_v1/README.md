# Margin Stage 0 Anchor

RQ: preliminary boundary point used as an H3 prior candidate.

Runner:

```bash
python -m cadet.runners.margin_stage0 \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage0_v1
```

Tracked here:

- `theta_117.npy`, used as an H3 prior candidate

Not tracked here: raw ULog files, parsed per-query logs, candidate `.npz`
files, and query caches.

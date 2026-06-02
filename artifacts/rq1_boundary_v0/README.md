# RQ1 Boundary Anchor

RQ: coarse structure-agnostic boundary search that produced an H3 prior
candidate.

Runner:

```bash
python -m cadet.violation_search \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/rq1_boundary_v0
```

Tracked here:

- `theta_boundary.npy`, used as an H3 prior candidate

Not tracked here: raw ULog files, parsed per-query logs, candidate `.npz`
files, and query caches.

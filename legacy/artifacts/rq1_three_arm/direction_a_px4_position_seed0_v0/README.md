# RQ1 Three-Arm Direction-A Probe: direction_a_px4_position_seed0_v0

Claim: channel-directed search exposes a thin target that random and
channel-agnostic search do not expose cleanly.

Runner: `cadet.runners.direction_a_probe`.
Command:

```bash
python -m cadet.runners.direction_a_probe \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/direction_a_px4_position_seed0_v0
```

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。**

Included small artifacts are report CSV/JSON files, pre-registration, three
representative theta files, and one small parsed minimal-trigger example used
for the paper-hook figure. Raw `*.ulg` logs and full per-query caches are
excluded.

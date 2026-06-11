# H3 Transition Handoff: h3_transition_seed0_v1

Claim: the POSCTL-to-AUTO.LOITER handoff produced no robust transition-specific
violations in this seed-0 probe.

Runner: `cadet.runners.h3_transition`.
Command:

```bash
python -m cadet.runners.h3_transition \
  --config configs/rq1_minimal.yaml \
  --seed 0 \
  --run-dir runs/h3_transition_seed0_v1
```

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。** The H3 runner uses `px4_transition`, but this result is
still only a seed-0 PX4 probe.

Included small artifacts are H3 report CSV/JSON files. Raw logs and query
caches are excluded.

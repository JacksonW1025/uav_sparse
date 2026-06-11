# SPH Plateau and Persistence: legacy_rq1_minimal_v0

Claim: safe-region finite differences behave like a plateau/noise measurement,
and support persistence fails along short projected paths.

Runner lineage: `cadet.runners.repeated_fd` and `cadet.runners.persistence_pilot`.
Original local run path: `runs/archive/rq1_zero_theta_sph_rejected/legacy_rq1_minimal_v0`.

Caveat: **PX4 px4_position（POSCTL）seed-0 单种子/单场景探针；多种子、第二属性、ArduPilot 复现为投稿前必做。**

Included small artifacts are J=5 denoised metrics, persistence step metrics,
transition overlap tables, and persistence summaries. Raw simulator logs,
query caches, seed-1 diagnostics, and ArduPilot diagnostic outputs are excluded.
The `noise≈282×step` prose number is not present as a named field in the
archived small files; this checkpoint keeps the underlying noise estimates but
does not re-state that ratio as a curated headline.

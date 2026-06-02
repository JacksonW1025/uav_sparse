# CADET Small Artifacts

This directory contains versioned, small artifacts for the CADET seed-0
PX4/POSCTL spine runs. Raw simulator logs remain excluded under `runs/`.

Included:

- report CSV files used for reported tables and test fixtures,
- `*_summary.json` and `pre_registration.json` files,
- selected `theta*.npy` files needed to inspect or rerun the key triggers,
- anchor theta files that H2/H3 runners use as default priors.

Excluded:

- `*.ulg`, `*.BIN`, `*.parquet`, and `*.npz`,
- per-query parsed logs,
- full `runs/queries/` caches.

The current trusted scope is PX4 `px4_position`, seed 0. These artifacts make
the reported Direction-A/ddmin numbers auditable from a fresh clone, but they
do not replace rerunning PX4 for additional seeds.

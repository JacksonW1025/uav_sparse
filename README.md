# uav_sparse

## Repo 状态

- 旧实验状态已在 `2026-06-11` 提交并推送，归档标签为 `archive/legacy-pre-planc-20260611`。
- 旧 CADET/MANTIS 实验内容已通过 `git mv` 归档到 `legacy/`，可从该目录或归档标签找回。
- 当前主线实验位于 `planc/`，用于 ArduPilot SITL geofence high-speed overshoot 门控实验。

## Legacy CADET README

# uav_sparse / CADET

Chinese version: [README_zh.md](legacy/README_zh.md)

This repository is the working codebase for **CADET**:
**C**ontrol-**A**llocation-**D**irected **E**xploration of
**T**riggers.

CADET searches for *feasible but unsafe pilot inputs* in UAV flight
controllers: legal, rate-limited stick sequences that violate a flight-mode
safety contract after the pilot returns the sticks to neutral. The Python
package name is `cadet`; the repository name remains `uav_sparse`.

Lineage: CADET is a successor to the RouthSearch style of work. RouthSearch
used a principled stability criterion to structure PID-parameter search; CADET
fixes PID gains, sensors, firmware, mixer, and task, then uses control
allocation to structure pilot-input search.

## Current Status

This is a research measurement pipeline, not a polished benchmark artifact.

The strongest current evidence is for **PX4 `px4_position` / POSCTL, seed 0**.
The docs record a sequence of failed hypotheses that led to the current method:
global sparse gradients failed, warm-started boundary search failed, transition
handoff bugs did not appear, but control-channel-directed trigger synthesis
survived.

Do not overclaim from this repository as-is:

- Quantitative results in the docs are single-seed local experiments.
- `artifacts/` contains the small seed-0 report and trigger files needed to
  audit the current numbers. `runs/` remains ignored because it contains bulky
  raw logs and per-query caches.
- ArduPilot support exists at the adapter level, but CADET has not yet been
  validated there.
- Direction-A / CADET runners are currently frozen to `px4_position`, seed `0`
  in code.

The four narrative documents under `docs/` are part of the research record:

- `docs/01_Research_Narrative_CADET.md`
- `docs/02_CADET_Method.md`
- `docs/03_Paper_Outline_ISSTA_CADET.md`
- `docs/CADET_工作全景梳理.md`

## Research Problem

Given a fixed flight controller configuration:

- fixed PID gains, sensors, firmware, mixer, and task,
- a flight mode such as PX4 POSCTL,
- a safety contract expressed as an STL-style robustness oracle,

CADET searches the pilot-input space for a trigger maneuver `U` such that:

- `U` is feasible: bounded stick magnitude and bounded per-window rate change,
- `U` is non-trivial: internal / non-saturated rather than full-stick abuse,
- the sticks return to neutral,
- the vehicle violates the post-neutral safety contract anyway.

The input model is a 40-dimensional vector:

- 10 time windows,
- 0.5 s per window,
- 4 channels: `roll`, `pitch`, `yaw`, `throttle`,
- 5 s active horizon,
- followed by an 8 s neutral tail.

The implemented contracts are in `src/cadet/properties.py`:

| Property | Robustness |
| --- | --- |
| `post_neutral_xy_drift` | `2.0 m - max horizontal drift` |
| `post_neutral_alt_drift` | `1.0 m - max altitude drift` |
| `post_neutral_xy_velocity` | `1.0 m/s - max horizontal speed` |

Robustness `rho > 0` means safe; `rho < 0` means violation. Boundary decisions
use repeated simulation and a 2-sigma rule: robust violation requires
`rho_mean + 2 * rho_std < 0`.

## What CADET Claims

CADET's precise claim is narrow:

> For feasible-but-unsafe pilot-input bugs, control-allocation-directed search
> can reliably synthesize small, human-readable, single/dual-channel,
> return-to-neutral triggers that random search misses and channel-agnostic
> search plus delta-debugging cannot reliably recover.

It does **not** claim:

- faster discovery of any violation,
- lower peak stick magnitude than every channel-agnostic baseline,
- a viability-theory or cell-sparse boundary guarantee,
- transition-handoff bug discovery,
- cross-seed or cross-platform generality before the missing experiments are
  run.

## Method Sketch

CADET is a two-part method.

1. First reach the violation region without relying on gradients, because the
   safe region behaves like a robustness plateau.
2. Then exploit control allocation: for a given property, search the few control
   channels that physically drive the constrained motion.

The current five stages are:

| Stage | Purpose |
| --- | --- |
| 0. Parameterize | Convert tick-level stick input into the 40D window/channel model. |
| 1. Reach boundary | Use non-gradient sampling / bracketing to find robust safe and violating points. |
| 2. Derive active channels | Map property to channels, e.g. `xy_velocity -> roll,pitch`, then verify with a noise-robust direction probe. |
| 3. Search reduced space | Search channel envelopes only on active channels and bracket the internal boundary. |
| 4. Synthesize trigger | Lightly reduce support/amplitude and report a readable trigger plus channel/time signature. |

The important mechanism is that minimal clean triggers are often not reachable
by deleting cells from a dense shallow violation. Delta-debugging needs
robustness margin to remove cells; the mild, bug-valued violations are near the
boundary and have little margin. CADET searches the reduced channel space
directly.

## Repository Map

| Path | Role |
| --- | --- |
| `configs/` | Experiment configurations. `rq1_minimal.yaml` is the main PX4/AP config; `synthetic_sanity.yaml` is for offline checks. |
| `src/cadet/input_model.py` | Projection to stick limits and rate limits; conversion from theta to time sequence. |
| `src/cadet/groups.py` | 40 channel-time groups. |
| `src/cadet/properties.py` | Post-neutral robustness contracts. |
| `src/cadet/query.py` | Query execution, caching, logs, and adapter dispatch. |
| `src/cadet/vehicle/` | PX4, ArduPilot, and synthetic simulator adapters. |
| `src/cadet/violation_search.py` | Coarse structure-agnostic violation-boundary search. |
| `src/cadet/runners/` | Historical and current experiment runners. |
| `artifacts/` | Versioned small CSV/JSON/theta artifacts for the seed-0 CADET spine runs. |
| `tests/` | Unit and pipeline tests for the input model, metrics, properties, synthetic FD, H3, and Direction-A logic. |
| `scripts/start_px4.sh` | Starts PX4 jMAVSim SITL. |
| `scripts/kill_sim.sh` | Cleans up PX4 / ArduPilot simulator processes. |
| `docs/` | Research narrative, method spec, paper outline, and code-to-claim working notes. |

## Main Experiment Runners

| Claim / Phase | Runner | Notes |
| --- | --- | --- |
| Safe region is a plateau; gradients are noise | `cadet.runners.repeated_fd`, `cadet.runners.persistence_pilot` | SPH-era finite-difference measurements. |
| Boundary sensitivity is channel-anisotropic | `cadet.runners.margin_stage1_redo` | Uses larger direction probe step `delta=0.2`; historical runner still expects earlier boundary/stage runs. |
| Cross-condition warm start does not save queries | `cadet.runners.route1_h2_campaign` | H2 negative result; default Point V anchor is now tracked under `artifacts/`. |
| POSCTL -> LOITER transition handoff is clean | `cadet.runners.h3_transition` | H3 negative result; prior theta candidates are now tracked under `artifacts/`. |
| Random vs. channel-agnostic vs. channel-directed probe | `cadet.runners.direction_a_probe` | Core Direction-A / CADET evidence; pre-registered J=5. |
| Delta-debugging necessity baseline | `cadet.runners.direction_a_ddmin` | Requires the Direction-A probe output as `--probe-dir`. |

Key single-seed Direction-A numbers from the docs:

| Arm | Search style | Interior violations | Trigger shape |
| --- | --- | ---: | --- |
| A | Uniform random | 0 | Finds saturated / trivial violations only. |
| B | Channel-agnostic interior bracketing | 7 | Lower peak amplitude possible, but dense and four-channel. |
| C | Channel-directed roll/pitch search | 18 | Support 4-8, roll/pitch only, readable triggers. |

The ddmin baseline minimized only `4/10` starts to clean triggers; final support
median remained `14.5` versus about `6` for channel-directed triggers.

## Setup

Python 3.10+ is expected.

```bash
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e ".[test]"
```

If you use the existing local virtual environment:

```bash
source .venv/bin/activate
```

For PX4 SITL, set `PX4_ROOT` if your checkout is not
`/home/car/PX4-Autopilot`:

```bash
PX4_ROOT=/path/to/PX4-Autopilot scripts/start_px4.sh
```

Clean simulator processes with:

```bash
scripts/kill_sim.sh
```

## Quick Checks

Config and parameterization dry run:

```bash
python -m cadet.runners.sanity \
  --config configs/synthetic_sanity.yaml \
  --dry-run
```

Run unit tests. On machines with ROS installed, disable third-party pytest
plugin autoloading; otherwise `launch_testing_ros` can break collection.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q
```

Verified locally in this workspace:

```text
31 passed in 22.80s
```

## Running PX4 Experiments

Start PX4 SITL in one terminal:

```bash
PX4_ROOT=/path/to/PX4-Autopilot scripts/start_px4.sh
```

Run a PX4 smoke check in another terminal:

```bash
python -m cadet.runners.smoke \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --skip-probes
```

Run the current Direction-A / CADET three-arm probe:

```bash
python -m cadet.runners.direction_a_probe \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/direction_a_px4_position_seed0_v0
```

Then run the ddmin necessity baseline using that probe output:

```bash
python -m cadet.runners.direction_a_ddmin \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --probe-dir runs/direction_a_px4_position_seed0_v0 \
  --run-dir runs/direction_a_ddmin_px4_position_seed0_v1
```

Run the coarse structure-agnostic boundary search:

```bash
python -m cadet.violation_search \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/rq1_boundary_v0
```

Run the historical H1 boundary-anisotropy chain:

```bash
python -m cadet.runners.margin_stage0 \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage0_v1

python -m cadet.runners.margin_stage1 \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_v1

python -m cadet.runners.margin_stage1_redo \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --run-dir runs/margin_stage1_redo_v1
```

Run the H2 cross-condition warm-start pilot from the archived Point V anchor:

```bash
python -m cadet.runners.route1_h2_campaign \
  --config configs/rq1_minimal.yaml \
  --scenario px4_position \
  --seed 0 \
  --theta-v artifacts/margin_stage1_redo_v1/theta_V.npy \
  --run-dir runs/route1_h2_px4_position_seed0_vmax_pilot_v0
```

Run the H3 POSCTL-to-LOITER transition probe from archived prior candidates:

```bash
python -m cadet.runners.h3_transition \
  --config configs/rq1_minimal.yaml \
  --seed 0 \
  --run-dir runs/h3_transition_seed0_v1
```

Historical H1 commands require their predecessor runs because the refined
candidate `.npz` and some FD snapshots are intentionally not tracked. Check each
runner's `--help` output and pass explicit paths when rerunning a campaign from
scratch.

## Outputs

Experiment outputs are written under `runs/`:

- `queries/*/input_theta.npy`
- `queries/*/input_sequence.csv`
- `queries/*/robustness.json`
- `queries/*/metadata.json`
- `reports/*.csv`
- `reports/*_summary.json`
- `reports/*_report.md`
- `groups.csv`

These files are not committed by default. For paper work, archive the small
report and trigger artifacts separately from bulky simulator logs:

- `reports/*.csv`
- `reports/*_summary.json`
- `reports/pre_registration.json`
- minimal trigger `theta.npy` files
- any tables used for paper numbers

## Small Artifacts

The tracked `artifacts/` directory contains the seed-0 CADET spine run outputs
that are small enough to keep in Git:

- `artifacts/direction_a_px4_position_seed0_v0/`
- `artifacts/direction_a_ddmin_px4_position_seed0_v1/`
- `artifacts/margin_stage1_redo_v1/`
- `artifacts/rq1_boundary_v0/`
- `artifacts/margin_stage0_v1/`

Each subdirectory has a README explaining the RQ, runner, command, included
CSV/JSON/theta files, and omitted raw logs. These artifacts are for auditing
reported numbers and for default theta anchors; rerunning PX4 still writes fresh
data under `runs/`.

## Runtime and Failure Modes

On the local PX4/jMAVSim setup used for the archived seed-0 runs, one simulator
repeat took roughly 20 seconds wall time. A J=5 robustness point therefore costs
about 1-2 minutes. The archived Direction-A probe took about 6.6 hours for 1200
successful repeats; the ddmin baseline took about 10.7 hours for 1965 repeats;
the H2 and H3 pilots took about 6.8 hours and 1.7 hours respectively.

Known failure modes:

- third-party ROS pytest plugins can break collection; use
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 pytest -q`,
- missing or wrong `PX4_ROOT` prevents SITL startup,
- stale PX4/jMAVSim processes and MAVLink ports require `scripts/kill_sim.sh`,
- excessive simulator speedup can make repeated robustness too noisy,
- missing predecessor runs break historical H1 runners unless explicit paths are
  supplied,
- raw `runs/` data is local-only unless small artifacts are deliberately copied
  into `artifacts/`.

## Known Gaps

Blocking before a paper-level claim:

- rerun Direction-A probe and ddmin on at least two additional seeds,
- add a second property such as `xy_drift`,
- derive active channels from control allocation rather than using the current
  hard-coded `["roll", "pitch"]` for `xy_velocity`,
- validate CADET on ArduPilot,
- compute cost per distinct trigger family,
- rerun historical H1 from a fresh clone with only tracked artifacts and
  documented predecessor commands.

Engineering caveats:

- `configs/rq1_minimal.yaml` uses stick limits `[-0.7, 0.7]`, while the
  Direction-A probe overrides to `[-1.0, 1.0]` so the saturated class is
  reachable. Paper text should use one explicit feasibility convention.
- `runs/`, `*.ulg`, `*.parquet`, and `*.npz` are ignored. This keeps the repo
  small but means claims and local artifacts must be curated deliberately.

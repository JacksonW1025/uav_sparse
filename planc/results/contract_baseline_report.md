# Contract-Testing Baseline — OverDraw Claim C2

## VERDICT: **CONFIRMED**

Claim **C2**: *contract-clean unsafe states (OverDraw, region `A ∧ ¬B`) are
structurally invisible to contract testing (PGFUZZ / RVFuzzer family, which flag
only rule violations).* An **independently implemented**, documentation-faithful
PGFUZZ-style contract/policy checker, run on the already-produced OverDraw traces
and on real contract-violation positive controls, confirms C2:

1. **Tier-1 (contract) violations = 0 across all 607 OverDraw runs** (oracleA v1: 65; ep v1: 542) — including all 490 `clean_unsafe` runs.
2. **The checker is non-trivial:** it produces Tier-1 hits on every positive control, covering all four Tier-1 policies (T1a/T1b/T1c/T1d).
3. **The anti-leakage audit passes** at field, value, and code level (no oracle-A consequence signal enters any Tier-1 policy).

The checker is implemented from scratch (`planc/src/contract_baseline_v1.py`); it
reads only raw decoded telemetry (`*_parsed.csv`), the operator command profile
(`*_command_profile.json`), and the parameter snapshot (`*_params.json`). It
never opens the project's own `*.oracle.json` (oracle A/B) sidecars, so "0
contract hits" is an *independent* result, not a restatement of oracle B.

---

## Contract-blindness comparison table

Two tiers, counted separately. **Tier-1 = CONTRACT (ISO 26262 / FuSA)**: the flight
controller's *specified* contracts. **Tier-2 = RESULT (ISO 21448 / SOTIF safety
goals)**: PGFUZZ-style physical-outcome predicates (not contracts).

| dataset | n | **Tier-1 any** | T1a clamp | T1b failsafe | T1c limit | T1d mode | **Tier-2 any** | T2a attitude | T2b alt-loss | T2c crash |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| **OverDraw — oracleA v1** | 65 | **0** | 0 | 0 | 0 | 0 | 59 | 59 | 49 | 44 |
| **OverDraw — ep v1 (E/P grid + reach)** | 542 | **0** | 0 | 0 | 0 | 0 | 514 | 507 | 440 | 399 |
| **OverDraw — total** | **607** | **0** | 0 | 0 | 0 | 0 | 573 | 566 | 489 | 443 |
| Control — PC_CLAMP (cmd > ANGLE_MAX) | 2 | **2** | **2** | 0 | 0 | 0 | 2 | 1 | 2 | 1 |
| Control — PC_FENCE (geofence breach) | 5 | **5** | n/a | **5** | **5** | **5** | 5 | 0 | 0 | 5 |
| Control — PC_GCS (GCS link-loss FS) | 4 | **4** | n/a | **4** | 0 | **4** | 4 | 0 | 0 | 4 |

Reading: the contract checker is **silent on the entire OverDraw region (Tier-1 =
0/607)** yet **fires on 100 % of the 490 `clean_unsafe` runs at Tier-2** (every
`clean_unsafe` run trips T2a/T2b/T2c). On the positive controls — real ArduCopter
SITL traces with genuine contract violations — Tier-1 fires as expected, proving
the silence on OverDraw is a property of the region, not a dead checker.

### Tier-1 policies (each with a documented source)

| id | contract | doc source | OverDraw | control that fires it |
|---|---|---|---:|---|
| **T1a** | commanded lean ≤ ANGLE_MAX | `ANGLE_MAX` param, `ArduCopter/Parameters.cpp:350` ("max lean angle in all flight modes", range 1000–8000 cd) | 0 | PC_CLAMP (cmd 58.5° > 45°) |
| **T1b** | no spurious preventive failsafe | radio/batt/GCS/EKF/GPS/fence/terrain/leak/deadreckon failsafe docs (`events.cpp`, `ekf_check.cpp`, `fence.cpp`) | 0 | PC_FENCE, PC_GCS |
| **T1c** | configured-limit / EKF-failsafe-action compliance | `FENCE_*`, `FS_EKF_ACTION` param docs (`AC_Fence.cpp:45`, `Parameters.cpp:385`) | 0 | PC_FENCE |
| **T1d** | mode transitions conform to spec | ArduCopter mode docs + `ModeReason` enum (`libraries/AP_Vehicle/ModeReason.h`) | 0 | PC_FENCE, PC_GCS |

### Tier-2 policies (SOTIF safety goals — *not* contracts)

| id | safety goal | source |
|---|---|---|
| **T2a** | achieved attitude within physical bound (sustained err > 60°/0.5 s) | PGFUZZ "attitude within limits" |
| **T2b** | no uncommanded altitude loss > 15 m | PGFUZZ "altitude maintained" |
| **T2c** | no ground contact / crash | PGFUZZ "drone should not crash" |

---

## Anti-leakage audit (Defence line 1: anti-circularity) — **PASS**

The Tier-1 checker may read only what a contract tester can see (commands, modes,
documented failsafe conditions). It must not read any oracle-A *consequence*
signal. Verified three ways (`contract_baseline_result.json → anti_leakage_audit`):

- **Field-set:** the union of declared Tier-1 input fields is **disjoint** from the
  oracle-A consequence set `{ATT.Roll/Pitch (achieved), POS.RelHomeAlt, XKF1.VD,
  ground-contact text, CRASH_CHECK, CRASH_FAILSAFE, XKF4.FS}` (intersection = ∅).
- **Value-level:** the raw fields shared with Tier-2 (`ERR.subsystem_name`,
  `MODE.reason_name`, `MSG.text`) are read by Tier-1 only for *values* that
  exclude the crash/ground consequence values — machine-checked: `CRASH_CHECK ∉`
  Tier-1 preventive subsystems; `CRASH_FAILSAFE ∉` Tier-1 preventive reasons; no
  ground/crash marker in the Tier-1 failsafe-text set.
- **Code-level:** the Tier-1 evaluator functions (`_eval_T1a/b/c/d`,
  `_failsafe_events`) reference **no** achieved-state attribute (`run.pos`,
  `run.xkf1_vd`, `run.xkf4_fs`, achieved roll/pitch, ground text), verified after
  stripping docstrings/comments via the AST; and the checker module opens no
  `oracle.json` sidecar in executable code.

Two leakage **traps that were deliberately avoided** (and would have produced a
false REFUTED):

- **The crash time is never used to gate Tier-1.** The failsafe/mode scan runs over
  the *entire* run with no consequence gate. (Oracle B used a "before hard-A"
  gate; that would have leaked the crash time. We do not — Tier-1 is strictly
  consequence-independent, and still = 0.)
- **`XKF4.FS` (raw EKF fault bitmask) is excluded from Tier-1.** It goes nonzero
  during aggressive/crash dynamics and would have falsely fired. The EKF *contract*
  is the failsafe **actuation** (`EKFCHECK` / `FAILSAFE_EKFINAV` / `EKF_FAILSAFE` =
  0 across all 607 runs), which T1b/T1c check. `FS_EKF_ACTION=1` is enabled, so this
  arm is **live, not vacuous** — the EKF estimator simply stayed healthy through the
  attitude divergence, a genuine pass.

**BIN cross-validation:** for 12 OverDraw runs (sampled across both experiments) the
checker was re-run by re-parsing the raw DataFlash `.BIN` directly instead of the
`_parsed.csv`. The failsafe/mode Tier-1 verdict matched in **12/12** cases — the CSV
fast-path launders nothing.

---

## SOTIF / OverDraw seam, and the dual-oracle value (Discussion)

The table makes the gap concrete: across 607 runs the contract checker sees **zero**
rule violations, while the physical safety goals are violated on **100 % of the 490
`clean_unsafe` runs** (crash, >15 m altitude loss, attitude past the envelope). That
gap — **Tier-1 clean ∧ Tier-2 hit** — *is* the OverDraw / SOTIF deficiency. The flight
controller honoured every documented contract (it clamped lean-angle *commands* to
ANGLE_MAX, fired no failsafe, made no illegitimate mode change) and the aircraft was
nonetheless driven into an externally-unsafe state by a legal input.

**The mechanism is in the firmware, not in the checker.** ANGLE_MAX clamps the
lean-angle *command* path (pilot stick via `get_pilot_desired_lean_angles`;
navigation via `AC_PosControl::get_lean_angle_max_cd`). In GUIDED_NOGPS the operator
streams *body-rate* setpoints, and the rate path
(`ModeGuided::angle_control_run → AC_AttitudeControl::input_rate_bf_roll_pitch_yaw`,
`AC_AttitudeControl.cpp:421-455`) integrates the commanded rate into the attitude
target **with no ANGLE_MAX reference** — confirmed by adversarial source review. So
the demanded attitude legitimately escapes the ANGLE_MAX *value* (≈177°) while the
ANGLE_MAX *contract* (a bound on commands) is never violated. The envelope is
*insufficient*, not *breached*.

**Why even a Tier-2-equipped PGFUZZ cannot close the gap (the dual-oracle point).**
A real PGFUZZ author would also write physical-state predicates (our Tier-2). Two
things follow, both confirmed by the completeness critics:

- The one predicate that *does* fire on OverDraw — "vehicle/demanded attitude ≤
  ANGLE_MAX", read as a *state* property (DesRoll 176.7°, achieved 178.2°) — is **not
  a faithful contract**. It reads the achieved/demanded attitude, i.e. an oracle-A
  consequence; it is a SOTIF *safety goal* (our T2a), not a rule the FC promised to
  enforce. A naive checker that lifts the ANGLE_MAX doc string into a state predicate
  would see it fire — but, **having no oracle B, it would report only "policy
  violation" and the developer would chase it as a firmware bug, never learning that
  the contract is in fact satisfied.** That is precisely the value of the dual oracle:
  it separates "the spec was broken" (a bug) from "the spec was kept but is
  insufficient" (OverDraw).
- RVFuzzer's control-instability / reference-tracking check also misses OverDraw: the
  attitude controller *tracks its (unclamped) demanded attitude well* (median in-window
  tracking error ≈ 6.7°). The instability is in the *demand*, not in the *tracking* —
  invisible to a tracking-error detector.

Genuine Tier-1 contracts that a contract fuzzer would check and that *are* live here —
EKF-failsafe health, motor/thrust authority, arm-time lean check — are all genuinely
**satisfied** on the OverDraw runs (not merely unchecked). This strengthens, rather
than weakens, C2: the contract layer is exercised and clean.

---

## Adversarial verification (Defence line 2: anti-strawman)

A 10-agent background workflow (`planc/results/contract_baseline_audit.json`)
independently scrutinised the result:

- **Provenance (4 agents, one per Tier-1 policy):** all four policies confirmed
  **faithful to ArduPilot source** (file:line evidence). The only defect found was a
  stale citation — T1d pointed at `ArduCopter/defines.h` for the `ModeReason` enum,
  which in ArduPilot 4.4.1 lives in `libraries/AP_Vehicle/ModeReason.h`; the enum
  *content* matched the checker. **Now corrected** in the checker and prereg.
- **Leakage (3 independent skeptics, instructed to refute):** all three found **no
  leak** (severity = none) — at field, value, code, window/gating, call-graph-closure,
  and empirical levels.
- **Completeness (2 critics, one further critic hit a session limit):** **0 credible
  C2 threats.** No faithful Tier-1 contract was found that fires on the contract-clean
  OverDraw region; the only firing predicates are Tier-2 state/consequence detectors
  (already reported) or are vacuous/gated-off (e.g. PGFUZZ `A.FLIP*` — never enters
  FLIP mode).

---

## Pre-registration and verdict criteria

Pre-registered in `planc/results/contract_baseline_prereg.json` before the full run.

- **CONFIRMED** = (1) OverDraw Tier-1 = 0 ∧ (2) ≥1 non-trivial positive control ∧ (3)
  anti-leakage audit passes. → **all three hold.**
- REFUTED = some OverDraw run has Tier-1 > 0 (→ re-check oracle B). → **0 offenders.**
- INCONCLUSIVE = checker not provably non-trivial, or no usable control. → not the case.

## Artifacts

- Checker: `planc/src/contract_baseline_v1.py`
- Orchestrator: `planc/src/run_contract_baseline_v1.py`
- PC_CLAMP SITL runner: `planc/src/run_contract_poscontrol_v1.py`
- Pre-registration: `planc/results/contract_baseline_prereg.json`
- Result JSON: `planc/results/contract_baseline_result.json`
- Adversarial audit: `planc/results/contract_baseline_audit.json`
- PC_CLAMP run summary: `planc/results/contract_pc_clamp_v1.json`
- Positive-control sources: legacy geofence-breach + GCS-failsafe DataFlash logs (no new SITL); PC_CLAMP = 2 new SITL runs (raw logs gitignored per repo convention, like the OverDraw logs).
- Tag: `planc/contract-baseline-v1-20260616`

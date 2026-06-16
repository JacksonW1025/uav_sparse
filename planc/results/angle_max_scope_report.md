# ANGLE_MAX Scope Investigation — by-design vs. oversight for the body-rate command path

**VERDICT: BY-DESIGN** &nbsp;|&nbsp; **Confidence: 0.87**

> The fact that `ANGLE_MAX` does not constrain the body-rate command path
> (`AC_AttitudeControl::input_rate_bf_roll_pitch_yaw`) is an **intentional scope
> limitation**, not a missing clamp. `ANGLE_MAX` is, by design, a *lean-angle command*
> limit applied caller-side on the angle/position paths; the attitude controller's
> **rate and quaternion interfaces are by design not bound by it** (so that ACRO
> aerobatics and FLIP can command full attitude). The rate path is **governed by a
> separate, intended regime** (`ACRO_RP_RATE` / `ATC_RATE_*_MAX` / acceleration
> shaping), not left ungoverned. Therefore the OverDraw contract genuinely holds on
> the rate path → the **SOTIF specification-insufficiency framing stands and C3 is
> strengthened** (the gap cannot be reliably closed by a single clamp).

This is the prereg's BY-DESIGN branch (`angle_max_scope_prereg.json`). One honest
caveat (Section I3 / I6) refines *which interface* the paper should anchor its
demonstration on, but does not change the scope verdict.

---

## I0 — Pinned target & sources

| Item | Value | Source |
|---|---|---|
| Vehicle / version | ArduCopter **V4.4.1** | `ArduCopter/version.h:9` (`THISFIRMWARE "ArduCopter V4.4.1"`) |
| Git tag | `Copter-4.4.1` | `git describe --tags` in source tree |
| HEAD commit | `e010f97906087a3a1975e1c4fcc1f88a249599ce` | `git rev-parse HEAD` |
| Local source tree | `/mnt/nvme/UAVFuzzing/ardupilot-home` | this is the tree SITL builds from |
| Canonical remote | `https://github.com/ArduPilot/ardupilot.git` | `git remote -v` |
| Doc source | `https://ardupilot.org` (Copter docs + parameter ref) + `https://mavlink.io` | corroborating |

Independent re-derivation: the call-graph below was grepped from the `ANGLE_MAX`
parameter outward, not copied from the contract-baseline probe. Evidence grade is
labelled per row: **[DOC]** documented intent > **[CODE]** code structure >
**[HIST]** commit/PR/issue > **[INFER]** inference.

---

## I1 — `ANGLE_MAX` scope table (path → clamp? → source)

`ANGLE_MAX` is stored as `aparm.angle_max` and surfaced as
`AC_AttitudeControl::lean_angle_max_cd()` (`libraries/AC_AttitudeControl/AC_AttitudeControl.h:307`,
`return _aparm.angle_max;`). Param metadata: *"Maximum lean angle in all flight modes"*,
units cdeg, range 1000–8000 (`ArduCopter/Parameters.cpp:350-357`).

### Paths that ARE clamped by ANGLE_MAX (angle-command path)

| Path | Where ANGLE_MAX is applied | Source | Grade |
|---|---|---|---|
| Pilot lean-angle (STABILIZE/ALT_HOLD/LAND/DRIFT/SPORT/AUTOROTATE/SYSTEMID…) | `get_pilot_desired_lean_angles()` passes `aparm.angle_max` into `rc_input_to_roll_pitch()`, which caps the stick→lean geometrically (and hard-caps at 85°) | `ArduCopter/mode.cpp:415-431`; `libraries/AP_Math/control.cpp:546-561` (`angle_max_deg = MIN(angle_max_deg,85.0)`, `thrust.limit_length(thrust_limit)`) | [CODE] |
| Position/velocity controller (LOITER/POSHOLD/GUIDED pos/AUTO) | accel target limited so the resulting lean ≤ ANGLE_MAX | `libraries/AC_AttitudeControl/AC_PosControl.cpp:506-509`, `:645`; `get_lean_angle_max_cd()` → `lean_angle_max_cd()` `:996-1004` | [CODE] |
| Loiter pilot lean | `AC_Loiter::get_angle_max_cd()` = MIN(ANGLE_MAX, PSC_ANGLE_MAX)·2/3 | `libraries/AC_WPNav/AC_Loiter.cpp:180-185` | [CODE] |
| AUTO attitude-time command | `angle_limit_cd = MAX(1000, MIN(aparm.angle_max, althold_max))` then `input_euler_angle_roll_pitch_yaw` | `ArduCopter/mode_auto.cpp:1148-1153` | [CODE] |
| Near-ground nav (below `WPNAV_NAVALT_MIN`) | thrust-vector attitude interpolated down toward `aparm.angle_max` | `ArduCopter/mode.cpp:711-723` | [CODE] |
| Pre-arm "Leaning" gate | refuse arming if tilt > ANGLE_MAX | `ArduCopter/AP_Arming.cpp:606-611` | [CODE] |
| FlowHold body-angle | `constrain_float(bf_angles, ±aparm.angle_max)` | `ArduCopter/mode_flowhold.cpp:199-203` | [CODE] |
| ACRO **only** when `ACRO_TRAINER==LIMITED` (opt-in) | soft "pull back toward level" rate when `roll/pitch_angle > angle_max` — **not a hard clamp** | `ArduCopter/mode_acro.cpp:149-162` | [CODE] |

### Paths that are NOT clamped by ANGLE_MAX (body-rate / quaternion path)

| Path | Behavior | Source | Grade |
|---|---|---|---|
| **`input_rate_bf_roll_pitch_yaw`** | integrates body-rate command straight into the quaternion attitude target (`_attitude_target = _attitude_target * from_axis_angle(rate·dt)`), normalize — **no angle limit**, only accel shaping | `libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:421-455` (esp. 444-446) | [CODE] |
| `input_rate_bf_roll_pitch_yaw_2` / `_3` (rate-loop-only) | no angle limit; `_3` limits only the integrated *error* angle (`AC_ATTITUDE_THRUST_ERROR_ANGLE`), not ANGLE_MAX | `:458-531` | [CODE] |
| `input_quaternion` (GUIDED attitude/quaternion target) | sets `_attitude_target = attitude_desired_quat`; only `ang_vel_limit()` (rate) is applied — **no ANGLE_MAX** | `:231-266` | [CODE] |
| `input_euler_rate_roll_pitch_yaw` | only pitch is `constrain_float(…, ±85°)` (gimbal-lock guard, *not* ANGLE_MAX); roll/yaw wrap freely | `:379-418` (line 405) | [CODE] |

**I1 result: CONFIRMED.** ANGLE_MAX is a *caller-side clamp on the lean-angle command*.
None of the attitude controller's `input_*` functions clamp ANGLE_MAX internally — the
angle paths are clamped *before* calling `input_euler_angle_*`, while the rate and
quaternion paths have no such upstream clamp. The body-rate primitive
`input_rate_bf_roll_pitch_yaw` is **structurally unbounded** by ANGLE_MAX.

---

## I2 — Intent evidence (documented + structural)

1. **ANGLE_MAX is documented as an *angle* limit, not an attitude ceiling.**
   *"Maximum lean angle in all flight modes"* (`ArduCopter/Parameters.cpp:352`). "Lean
   angle" is a command-path concept (the tilt the *thrust-vector / stick command*
   produces). The rate interfaces command *angular velocity*, not a lean angle. **[DOC]**
   *(This is also the one phrase a reviewer can lean on — addressed in I6.)*

2. **ACRO is documented as intentionally aerobatic / non-self-leveling.**
   *"Acro mode uses the RC sticks to control the angular velocity… Release the sticks
   and the vehicle will maintain its current attitude and will not return to level…
   useful for aerobatics such as flips or rolls"* (`https://ardupilot.org/copter/docs/acro-mode.html`). **[DOC]**

3. **The rate path has its own, separate governance** — it is *not* ungoverned:
   - `ACRO_RP_RATE` / `ACRO_Y_RATE` (max rotation rate), `ACRO_RP_EXPO` / `ACRO_Y_EXPO`,
     `ACRO_RP_RATE_TC` — `ArduCopter/Parameters.cpp:1058-1098`; consumed in
     `ArduCopter/mode_acro.cpp:122-128`. **[DOC]/[CODE]**
   - Attitude-controller rate ceiling `ATC_RATE_R/P/Y_MAX` (`_ang_vel_*_max`,
     `AC_AttitudeControl.cpp:113-141`) and `ATC_ACCEL_*_MAX` acceleration shaping
     applied in `input_shaping_ang_vel` inside `input_rate_bf_*`. **[CODE]**
   The rate interface is governed by a **rate/acceleration regime**, which is the
   appropriate governance for a velocity command — orthogonal to an angle limit.

4. **ANGLE_MAX reaches the rate context only through an explicit opt-in.** The angle
   limit applies to ACRO *only* when `ACRO_TRAINER==2 ("Leveling and Limited")`, and the
   docs define `ACRO_TRAINER=0` as *"full Rate control with no automatic leveling nor
   angle-limiting"* (`acro-mode.html`). The firmware author wrote ANGLE_MAX-limiting code
   for ACRO (`mode_acro.cpp:149-162`) and **deliberately gated it behind a training-wheels
   option** rather than clamping the primitive. **[DOC]/[CODE]** — strong structural
   intent: the absence of a rate-path clamp is a chosen baseline, not an unconsidered gap.

**I2 result: intent = TRUE.** Documented + structural evidence converge: the rate path
is *meant* to allow attitude beyond ANGLE_MAX and carries its own governance.

---

## I3 — Interface legitimacy (with the one nuance the paper must handle)

**Routing verified end-to-end (code):**
`SET_ATTITUDE_TARGET` (MAVLink msg 82) → `GCS_Mavlink.cpp:1122-1195` (requires
`in_guided_mode()`; decodes `type_mask` body-rate-ignore + attitude-ignore bits; fills
`ang_vel` from `body_roll/pitch/yaw_rate`) → `ModeGuided::set_angle()`
(`mode_guided.cpp:626`) → `ModeGuided::angle_control_run()` (`:911`) → **when the
quaternion is ignored/zero, `input_rate_bf_roll_pitch_yaw(ang_vel)`** (`:964-965`).
`GUIDED_NOGPS` (mode 20, *"guided mode but only accepts attitude and altitude"*,
`mode.h:33`) inits via `angle_control_start()` (`mode_guided_nogps.cpp:13`) and shares the
same handler. ArduCopter advertises `MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET`
(`GCS_Mavlink.cpp:1459`). **[CODE]**

**Protocol level:** body-rate setpoints are first-class, documented MAVLink fields —
`body_roll_rate/body_pitch_rate/body_yaw_rate` [rad/s] with dedicated `type_mask`
ignore bits (`https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET`). **[DOC]**

**NUANCE (must be stated honestly):** ArduPilot's *developer* doc
`https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html` marks the
`SET_ATTITUDE_TARGET` body-rate fields **"not supported"** for GUIDED and instructs
`type_mask … should always be 0b00000111` (i.e. ignore all three body rates, drive by
quaternion attitude only). So the **GUIDED body-rate route is doc-discouraged** — even
though the 4.4.1 firmware does *not enforce* that (it reads the rates and acts on them).
This does **not** rescue a clamp: even the *supported* GUIDED quaternion-attitude path
(`input_quaternion`, `:231-266`) does **not** enforce ANGLE_MAX either — it only
rate-limits. **[DOC]/[CODE]**

**I3 result: legitimacy = TRUE, with a re-anchoring recommendation.** The scope gap is
reachable under *fully supported* operation through:
  - **ACRO** (a documented, supported mode built on the same unbounded primitive — the
    cleanest demonstration), and
  - a **GUIDED attitude-quaternion target > ANGLE_MAX** (supported interface, also
    unclamped by `input_quaternion`).
The GUIDED *body-rate* field (which the predecessor probe used) is real in code but
doc-discouraged; the paper should re-anchor on ACRO / the quaternion path so a reviewer
cannot dismiss the demonstration as "an unsupported input." See I6.

---

## I4 — History / controversy

| # | Evidence | Source | Tag |
|---|---|---|---|
| 1 | Rate-only attitude path was a **deliberately added feature** by a core maintainer (Leonard Hall) | commit `c53ba22daa` "AC_AttitudeControl: add new rate only attitude control" (verified present in local clone) | [BY-DESIGN] [HIST] |
| 2 | **No commit ever added/removed an ANGLE_MAX clamp in the rate path.** `git log -S angle_max -- AC_AttitudeControl.cpp` returns only althold-limit / `RATE_RP_MAX` / earth-frame-constraint / ACRO-fix work — none a rate-path ANGLE_MAX clamp | local `git log -S "angle_max"`; independently re-run | [BY-DESIGN] [HIST] |
| 3 | Rate path is **stable across the 4.3.x → 4.4.1 history** in the pinned clone (only cosmetic refactors of `input_rate_bf_roll_pitch_yaw`); agent reports continued stability through later releases (lower-confidence, clone tags only verified to 4.4.x) | local tags `Copter-4.3.0 … 4.4.1` | [BY-DESIGN] [HIST] |
| 4 | **Angle-limiting in ACRO was requested as an *Enhancement*, not reported as a bug.** Issue **#11888** "Copter: Acro Trainer attitude limit only" (opener MrVoltz, labels **Copter / Enhancement**, **Closed**): *"I would like to be able to use only the angle limiting option of ACRO_TRAINER, so it works the same as Acro Trainer in Betaflight."* → implemented as opt-in. The baseline (no rate-path clamp) was the deliberate baseline | `https://github.com/ArduPilot/ardupilot/issues/11888` (independently verified via WebFetch) | [BY-DESIGN] [HIST] |
| 5 | Contrast: the **one** ANGLE_MAX *bug* found (#10524 "FlowHold can lean past ANGLE_MAX") was in the **stabilized lean-angle path** — wrong arg to `get_pilot_desired_lean_angles()` — i.e. a path that *is* meant to honor ANGLE_MAX. Maintainers fix ANGLE_MAX leaks in *angle-command* paths; they conspicuously have **not** for the rate path | `https://github.com/ArduPilot/ardupilot/issues/10524` (agent-reported) | [NEUTRAL→BY-DESIGN] [HIST] |
| 6 | **No issue/PR anywhere asserts the rate path / ACRO / SET_ATTITUDE_TARGET-rate ignoring ANGLE_MAX is a defect.** No WONTFIX-style verbatim maintainer pronouncement either — intent is expressed structurally + in the wiki, not as a single sentence | GitHub + forum search (agent) | [BY-DESIGN (silence-of-bug-reports)] [HIST] |

*Note on the history agent:* some of its commit **dates** were implausible (e.g. a
"2026-01-21" stamp) and its "stable through 4.6.3" claim could not be confirmed (the
pinned clone's tags reach ~4.4.x). I therefore independently re-verified the load-bearing
items (#11888 via WebFetch; `c53ba22daa` and the `-S angle_max` history via local git) and
downgraded the cross-version-stability claim to what I could verify (4.3.x→4.4.1).

**I4 result:** history is consistent with **intended/by-design and never litigated as a
bug**. The strongest single signal: a user who *wanted* ANGLE_MAX limiting in ACRO had to
file an **enhancement** and get a new opt-in feature built — confirming the unclamped rate
path is the deliberate baseline.

---

## I5 — Fixability / design tension

`input_rate_bf_roll_pitch_yaw` is a **shared primitive** deliberately driven beyond
ANGLE_MAX by intended functionality:

- **FLIP mode** commands it at `FLIP_ROTATION_RATE` (≈400°/s) through a **full inverted
  flip** (state machine Start→Roll/Pitch→Recover, explicitly passing ±90° and inversion)
  — `ArduCopter/mode_flip.cpp:118-175`. A global ANGLE_MAX attitude bound would make FLIP
  **impossible**. **[CODE]**
- **ACRO** aerobatics (flips/rolls) require unbounded attitude by documented design
  (I2.2). **[DOC]**

Therefore clamping the rate primitive to ANGLE_MAX (default 30°) is **not a viable
one-line fix**: it would break FLIP and ACRO. The "aerobatic freedom vs. attitude safety"
requirement **cannot be satisfied by a single threshold on the shared primitive** — which
is precisely a **SOTIF specification insufficiency**, not a FuSA implementation bug. The
firmware's own resolution of this tension — the *separate, soft, opt-in*
`ACRO_TRAINER::LIMITED` mechanism (`mode_acro.cpp:149-162`) rather than a hard clamp —
is direct evidence that ArduPilot treats this as a configurable-behavior trade-off, not a
missing guard. **[CODE]/[INFER]**

**I5 result: not-simply-fixable = TRUE.**

---

## I6 — Synthesis, counter-arguments, verdict

**Verdict: BY-DESIGN. Confidence 0.87.**

All four prereg criteria for BY-DESIGN hold:
1. ANGLE_MAX is intentionally scoped to the angle-command path (I1, [CODE]).
2. The rate path is by-design unbound by ANGLE_MAX **and** has its own independent
   governance — `ACRO_RP_RATE` / `ATC_RATE_*_MAX` / accel shaping, plus the opt-in
   `ACRO_TRAINER::LIMITED` (I2, [DOC]/[CODE]).
3. The rate interface (ACRO; and SET_ATTITUDE_TARGET at the protocol/code level) is a
   supported, documented interface (I3, [DOC]/[CODE]) — with the GUIDED-body-rate caveat
   below.
4. Clamping the rate path to ANGLE_MAX would break FLIP/ACRO (I5, [CODE]).

→ The OverDraw contract **genuinely holds on the rate path**: a body-rate-/quaternion-
commanded demanded attitude can legally reach ~177° with every configured limit and
failsafe satisfied. This is a **SOTIF specification-insufficiency**, and it **strengthens
C3**: the gap is *not* reliably patchable by "add one clamp," because the unclamped
primitive is load-bearing for intended aerobatic functionality.

**Counter-arguments considered and rebutted (intellectual-honesty pass):**

- *"The param says 'Maximum lean angle in **all flight modes**' → it was meant to bound
  everything (OVERSIGHT)."* — It governs a **lean angle**, which rate modes do not
  command. The firmware applies it wherever a lean-angle command is generated, across all
  those modes. The author explicitly wrote ANGLE_MAX-limiting for ACRO and made it
  **opt-in** (`ACRO_TRAINER`), and FLIP — an officially documented mode — is impossible
  under a global attitude bound. So "all flight modes" = "the lean-angle command in
  whichever mode produces one," not "the achieved/target attitude in every mode." This is
  the single genuine ambiguity and is the main reason confidence is 0.87 rather than ~0.95.

- *"The GUIDED body-rate field is 'not supported' per the dev doc → the demonstration uses
  undefined behavior, so the contract isn't clean."* — The scope gap is reachable via
  **supported** interfaces (ACRO; and a GUIDED quaternion-attitude target, which
  `input_quaternion` also leaves unclamped). The GUIDED body-rate route being
  doc-discouraged is a reason to **re-anchor the demonstration**, not evidence the scope
  is a bug; the firmware fully implements and acts on those rates (discouraged ≠ rejected/
  undefined). **Recommendation for the paper:** anchor the witness on ACRO (with
  `ACRO_TRAINER=0`, a documented config) or a supported GUIDED quaternion target > ANGLE_MAX.

- *"FlowHold once leaked past ANGLE_MAX and was fixed as a bug (#10524) → maybe the rate
  path is the same."* — #10524 was in the **stabilized lean-angle** path, which *is*
  supposed to honor ANGLE_MAX (wrong argument passed). That maintainers fix ANGLE_MAX
  leaks in angle-command paths, yet have never done so for the rate path, **reinforces**
  that the rate path's freedom is intentional.

**Framing guidance for OverDraw C2/C3:** the most robust statement is the
**non-composition** one — *"ANGLE_MAX (an angle-command-path safety mechanism) by design
does not compose onto the attitude controller's rate and quaternion interfaces; ACRO and
FLIP require those interfaces to be unbounded by ANGLE_MAX, so the gap is a SOTIF
specification insufficiency that cannot be closed by a single clamp without breaking
intended aerobatic functionality."* This holds independently of the supported/unsupported
debate about the GUIDED body-rate field, and is strictly stronger than "ANGLE_MAX is
intentionally limited."

---

## Paper-grade judgment sentences (drop-in, with sources)

1. *"In ArduCopter 4.4.1, `ANGLE_MAX` clamps the **lean-angle command** path — pilot stick
   input via `rc_input_to_roll_pitch` (`libraries/AP_Math/control.cpp:546-561`, called from
   `ArduCopter/mode.cpp:415-431`) and the position controller's accel→lean limit
   (`libraries/AC_AttitudeControl/AC_PosControl.cpp:506-509`) — but is **not** applied to
   the body-rate path `AC_AttitudeControl::input_rate_bf_roll_pitch_yaw`, which integrates
   the rate command directly into the quaternion attitude target with no angle limit
   (`libraries/AC_AttitudeControl/AC_AttitudeControl.cpp:421-455`, esp. lines 444-446)."*

2. *"This scope limitation is by design: ACRO is documented to command angular velocity
   for aerobatics 'such as flips or rolls' with no self-leveling
   (`ardupilot.org/copter/docs/acro-mode.html`); `ANGLE_MAX` reaches ACRO only through the
   opt-in `ACRO_TRAINER=2` feature, with `ACRO_TRAINER=0` documented as 'full Rate control
   with no automatic leveling nor angle-limiting'; and FLIP mode deliberately drives the
   same primitive at ~400°/s through a full inverted rotation
   (`ArduCopter/mode_flip.cpp:118-175`). Clamping the rate primitive to `ANGLE_MAX` would
   break both modes."*

3. *"The rate path is governed by an independent regime (`ACRO_RP_RATE`/`ACRO_Y_RATE`,
   `ArduCopter/Parameters.cpp:1058-1098`; `ATC_RATE_R/P/Y_MAX`,
   `AC_AttitudeControl.cpp:113-141`), not left ungoverned — its governance is on
   *rate/acceleration*, orthogonal to an *angle* limit. ArduPilot's history corroborates
   intent: angle-limiting in ACRO was added as an Enhancement on user request
   (issue #11888, labelled Enhancement, closed), never as a bugfix, and no
   issue/PR/commit ever added an `ANGLE_MAX` clamp to the rate path."*

4. *"Consequently, under all configured limits and failsafe contracts satisfied, a
   rate-/quaternion-commanded **demanded** attitude can legally reach ~177°: the OverDraw
   region is a SOTIF specification-insufficiency on a by-design-unbounded interface, not a
   missing-clamp FuSA bug — and cannot be reliably remediated by a single attitude clamp
   without disabling intended aerobatic capability (C3)."*

5. *(Methodological note for reviewers)* *"Even the documented-supported GUIDED
   attitude-quaternion path does not enforce `ANGLE_MAX` (`input_quaternion`,
   `AC_AttitudeControl.cpp:231-266`, applies only `ang_vel_limit`); the witness is
   therefore not contingent on the body-rate field, which ArduPilot's developer
   documentation marks 'not supported' for GUIDED
   (`ardupilot.org/dev/docs/copter-commands-in-guided-mode.html`)."*

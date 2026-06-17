"""Independent PGFUZZ-style contract/policy checker -- OverDraw claim C2 baseline.

This is the *contract testing baseline* for OverDraw claim C2: contract-clean
unsafe states are structurally invisible to contract testing (PGFUZZ / RVFuzzer
family, which only flag *rule violations*). We run an independently implemented,
documentation-faithful contract/policy checker on the already-produced OverDraw
traces and on real contract-violation positive controls.

DEFENCE LINE 1 (anti-circularity / non-leakage):
    The Tier-1 (contract) checker may read ONLY what a contract tester can see:
    operator commands, modes, documented failsafe trigger conditions, and
    documented policy predicates. It must NOT read any oracle-A *consequence*
    signal (ground-contact flag, altitude-loss, attitude-divergence). This module
    therefore reads only raw decoded telemetry (ATT / MODE / ERR / EV / MSG / POS
    / GUIA / XKF) plus the operator command profile and the parameter snapshot.
    It never opens the *.oracle.json sidecars. The per-policy ``input_fields``
    lists are machine-checked against the oracle-A consequence field set by the
    companion anti-leakage audit.

DEFENCE LINE 2 (anti-strawman):
    Every Tier-1/Tier-2 policy carries a documentation source, and the checker is
    proven non-trivial on real positive controls (it fires when a contract is
    genuinely violated).

TWO TIERS, counted separately:
    Tier-1 (CONTRACT / FuSA): the flight controller's *specified* contracts.
        Expected 0 in the OverDraw region.
    Tier-2 (RESULT / SOTIF safety goals): PGFUZZ-style physical-outcome
        predicates. May fire in the OverDraw region; reported separately. The
        gap (Tier-1 clean AND Tier-2 hit) is exactly the SOTIF/OverDraw seam.

The checker is intentionally implemented from scratch (it does not import the
project oracle) so that "OverDraw region: 0 contract hits" is an *independent*
confirmation, not a restatement of the project's own oracle B.
"""
from __future__ import annotations

import csv
import json
import math
import statistics
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any

try:  # pymavlink only needed for the BIN re-parse path (positive controls)
    from pymavlink import mavutil
except Exception:  # pragma: no cover
    mavutil = None

csv.field_size_limit(10_000_000)

# --------------------------------------------------------------------------- #
# Documented ArduPilot enumerations (public; copied here for independence).
# Sources: ArduCopter/defines.h (ModeReason, LogErrorSubsystem) and the
# parameter / failsafe documentation referenced inline by each policy below.
# --------------------------------------------------------------------------- #
MODE_REASONS = {
    0: "UNKNOWN", 1: "RC_COMMAND", 2: "GCS_COMMAND", 3: "RADIO_FAILSAFE",
    4: "BATTERY_FAILSAFE", 5: "GCS_FAILSAFE", 6: "EKF_FAILSAFE", 7: "GPS_GLITCH",
    8: "MISSION_END", 9: "THROTTLE_LAND_ESCAPE", 10: "FENCE_BREACHED",
    11: "TERRAIN_FAILSAFE", 12: "BRAKE_TIMEOUT", 13: "FLIP_COMPLETE",
    14: "AVOIDANCE", 15: "AVOIDANCE_RECOVERY", 16: "THROW_COMPLETE",
    17: "TERMINATE", 18: "TOY_MODE", 19: "CRASH_FAILSAFE", 25: "FAILSAFE",
    26: "INITIALISED", 29: "LEAK_FAILSAFE", 50: "DEADRECKON_FAILSAFE",
}
ERROR_SUBSYSTEMS = {
    1: "MAIN", 2: "RADIO", 3: "COMPASS", 5: "FAILSAFE_RADIO", 6: "FAILSAFE_BATT",
    7: "FAILSAFE_GPS", 8: "FAILSAFE_GCS", 9: "FAILSAFE_FENCE", 10: "FLIGHT_MODE",
    11: "GPS", 12: "CRASH_CHECK", 13: "FLIP", 16: "EKFCHECK",
    17: "FAILSAFE_EKFINAV", 18: "BARO", 19: "CPU", 20: "FAILSAFE_ADSB",
    21: "TERRAIN", 22: "NAVIGATION", 23: "FAILSAFE_TERRAIN", 24: "EKF_PRIMARY",
    25: "THRUST_LOSS_CHECK", 26: "FAILSAFE_SENSORS", 27: "FAILSAFE_LEAK",
    28: "PILOT_INPUT", 29: "FAILSAFE_VIBE", 30: "INTERNAL_ERROR",
    31: "FAILSAFE_DEADRECKON",
}
COPTER_MODES = {
    0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD", 3: "AUTO", 4: "GUIDED", 5: "LOITER",
    6: "RTL", 7: "CIRCLE", 9: "LAND", 11: "DRIFT", 13: "SPORT", 14: "FLIP",
    15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE", 18: "THROW", 19: "AVOID_ADSB",
    20: "GUIDED_NOGPS", 21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
    24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE", 27: "AUTO_RTL",
}

# Harness-only mode reasons -- excluded from Tier-1 per OverDraw discipline
# (artifact #2: GCS_COMMAND arm/takeoff/mode-set/cleanup is harness, not a
# flight-controller decision). INITIALISED is power-on, RC_COMMAND is operator
# stick. These are *operational*, never contract violations.
OPERATIONAL_MODE_REASONS = {"INITIALISED", "GCS_COMMAND", "RC_COMMAND", "UNKNOWN",
                            "MISSION_END"}

# Preventive flight-controller failsafe reasons (a mode transition driven by one
# of these is a *failsafe-induced* transition). CRASH_FAILSAFE is deliberately
# NOT here: crash-check is a post-impact *consequence detector*, not a preventive
# guard, so it is scored under Tier-2 (see CRASH_DETECTOR_*).
PREVENTIVE_MODE_REASONS = {
    "RADIO_FAILSAFE", "BATTERY_FAILSAFE", "GCS_FAILSAFE", "EKF_FAILSAFE",
    "GPS_GLITCH", "FENCE_BREACHED", "TERRAIN_FAILSAFE", "AVOIDANCE", "TERMINATE",
    "FAILSAFE", "LEAK_FAILSAFE", "DEADRECKON_FAILSAFE",
}
# Preventive failsafe ERR subsystems. CRASH_CHECK excluded (consequence detector).
PREVENTIVE_FAILSAFE_SUBSYSTEMS = {
    "FAILSAFE_RADIO", "FAILSAFE_BATT", "FAILSAFE_GPS", "FAILSAFE_GCS",
    "FAILSAFE_FENCE", "EKFCHECK", "FAILSAFE_EKFINAV", "FAILSAFE_ADSB",
    "FAILSAFE_TERRAIN", "THRUST_LOSS_CHECK", "FAILSAFE_SENSORS", "FAILSAFE_LEAK",
    "FAILSAFE_VIBE", "FAILSAFE_DEADRECKON",
}
# Failsafe STATUSTEXT markers (lower-case substring match). "sim hit ground" and
# "crash" markers are deliberately absent -- those are consequence signals (T2c).
PREVENTIVE_TEXT_MARKERS = ("failsafe", "ekf failsafe", "thrust loss", "deadreckon",
                           "fence breach")
# Crash-check / crash markers -> Tier-2 consequence detector (NOT Tier-1).
CRASH_DETECTOR_SUBSYSTEMS = {"CRASH_CHECK"}
CRASH_DETECTOR_MODE_REASONS = {"CRASH_FAILSAFE"}
GROUND_CONTACT_TEXT = ("sim hit ground", "crash")
# Safety / autoland modes a failsafe would force the vehicle into.
SAFETY_MODES = {"RTL", "LAND", "SMART_RTL", "AUTO_RTL", "BRAKE", "AVOID_ADSB",
                "AUTOROTATE"}

# Documented thresholds (pre-registered; mirror oracleA_v1_config.yaml `oracle`).
ANGLE_MAX_TOLERANCE_DEG = 0.1        # command-amplitude clamp tolerance
DIVERGENCE_ERROR_DEG = 60.0          # attitude-error divergence threshold
DIVERGENCE_DURATION_S = 0.5
ALTITUDE_LOSS_THRESH_M = 15.0        # uncommanded altitude loss
H_FLOOR_M = 1.0                      # ground floor
FLOOR_DESCENT_RATE_M_S = 2.0
RATE_ACTIVE_THRESHOLD_DEG_S = 5.0    # window: maneuver active when |rate|>=this
ATT_ACTIVE_THRESHOLD_DEG = 2.0
OBSERVATION_AFTER_PROFILE_S = 18.0


# --------------------------------------------------------------------------- #
# Small numeric helpers (independent copies; behaviour mirrors the harness).
# --------------------------------------------------------------------------- #
def _f(v: Any, default: float = 0.0) -> float:
    try:
        if v is None or v == "":
            return default
        return float(v)
    except Exception:
        return default


def percentile(values: list[float], pct: float) -> float | None:
    clean = sorted(v for v in values if math.isfinite(v))
    if not clean:
        return None
    if len(clean) == 1:
        return clean[0]
    rank = (len(clean) - 1) * pct
    lo, hi = math.floor(rank), math.ceil(rank)
    if lo == hi:
        return clean[lo]
    return clean[lo] + (clean[hi] - clean[lo]) * (rank - lo)


def sustained_exceed(rows: list[tuple[float, float]], threshold: float,
                     duration_s: float) -> dict[str, Any]:
    over_start = None
    previous_t = None
    best = 0.0
    intervals: list[dict[str, float]] = []
    for t_s, value in rows:
        if value > threshold:
            if over_start is None:
                over_start = t_s
        elif over_start is not None:
            end_t = previous_t if previous_t is not None else t_s
            elapsed = max(0.0, end_t - over_start)
            best = max(best, elapsed)
            if elapsed >= duration_s:
                intervals.append({"start_s": over_start, "end_s": end_t})
            over_start = None
        previous_t = t_s
    if over_start is not None and previous_t is not None:
        elapsed = max(0.0, previous_t - over_start)
        best = max(best, elapsed)
        if elapsed >= duration_s:
            intervals.append({"start_s": over_start, "end_s": previous_t})
    return {"ok": bool(intervals), "max_duration_s": best, "intervals": intervals}


def nearest_value(rows: list[tuple[float, float]], t_s: float | None) -> float | None:
    if not rows or t_s is None:
        return None
    return min(rows, key=lambda item: abs(item[0] - t_s))[1]


def command_at(profile: list[dict[str, float]], rel_t: float) -> dict[str, float]:
    if not profile:
        return {"roll_deg": 0.0, "pitch_deg": 0.0}
    best = min(profile, key=lambda s: abs(_f(s.get("t_s")) - rel_t))
    return best


# --------------------------------------------------------------------------- #
# Normalised run data -- the only thing the policy evaluator sees.
# --------------------------------------------------------------------------- #
class RunData:
    def __init__(self, run_id: str, source: str) -> None:
        self.run_id = run_id
        self.source = source                       # 'csv' | 'bin'
        self.params: dict[str, float] = {}
        self.angle_max_deg: float | None = None
        self.command_profile: list[dict[str, float]] | None = None
        self.att: list[dict[str, float]] = []      # t, des_roll, des_pitch, roll, pitch
        self.pos: list[dict[str, float]] = []      # t, rel_home_alt
        self.xkf1_vd: list[tuple[float, float]] = []   # t, VD (down velocity)
        self.xkf4_fs: list[tuple[float, float]] = []   # t, FS  (AUDIT ONLY; not used by Tier-1)
        self.modes: list[dict[str, Any]] = []      # t, mode_name, reason_name
        self.errs: list[dict[str, Any]] = []       # t, subsystem_name, ecode
        self.msgs: list[dict[str, Any]] = []       # t, text
        self.guia_active_t: list[float] = []       # times the rate command is active

    # --- maneuver window, derived from INPUT timing only (no consequence) --- #
    def window(self) -> tuple[float | None, float | None]:
        start = min(self.guia_active_t) if self.guia_active_t else None
        if start is None and self.command_profile:
            # fall back to attitude-active onset
            active = [r["t"] for r in self.att
                      if math.hypot(r.get("des_roll", 0.0), r.get("des_pitch", 0.0))
                      >= ATT_ACTIVE_THRESHOLD_DEG]
            start = min(active) if active else None
        if start is None:
            ts = [r["t"] for r in self.att] or [r["t"] for r in self.pos]
            return (min(ts), max(ts)) if ts else (None, None)
        prof_dur = _f(self.command_profile[-1].get("t_s")) if self.command_profile else 0.0
        end = start + prof_dur + OBSERVATION_AFTER_PROFILE_S
        return (start, end)


# --------------------------------------------------------------------------- #
# Loaders
# --------------------------------------------------------------------------- #
def _params_snapshot(params_path: Path) -> dict[str, float]:
    if not params_path.exists():
        return {}
    try:
        raw = json.loads(params_path.read_text())
    except Exception:
        return {}
    if isinstance(raw, dict) and "snapshot" in raw and isinstance(raw["snapshot"], dict):
        return {k: _f(v) for k, v in raw["snapshot"].items()}
    if isinstance(raw, dict):
        return {k: _f(v) for k, v in raw.items() if isinstance(v, (int, float))}
    return {}


def load_run_from_csv(stem: Path) -> RunData:
    """Load an OverDraw run from its raw decoded-message CSV dump.

    Reads ONLY raw telemetry + command profile + param snapshot. Never opens
    the *.oracle.json sidecar.
    """
    run = RunData(stem.name, "csv")
    run.params = _params_snapshot(stem.parent / f"{stem.name}_params.json")
    if "ANGLE_MAX" in run.params:
        run.angle_max_deg = run.params["ANGLE_MAX"] / 100.0

    prof_path = stem.parent / f"{stem.name}_command_profile.json"
    if prof_path.exists():
        prof = json.loads(prof_path.read_text())
        samples = prof.get("samples") if isinstance(prof, dict) else None
        if samples:
            run.command_profile = samples

    csv_path = stem.parent / f"{stem.name}_parsed.csv"
    with csv_path.open() as fh:
        for row in csv.DictReader(fh):
            _ingest_row(run, row)
    return run


def _ingest_row(run: RunData, row: dict[str, str]) -> None:
    typ = row.get("type")
    t = _f(row.get("time_s"), math.nan)
    if not math.isfinite(t):
        return
    if typ == "ATT":
        run.att.append({"t": t,
                        "des_roll": _f(row.get("DesRoll")), "des_pitch": _f(row.get("DesPitch")),
                        "roll": _f(row.get("Roll")), "pitch": _f(row.get("Pitch"))})
    elif typ == "POS":
        rh = row.get("RelHomeAlt")
        if rh not in (None, ""):
            run.pos.append({"t": t, "rel_home_alt": _f(rh)})
    elif typ == "GUIA":
        if abs(_f(row.get("RollRt"))) >= RATE_ACTIVE_THRESHOLD_DEG_S or \
           abs(_f(row.get("PitchRt"))) >= RATE_ACTIVE_THRESHOLD_DEG_S:
            run.guia_active_t.append(t)
    elif typ == "XKF1":
        if int(_f(row.get("C"), 0)) == 0 and row.get("VD") not in (None, ""):
            run.xkf1_vd.append((t, _f(row.get("VD"))))
    elif typ == "XKF4":
        if int(_f(row.get("C"), 0)) == 0 and row.get("FS") not in (None, ""):
            run.xkf4_fs.append((t, _f(row.get("FS"))))   # AUDIT ONLY
    elif typ == "MODE":
        run.modes.append({"t": t,
                          "mode_name": row.get("mode_name") or COPTER_MODES.get(int(_f(row.get("ModeNum", row.get("Mode")), -1)), "?"),
                          "reason_name": row.get("reason_name") or MODE_REASONS.get(int(_f(row.get("Rsn"), -1)), "?")})
    elif typ == "ERR":
        run.errs.append({"t": t,
                         "subsystem_name": row.get("subsystem_name") or ERROR_SUBSYSTEMS.get(int(_f(row.get("Subsys"), -1)), "?"),
                         "ecode": int(_f(row.get("ECode"), 0))})
    elif typ in ("MSG", "STAT"):
        run.msgs.append({"t": t, "text": str(row.get("Message") or row.get("Msg") or "")})


def load_run_from_bin(bin_path: Path, angle_max_deg: float | None = None,
                      home_alt_m: float | None = None) -> RunData:
    """Re-parse a DataFlash .BIN directly (used for legacy positive-control logs
    whose old CSVs lack the attitude/mode/err streams). Produces the identical
    normalised structure, so the SAME policy evaluator runs on it."""
    if mavutil is None:
        raise RuntimeError("pymavlink not available for BIN parse")
    run = RunData(bin_path.stem, "bin")
    run.angle_max_deg = angle_max_deg
    types = ["ATT", "GUIA", "MODE", "ERR", "MSG", "POS", "XKF1", "XKF4", "PARM"]
    mlog = mavutil.mavlink_connection(str(bin_path), robust_parsing=True)
    start_t = None
    while True:
        msg = mlog.recv_match(type=types, blocking=False)
        if msg is None:
            break
        if msg.get_type() == "BAD_DATA":
            continue
        d = msg.to_dict()
        ts = d.get("TimeUS")
        t = float(ts) / 1e6 if ts is not None else None
        if t is None:
            continue
        if start_t is None:
            start_t = t
        rel = t - start_t
        typ = msg.get_type()
        if typ == "PARM":
            if d.get("Name") == "ANGLE_MAX" and run.angle_max_deg is None:
                run.angle_max_deg = _f(d.get("Value")) / 100.0
            run.params[str(d.get("Name"))] = _f(d.get("Value"))
            continue
        # map into the same shape _ingest_row produces
        if typ == "ATT":
            run.att.append({"t": rel, "des_roll": _f(d.get("DesRoll")), "des_pitch": _f(d.get("DesPitch")),
                            "roll": _f(d.get("Roll")), "pitch": _f(d.get("Pitch"))})
        elif typ == "POS":
            rh = d.get("RelHomeAlt")
            if rh is not None:
                run.pos.append({"t": rel, "rel_home_alt": _f(rh)})
        elif typ == "GUIA":
            if abs(_f(d.get("RollRt"))) >= RATE_ACTIVE_THRESHOLD_DEG_S or \
               abs(_f(d.get("PitchRt"))) >= RATE_ACTIVE_THRESHOLD_DEG_S:
                run.guia_active_t.append(rel)
        elif typ == "XKF1" and int(_f(d.get("C"), 0)) == 0 and d.get("VD") is not None:
            run.xkf1_vd.append((rel, _f(d.get("VD"))))
        elif typ == "XKF4" and int(_f(d.get("C"), 0)) == 0 and d.get("FS") is not None:
            run.xkf4_fs.append((rel, _f(d.get("FS"))))
        elif typ == "MODE":
            run.modes.append({"t": rel, "mode_name": COPTER_MODES.get(int(_f(d.get("ModeNum", d.get("Mode")), -1)), "?"),
                              "reason_name": MODE_REASONS.get(int(_f(d.get("Rsn"), -1)), str(d.get("Rsn")))})
        elif typ == "ERR":
            run.errs.append({"t": rel, "subsystem_name": ERROR_SUBSYSTEMS.get(int(_f(d.get("Subsys"), -1)), str(d.get("Subsys"))),
                             "ecode": int(_f(d.get("ECode"), 0))})
        elif typ == "MSG":
            run.msgs.append({"t": rel, "text": str(d.get("Message") or "")})
    return run


# --------------------------------------------------------------------------- #
# POLICY REGISTRY -- every policy carries tier, doc source, and the exact list
# of raw input fields it reads (machine-checked by the anti-leakage audit).
# --------------------------------------------------------------------------- #
POLICY_META = {
    # ---------------- Tier-1: CONTRACT (FuSA) -- expect 0 in OverDraw -------- #
    "T1a_angle_max_command_clamp": {
        "tier": 1,
        "name": "Commanded lean angle within ANGLE_MAX",
        "doc_source": "ArduPilot ANGLE_MAX parameter (ArduCopter/Parameters.cpp; "
                      "'maximum lean angle in all flight modes', legal range "
                      "1000-8000 cd). Contract: pilot/navigation lean-angle COMMANDS "
                      "are clamped to ANGLE_MAX.",
        "violation": "Operator-commanded lean amplitude exceeds ANGLE_MAX+tol.",
        "input_fields": ["command_profile.roll_deg", "command_profile.pitch_deg",
                         "params.ANGLE_MAX"],
        "scope_note": "ANGLE_MAX governs lean-angle COMMANDS, not SET_ATTITUDE_TARGET "
                      "quaternion targets, body-rate setpoints, or ACRO trainer-off "
                      "rate control; demanded attitude (ATT.DesRoll/DesPitch) "
                      "excursions on those interfaces are out of Tier-1 scope and "
                      "recorded under Tier-2 (T2a), not here.",
    },
    "T1b_no_preventive_failsafe": {
        "tier": 1,
        "name": "No spurious preventive failsafe trigger",
        "doc_source": "ArduPilot failsafe docs (radio/battery/GCS/EKF/GPS/fence/"
                      "terrain/leak/deadreckon). Contract: a preventive failsafe "
                      "actuates only when its documented condition holds; given the "
                      "fixed P (batteries/RC/GCS/fence disabled) none should fire.",
        "violation": "A preventive failsafe ERR subsystem, failsafe MODE reason, "
                     "or failsafe STATUSTEXT occurs (GCS_COMMAND harness excluded; "
                     "crash-check excluded as a Tier-2 consequence detector).",
        "input_fields": ["ERR.subsystem_name", "ERR.ecode", "MODE.reason_name",
                         "MSG.text(failsafe markers)"],
    },
    "T1c_configured_limit_compliance": {
        "tier": 1,
        "name": "Configured envelope/limit compliance (geofence, EKF action)",
        "doc_source": "ArduPilot FENCE_* and FS_EKF_ACTION parameter docs. Contract: "
                      "a configured-and-enabled limit is honoured; a DISABLED limit "
                      "never actuates (no false positive); the EKF failsafe action "
                      "does not actuate spuriously.",
        "violation": "A geofence breach event (FENCE_BREACHED / FAILSAFE_FENCE), or "
                     "an EKF-failsafe actuation (EKFCHECK / FAILSAFE_EKFINAV / "
                     "EKF_FAILSAFE), occurs.",
        "input_fields": ["ERR.subsystem_name", "ERR.ecode", "MODE.reason_name",
                         "params.FENCE_ENABLE"],
        "scope_note": "Uses the failsafe ACTUATION signal only. The raw EKF "
                      "filter-fault bitmask XKF4.FS is deliberately EXCLUDED -- it "
                      "is a filter-internal health flag that correlates with "
                      "aggressive/crash dynamics (a leakage trap), not the "
                      "documented EKF-failsafe contract.",
    },
    "T1d_mode_transition_legitimacy": {
        "tier": 1,
        "name": "Mode transitions conform to spec",
        "doc_source": "ArduCopter flight-mode docs + ModeReason enum "
                      "(libraries/AP_Vehicle/ModeReason.h in ArduPilot 4.4.1). Contract: every mode transition has a "
                      "documented operational reason (power-on, GCS/RC command) or a "
                      "valid failsafe reason; no spurious safety-mode transition.",
        "violation": "A transition into a safety/autoland mode (RTL/LAND/BRAKE/...) "
                     "or any transition with a preventive failsafe reason "
                     "(GCS_COMMAND harness cleanup excluded).",
        "input_fields": ["MODE.mode_name", "MODE.reason_name"],
    },
    # ---------------- Tier-2: RESULT (SOTIF safety goals) ------------------- #
    "T2a_attitude_within_bound": {
        "tier": 2,
        "name": "Attitude stays within physical safety bound",
        "doc_source": "PGFUZZ-style physical-state policy ('attitude must stay "
                      "within limits'). SAFETY GOAL, not a contract.",
        "violation": "Sustained achieved-attitude error > %.0f deg for >= %.1f s "
                     "(includes rate-commanded excursions beyond the ANGLE_MAX "
                     "envelope)." % (DIVERGENCE_ERROR_DEG, DIVERGENCE_DURATION_S),
        "input_fields": ["ATT.Roll", "ATT.Pitch", "command_profile.roll_deg",
                         "command_profile.pitch_deg"],
        "consequence": True,
    },
    "T2b_no_uncommanded_altitude_loss": {
        "tier": 2,
        "name": "No large uncommanded altitude loss",
        "doc_source": "PGFUZZ-style 'altitude should be maintained' policy. SAFETY "
                      "GOAL, not a contract.",
        "violation": "RelHomeAlt drops > %.0f m within the maneuver window." % ALTITUDE_LOSS_THRESH_M,
        "input_fields": ["POS.RelHomeAlt"],
        "consequence": True,
    },
    "T2c_no_ground_contact": {
        "tier": 2,
        "name": "No ground contact / crash",
        "doc_source": "PGFUZZ-style 'the drone should not crash' policy. SAFETY "
                      "GOAL, not a contract.",
        "violation": "Ground-contact STATUSTEXT ('SIM Hit ground'), low-floor "
                     "descent (RelHomeAlt<%.0fm & down-speed>%.0f m/s), or a "
                     "crash-check actuation." % (H_FLOOR_M, FLOOR_DESCENT_RATE_M_S),
        "input_fields": ["MSG.text(ground markers)", "POS.RelHomeAlt", "XKF1.VD",
                         "ERR.subsystem_name(CRASH_CHECK)", "MODE.reason_name(CRASH_FAILSAFE)"],
        "consequence": True,
    },
}

# The oracle-A *consequence* field set (defence-line-1 reference): the raw
# signals that DISCRIMINATE "unsafe" from "safe" in hardened oracle A. The
# anti-leakage audit requires Tier-1 input_fields to be DISJOINT from this set.
#
# Note on the operator command profile: command_profile.roll_deg/pitch_deg is the
# pre-generated OPERATOR INPUT (deterministic from r and ANGLE_MAX, written before
# flight) -- it is what a contract tester sees and mutates, NOT a consequence. It
# is therefore NOT in this set even though oracle A uses it as the *baseline* for
# the attitude-error divergence test. The discriminating consequence is the
# ACHIEVED attitude (ATT.Roll/ATT.Pitch), which no Tier-1 policy reads.
ORACLE_A_CONSEQUENCE_FIELDS = {
    "ATT.Roll", "ATT.Pitch",                       # achieved attitude / divergence
    "POS.RelHomeAlt",                              # altitude loss / floor
    "XKF1.VD",                                     # floor descent rate
    "MSG.text(ground markers)",                    # 'SIM Hit ground'
    "ERR.subsystem_name(CRASH_CHECK)",             # crash detector
    "MODE.reason_name(CRASH_FAILSAFE)",            # crash detector
    "XKF4.FS",                                     # raw EKF fault bitmask (leakage trap)
}


# --------------------------------------------------------------------------- #
# Policy evaluators -- each returns {hit, detail, evidence}
# --------------------------------------------------------------------------- #
def _eval_T1a(run: RunData) -> dict[str, Any]:
    if run.command_profile is None or run.angle_max_deg is None:
        return {"hit": False, "applicable": False, "detail": "no command profile / ANGLE_MAX"}
    amax = run.angle_max_deg
    angles = [math.hypot(_f(s.get("roll_deg")), _f(s.get("pitch_deg"))) for s in run.command_profile]
    max_cmd = max(angles, default=0.0)
    hit = max_cmd > amax + ANGLE_MAX_TOLERANCE_DEG
    return {"hit": hit, "applicable": True,
            "detail": f"max commanded lean {max_cmd:.2f} deg vs ANGLE_MAX {amax:.1f} deg",
            "max_command_angle_deg": max_cmd, "angle_max_deg": amax}


def _failsafe_events(run: RunData) -> list[dict[str, Any]]:
    out = []
    for e in run.errs:
        if e["ecode"] != 0 and e["subsystem_name"] in PREVENTIVE_FAILSAFE_SUBSYSTEMS:
            out.append({"t": e["t"], "kind": "ERR", "subsystem": e["subsystem_name"], "ecode": e["ecode"]})
    for m in run.modes:
        if m["reason_name"] in PREVENTIVE_MODE_REASONS:
            out.append({"t": m["t"], "kind": "MODE", "reason": m["reason_name"], "mode": m["mode_name"]})
    for s in run.msgs:
        low = s["text"].lower()
        if any(mk in low for mk in PREVENTIVE_TEXT_MARKERS):
            out.append({"t": s["t"], "kind": "MSG", "text": s["text"]})
    return sorted(out, key=lambda x: x["t"])


def _eval_T1b(run: RunData) -> dict[str, Any]:
    ev = _failsafe_events(run)
    return {"hit": bool(ev), "applicable": True,
            "detail": f"{len(ev)} preventive failsafe event(s)",
            "events": ev[:10]}


def _eval_T1c(run: RunData) -> dict[str, Any]:
    ev = []
    for e in run.errs:
        if e["ecode"] != 0 and e["subsystem_name"] in {"FAILSAFE_FENCE", "EKFCHECK", "FAILSAFE_EKFINAV"}:
            ev.append({"t": e["t"], "kind": "ERR", "subsystem": e["subsystem_name"], "ecode": e["ecode"]})
    for m in run.modes:
        if m["reason_name"] in {"FENCE_BREACHED", "EKF_FAILSAFE"}:
            ev.append({"t": m["t"], "kind": "MODE", "reason": m["reason_name"]})
    return {"hit": bool(ev), "applicable": True,
            "detail": f"{len(ev)} configured-limit event(s)", "events": ev[:10],
            "fence_enabled": bool(run.params.get("FENCE_ENABLE", 0.0))}


def _eval_T1d(run: RunData) -> dict[str, Any]:
    ev = []
    for m in run.modes:
        reason = m["reason_name"]
        mode = m["mode_name"]
        if reason in OPERATIONAL_MODE_REASONS:
            continue
        # crash-check induced LAND is a consequence detector -> Tier-2
        if reason in CRASH_DETECTOR_MODE_REASONS:
            continue
        if reason in PREVENTIVE_MODE_REASONS or mode in SAFETY_MODES:
            ev.append({"t": m["t"], "mode": mode, "reason": reason})
    return {"hit": bool(ev), "applicable": True,
            "detail": f"{len(ev)} illegitimate/failsafe mode transition(s)", "events": ev[:10]}


def _att_error_series(run: RunData, lo: float | None, hi: float | None,
                      start: float | None) -> list[tuple[float, float]]:
    out = []
    for r in run.att:
        t = r["t"]
        if lo is not None and not (lo <= t <= hi):
            continue
        if run.command_profile is not None and start is not None:
            cmd = command_at(run.command_profile, t - start)
            err = math.hypot(r["roll"] - _f(cmd.get("roll_deg")), r["pitch"] - _f(cmd.get("pitch_deg")))
        else:
            err = math.hypot(r["roll"], r["pitch"])     # absolute lean when no command ref
        out.append((t, err))
    return out


def _eval_T2a(run: RunData) -> dict[str, Any]:
    lo, hi = run.window()
    errs = _att_error_series(run, lo, hi, lo)
    div = sustained_exceed(errs, DIVERGENCE_ERROR_DEG, DIVERGENCE_DURATION_S)
    max_err = max((v for _, v in errs), default=None)
    return {"hit": bool(div["ok"]), "applicable": bool(errs),
            "detail": f"max attitude error {max_err:.1f} deg" if max_err is not None else "no att",
            "max_attitude_error_deg": max_err}


def _eval_T2b(run: RunData) -> dict[str, Any]:
    lo, hi = run.window()
    alts = [(r["t"], r["rel_home_alt"]) for r in run.pos if (lo is None or lo <= r["t"] <= hi)]
    if not alts:
        return {"hit": False, "applicable": False, "detail": "no POS.RelHomeAlt"}
    start_alt = nearest_value(alts, lo) if lo is not None else alts[0][1]
    min_alt = min(v for _, v in alts)
    loss = (start_alt - min_alt) if start_alt is not None else None
    hit = loss is not None and loss > ALTITUDE_LOSS_THRESH_M
    return {"hit": bool(hit), "applicable": True,
            "detail": f"altitude loss {loss:.1f} m" if loss is not None else "n/a",
            "altitude_loss_m": loss}


def _eval_T2c(run: RunData) -> dict[str, Any]:
    lo, hi = run.window()
    ground = [m for m in run.msgs if any(g in m["text"].lower() for g in GROUND_CONTACT_TEXT)
              and (lo is None or lo <= m["t"] <= hi)]
    floor = []
    for r in run.pos:
        if lo is not None and not (lo <= r["t"] <= hi):
            continue
        vd = nearest_value(run.xkf1_vd, r["t"])
        if r["rel_home_alt"] < H_FLOOR_M and vd is not None and vd > FLOOR_DESCENT_RATE_M_S:
            floor.append({"t": r["t"], "alt": r["rel_home_alt"], "vd": vd})
            break
    crash_check = [e for e in run.errs if e["subsystem_name"] in CRASH_DETECTOR_SUBSYSTEMS and e["ecode"] != 0]
    crash_check += [m for m in run.modes if m["reason_name"] in CRASH_DETECTOR_MODE_REASONS]
    hit = bool(ground or floor or crash_check)
    return {"hit": hit, "applicable": True,
            "detail": f"{len(ground)} ground msgs, {len(floor)} floor markers, {len(crash_check)} crash-check",
            "ground_messages": [g["text"] for g in ground[:3]]}


EVALUATORS = {
    "T1a_angle_max_command_clamp": _eval_T1a,
    "T1b_no_preventive_failsafe": _eval_T1b,
    "T1c_configured_limit_compliance": _eval_T1c,
    "T1d_mode_transition_legitimacy": _eval_T1d,
    "T2a_attitude_within_bound": _eval_T2a,
    "T2b_no_uncommanded_altitude_loss": _eval_T2b,
    "T2c_no_ground_contact": _eval_T2c,
}


# Interfaces on which T1a (ANGLE_MAX lean-angle command clamp) is, BY DESIGN,
# not a contract: SET_ATTITUDE_TARGET quaternion-attitude / body-rate, and ACRO.
# Source: angle_max_scope_v1 (BY-DESIGN 0.87) -- ANGLE_MAX governs the lean-angle
# command path only; input_quaternion (AC_AttitudeControl.cpp:231-266) and the
# rate path are deliberately unbounded by ANGLE_MAX. On these interfaces the
# substance of T1a (attitude beyond the ANGLE_MAX envelope) is scored under
# Tier-2 (T2a), never Tier-1 -- this is what stops the checker from circularly
# branding an intended overdraw as a "contract violation".
ATTITUDE_TARGET_INTERFACES = {"quaternion", "guided_quaternion", "rate", "guided_rate", "acro", "acro_rate"}
T1A_POLICY = "T1a_angle_max_command_clamp"


def check_run(run: RunData, interface: str = "lean_angle_command") -> dict[str, Any]:
    policies = {pid: EVALUATORS[pid](run) for pid in POLICY_META}
    # interface-aware scoping of T1a (see ATTITUDE_TARGET_INTERFACES)
    t1a_in_scope = interface not in ATTITUDE_TARGET_INTERFACES
    if not t1a_in_scope and T1A_POLICY in policies:
        policies[T1A_POLICY] = dict(policies[T1A_POLICY])
        policies[T1A_POLICY]["applicable"] = False
        policies[T1A_POLICY]["hit"] = False
        policies[T1A_POLICY]["interface_scope_note"] = (
            f"T1a not applicable on interface '{interface}': ANGLE_MAX is a "
            "lean-angle-command contract and by design does not govern the "
            "attitude-target interface (angle_max_scope_v1); scored under Tier-2 T2a."
        )
    tier1 = {pid: r for pid, r in policies.items() if POLICY_META[pid]["tier"] == 1}
    tier2 = {pid: r for pid, r in policies.items() if POLICY_META[pid]["tier"] == 2}
    tier1_hits = [pid for pid, r in tier1.items() if r.get("hit")]
    tier2_hits = [pid for pid, r in tier2.items() if r.get("hit")]
    return {
        "run_id": run.run_id,
        "source": run.source,
        "interface": interface,
        "t1a_in_scope": t1a_in_scope,
        "angle_max_deg": run.angle_max_deg,
        "tier1_any": bool(tier1_hits),
        "tier2_any": bool(tier2_hits),
        "tier1_hit_policies": tier1_hits,
        "tier2_hit_policies": tier2_hits,
        "policies": policies,
    }

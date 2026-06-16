"""PC_CLAMP positive control for the contract-baseline (claim C2, section 4.3).

Runs a small ArduCopter SITL batch through the *same* oracleA_v1 harness, but
with the command-amplitude guard removed so the operator-commanded lean
amplitude EXCEEDS ANGLE_MAX. This produces a real same-stack trace that
genuinely violates the T1a (ANGLE_MAX command-clamp) contract, proving the
contract checker's T1a predicate is non-trivial.

This is the ONLY new SITL the contract baseline runs (sanctioned by the task:
the positive control may run a little new SITL). The OverDraw set itself is
analysed entirely from existing logs.

Usage:
    python3 planc/src/run_contract_poscontrol_v1.py [--r 300] [--seeds 0 1]
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

THIS = Path(__file__).resolve()
SRC_ROOT = THIS.parent
PLANC_ROOT = THIS.parents[1]
sys.path.insert(0, str(SRC_ROOT))

import run_oracleA_v1 as oa  # noqa: E402


def build_config() -> dict:
    cfg = oa.load_yaml(PLANC_ROOT / "config" / "oracleA_v1_config.yaml")
    cfg["experiment"]["run_prefix"] = "pcclampv1"
    cfg["experiment"]["artifact_stem"] = "contract_pc_clamp_v1"
    # Remove the amplitude guard and push the commanded amplitude past ANGLE_MAX.
    # command_amplitude_deg = min(max_nominal, ANGLE_MAX*frac, ANGLE_MAX - guard).
    # With guard negative and frac>1 the commanded lean exceeds ANGLE_MAX.
    cfg["command"]["amplitude_fraction_of_angle_max"] = 1.3
    cfg["command"]["max_nominal_amplitude_deg"] = 95.0
    cfg["oracle"]["command_angle_guard_deg"] = -25.0
    return cfg


def make_point(r: float, seed: int) -> dict:
    return {
        "role": "pcclamp",
        "phase": "poscontrol",
        "layer": "default",
        "r_deg_s": float(r),
        "seed": int(seed),
        "wind_m_s": 0.0,
        "turbulence_m_s": 0.0,
        "angle_max_cd": 4500.0,
        "model": "m100",
    }


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--r", type=float, default=300.0)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    args = ap.parse_args()

    cfg = build_config()
    out_path = PLANC_ROOT / "results" / "contract_pc_clamp_v1.json"
    runs = []
    for seed in args.seeds:
        point = make_point(args.r, seed)
        run_id = oa.run_id_for(cfg, point)
        print(f"[pc_clamp] running {run_id} ...", flush=True)
        res = oa.run_one(cfg, point)
        summary = {
            "run_id": run_id,
            "error": res.get("error"),
            "bin_path": res.get("bin_path"),
            "command_max_angle_deg": (res.get("command") or {}).get("max_command_angle_deg"),
            "angle_max_deg": (res.get("command") or {}).get("angle_max_deg"),
            "command_exceeded_angle_max": (res.get("command") or {}).get("command_exceeded_angle_max"),
            "label": res.get("label"),
        }
        print(f"[pc_clamp]   -> {json.dumps(summary)}", flush=True)
        runs.append(summary)
        out_path.write_text(json.dumps({"runs": runs}, indent=2) + "\n")
    print(f"[pc_clamp] wrote {out_path}", flush=True)


if __name__ == "__main__":
    main()

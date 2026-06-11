from __future__ import annotations

import json
import math
import time
from pathlib import Path
from typing import Any

from pymavlink import mavutil


class ParamError(RuntimeError):
    pass


def _clean_param_id(raw: Any) -> str:
    if isinstance(raw, bytes):
        raw = raw.decode("ascii", errors="ignore")
    return str(raw).strip("\x00")


class ParamManager:
    def __init__(self, master, tolerance: float = 1.0e-4):
        self.master = master
        self.tolerance = tolerance
        self.records: list[dict[str, Any]] = []

    def read(self, name: str, timeout_s: float = 5.0) -> float:
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("ascii"),
            -1,
        )
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if _clean_param_id(msg.param_id) == name:
                return float(msg.param_value)
        raise ParamError(f"Timed out reading parameter {name}")

    def set_and_readback(self, name: str, value: float, timeout_s: float = 8.0) -> dict[str, Any]:
        before = None
        try:
            before = self.read(name, timeout_s=3.0)
        except Exception:
            before = None
        self.master.mav.param_set_send(
            self.master.target_system,
            self.master.target_component,
            name.encode("ascii"),
            float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
        )
        deadline = time.time() + timeout_s
        readback = None
        while time.time() < deadline:
            msg = self.master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if _clean_param_id(msg.param_id) == name:
                readback = float(msg.param_value)
                break
        if readback is None:
            readback = self.read(name, timeout_s=3.0)
        ok = math.isclose(float(value), float(readback), rel_tol=0.0, abs_tol=self.tolerance)
        record = {
            "name": name,
            "requested": float(value),
            "before": before,
            "readback": readback,
            "ok": ok,
        }
        self.records.append(record)
        if not ok:
            raise ParamError(f"Parameter {name} readback mismatch: requested {value}, got {readback}")
        return record

    def apply(self, params: dict[str, Any]) -> list[dict[str, Any]]:
        for name, value in params.items():
            self.set_and_readback(name, float(value))
        return self.records

    def snapshot(self, names: list[str]) -> dict[str, float | None]:
        out: dict[str, float | None] = {}
        for name in names:
            try:
                out[name] = self.read(name, timeout_s=3.0)
            except Exception:
                out[name] = None
        return out

    def write_records(self, path: Path, snapshot: dict[str, Any] | None = None) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        payload = {"records": self.records, "snapshot": snapshot or {}}
        path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

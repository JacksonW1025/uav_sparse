from __future__ import annotations

import json
import math
from pathlib import Path

import pandas as pd
from pymavlink import mavutil

from cadet.config import ScenarioCfg
from cadet.vehicle.base import VehicleAdapter
from cadet.vehicle.mavlink_common import (
    MavlinkVehicleMixin,
    copy_latest_matching,
    kill_process_patterns,
    start_process,
)

PX4_PARAM_TYPE_BY_METADATA = {
    "FLOAT": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    "DOUBLE": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    "INT32": mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    "UINT32": mavutil.mavlink.MAV_PARAM_TYPE_UINT32,
    "INT16": mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    "UINT16": mavutil.mavlink.MAV_PARAM_TYPE_UINT16,
    "INT8": mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    "UINT8": mavutil.mavlink.MAV_PARAM_TYPE_UINT8,
    "BOOL": mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    "BOOLEAN": mavutil.mavlink.MAV_PARAM_TYPE_INT32,
}


class PX4Adapter(MavlinkVehicleMixin, VehicleAdapter):
    def __init__(self, config):
        self.config = config
        self.px4_root = Path(config.simulator.get("px4", {}).get("root", "/home/car/PX4-Autopilot"))
        self.sim_cfg = config.simulator.get("px4", {})
        self.log_dir = Path("runs") / config.experiment_id / "sim_logs"
        self.process = None
        self.timing = {}
        self._parameter_metadata_by_name: dict[str, dict] | None = None

    def prepare(self, scenario: ScenarioCfg, seed: int) -> None:
        self.timing = {}
        if self.sim_cfg.get("cleanup_each_run", True):
            self.shutdown()
        speed = float(self.sim_cfg.get("sim_speed_factor", 5.0))
        env = {
            "HEADLESS": "1",
            "PX4_SIM_SPEED_FACTOR": str(speed),
            "PX4_HOME_LAT": f"{47.397742 + seed * 1e-6:.7f}",
            "PX4_HOME_LON": f"{8.545594 + seed * 1e-6:.7f}",
        }
        log_path = self.log_dir / f"px4_seed{seed}.log"
        self.process = start_process(["make", "px4_sitl", "jmavsim"], self.px4_root, env, log_path)
        mav = self._connect(self.sim_cfg.get("mavlink_url", "udpin:127.0.0.1:14540"), timeout_s=90)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)
        self._set_param("COM_RC_IN_MODE", 1, mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        self._set_param("MIS_TAKEOFF_ALT", float(scenario.takeoff_alt_m), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        self._apply_param_overrides(scenario)
        self._arm()
        mav.set_mode("TAKEOFF")
        self._wait_altitude(float(scenario.takeoff_alt_m), tolerance_m=0.8, timeout_s=60)
        self._send_neutral_manual_for(2.0, speed)
        self._set_mode("POSCTL", timeout_s=15)
        self._manual_climb_to_altitude(float(scenario.takeoff_alt_m), speed)
        self._set_scenario_mode(scenario.perturb_mode)
        self._wait_hover_stable(duration_s=1.5, keep_manual_alive=True)

    def run(self, input_sequence: pd.DataFrame, scenario: ScenarioCfg, output_dir: Path) -> Path:
        speed = float(self.sim_cfg.get("sim_speed_factor", 5.0))
        raw_path = self._execute_sequence(input_sequence, scenario, output_dir, speed)
        copy_latest_matching(
            [
                self.px4_root / "build/px4_sitl_default/rootfs/log",
                self.px4_root / "build/px4_sitl_default/rootfs/fs/microsd/log",
                self.px4_root / "build/px4_sitl_default/tmp/rootfs/fs/microsd/log",
            ],
            "*.ulg",
            output_dir / "raw_log.ulg",
        )
        return raw_path

    def parse_log(self, raw_log_path: Path) -> pd.DataFrame:
        return self._parse_telemetry_jsonl(raw_log_path)

    def shutdown(self) -> None:
        self._terminate_process()
        kill_process_patterns(["jmavsim", "px4_sitl", "PX4_SYS_AUTOSTART", "build/px4_sitl_default/bin/px4"])

    def _set_scenario_mode(self, mode: str) -> None:
        self._set_mode(self._scenario_mode_command(mode), timeout_s=15)

    def _send_scenario_mode_command(self, mode: str) -> None:
        self._send_mode_command(self._scenario_mode_command(mode))

    def _scenario_mode_command(self, mode: str) -> str:
        if mode == "Position":
            return "POSCTL"
        if mode == "Hold":
            return "LOITER"
        return mode

    def _apply_param_overrides(self, scenario: ScenarioCfg) -> None:
        overrides = dict(getattr(scenario, "param_overrides", {}) or {})
        for name, value in overrides.items():
            metadata = self._parameter_metadata(name)
            if bool(metadata.get("rebootRequired", False)):
                raise RuntimeError(f"PX4 parameter {name} requires reboot; per-run override cannot guarantee effect")
            param_type = self._param_type_from_metadata(name, metadata)
            if param_type != mavutil.mavlink.MAV_PARAM_TYPE_REAL32 and not math.isclose(
                float(value),
                float(int(value)),
                rel_tol=0.0,
                abs_tol=0.0,
            ):
                raise RuntimeError(f"PX4 parameter {name} has integer type but override value is non-integral: {value}")
            set_ok = self._set_param(name, float(value), param_type)
            if not set_ok:
                raise RuntimeError(f"PX4 parameter override failed to set {name}={value}")
            actual, actual_type = self._read_param(name)
            self._verify_param_value(name, value, actual)
            safe_name = "".join(ch if ch.isalnum() else "_" for ch in name)
            self.timing[f"param_override_{safe_name}_target"] = float(value)
            self.timing[f"param_override_{safe_name}_readback"] = float(actual)
            self.timing[f"param_override_{safe_name}_param_type"] = int(actual_type)
            self.timing[f"param_override_{safe_name}_reboot_required"] = bool(metadata.get("rebootRequired", False))

    def _parameter_metadata(self, name: str) -> dict:
        metadata = self._load_parameter_metadata()
        if name not in metadata:
            raise RuntimeError(f"PX4 parameter metadata not found for override: {name}")
        return metadata[name]

    def _load_parameter_metadata(self) -> dict[str, dict]:
        if self._parameter_metadata_by_name is not None:
            return self._parameter_metadata_by_name
        path = Path(self.sim_cfg.get("parameters_json", self.px4_root / "build/px4_sitl_default/parameters.json"))
        if not path.exists():
            raise RuntimeError(f"PX4 parameter metadata file not found: {path}")
        with path.open("r", encoding="utf-8") as f:
            raw = json.load(f)
        parameters = raw.get("parameters", [])
        self._parameter_metadata_by_name = {str(row["name"]): dict(row) for row in parameters if "name" in row}
        return self._parameter_metadata_by_name

    def _param_type_from_metadata(self, name: str, metadata: dict) -> int:
        type_name = str(metadata.get("type", "Float")).upper()
        if type_name not in PX4_PARAM_TYPE_BY_METADATA:
            raise RuntimeError(f"Unsupported PX4 parameter type for {name}: {metadata.get('type')}")
        return PX4_PARAM_TYPE_BY_METADATA[type_name]

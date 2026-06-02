from __future__ import annotations

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


class PX4Adapter(MavlinkVehicleMixin, VehicleAdapter):
    def __init__(self, config):
        self.config = config
        self.px4_root = Path(config.simulator.get("px4", {}).get("root", "/home/car/PX4-Autopilot"))
        self.sim_cfg = config.simulator.get("px4", {})
        self.log_dir = Path("runs") / config.experiment_id / "sim_logs"
        self.process = None
        self.timing = {}

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

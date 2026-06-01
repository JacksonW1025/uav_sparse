from __future__ import annotations

import time
from pathlib import Path

import pandas as pd
from pymavlink import mavutil

from sparsepilot.config import ScenarioCfg
from sparsepilot.vehicle.base import VehicleAdapter
from sparsepilot.vehicle.mavlink_common import (
    MavlinkVehicleMixin,
    copy_latest_matching,
    kill_process_patterns,
    start_process,
)


class ArduPilotAdapter(MavlinkVehicleMixin, VehicleAdapter):
    def __init__(self, config):
        self.config = config
        self.ap_root = Path(config.simulator.get("ardupilot", {}).get("root", "/home/car/ardupilot"))
        self.sim_cfg = config.simulator.get("ardupilot", {})
        self.log_dir = Path("runs") / config.experiment_id / "sim_logs"
        self.process = None
        self.timing = {}

    def prepare(self, scenario: ScenarioCfg, seed: int) -> None:
        self.timing = {}
        prepare_start = time.monotonic()
        if self.sim_cfg.get("cleanup_each_run", True):
            cleanup_start = time.monotonic()
            self.shutdown()
            self.timing["cleanup_wall_time_s"] = time.monotonic() - cleanup_start
        speed = float(self.sim_cfg.get("sim_speedup", 5.0))
        sim_vehicle = self.ap_root / "Tools/autotest/sim_vehicle.py"
        cmd = [
            str(sim_vehicle),
            "-v",
            "ArduCopter",
            "-f",
            "quad",
            "--no-mavproxy",
            "--speedup",
            str(int(round(speed))),
        ]
        if seed == 0:
            cmd.append("--wipe")
        log_path = self.log_dir / f"ardupilot_seed{seed}.log"
        startup_connect_start = time.monotonic()
        startup_process_wall_time_s = 0.0
        connect_wall_time_s = 0.0
        mav = None
        last_connect_error = None
        for attempt in range(2):
            startup_start = time.monotonic()
            self.process = start_process(cmd, self.ap_root, {}, log_path)
            startup_process_wall_time_s += time.monotonic() - startup_start
            connect_start = time.monotonic()
            try:
                mav = self._connect(self.sim_cfg.get("direct_mavlink_url", "tcp:127.0.0.1:5760"), timeout_s=120)
                connect_wall_time_s += time.monotonic() - connect_start
                self.timing["startup_attempts"] = attempt + 1
                break
            except TimeoutError as exc:
                connect_wall_time_s += time.monotonic() - connect_start
                last_connect_error = exc
                self._terminate_process()
                kill_process_patterns(["sim_vehicle.py", "arducopter", "MAVProxy"])
                time.sleep(2.0)
        if mav is None:
            raise last_connect_error or TimeoutError("Timed out waiting for ArduPilot heartbeat")
        self.timing["startup_process_wall_time_s"] = startup_process_wall_time_s
        self.timing["connect_wall_time_s"] = connect_wall_time_s
        self.timing["startup_connect_wall_time_s"] = time.monotonic() - startup_connect_start
        interval_start = time.monotonic()
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 50)
        self._request_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)
        self.timing["message_interval_wall_time_s"] = time.monotonic() - interval_start
        global_start = time.monotonic()
        self._wait_global_position_ready(timeout_s=30)
        self.timing["global_position_ready_wall_time_s"] = time.monotonic() - global_start
        mode_start = time.monotonic()
        self._set_scenario_mode(scenario.perturb_mode)
        self.timing["prearm_mode_wall_time_s"] = time.monotonic() - mode_start
        arm_start = time.monotonic()
        self._arm()
        self.timing["arm_wall_time_s"] = time.monotonic() - arm_start
        climb_start = time.monotonic()
        self._manual_climb_to_altitude(float(scenario.takeoff_alt_m), speed, timeout_s=80, tolerance_m=0.8)
        self.timing["takeoff_climb_wall_time_s"] = time.monotonic() - climb_start
        stabilize_start = time.monotonic()
        self._set_scenario_mode(scenario.perturb_mode)
        self._wait_hover_stable(duration_s=1.5)
        self.timing["postclimb_mode_stabilize_wall_time_s"] = time.monotonic() - stabilize_start
        self.timing["prepare_total_measured_wall_time_s"] = time.monotonic() - prepare_start

    def run(self, input_sequence: pd.DataFrame, scenario: ScenarioCfg, output_dir: Path) -> Path:
        speed = float(self.sim_cfg.get("sim_speedup", 5.0))
        raw_path = self._execute_sequence(input_sequence, scenario, output_dir, speed)
        copy_latest_matching(
            [
                self.ap_root / "logs",
                self.ap_root / "ArduCopter/logs",
                self.ap_root / "build/sitl/logs",
            ],
            "*.BIN",
            output_dir / "raw_log.BIN",
        )
        return raw_path

    def parse_log(self, raw_log_path: Path) -> pd.DataFrame:
        return self._parse_telemetry_jsonl(raw_log_path)

    def shutdown(self) -> None:
        self._terminate_process()
        kill_process_patterns(["sim_vehicle.py", "arducopter", "MAVProxy"])

    def _set_scenario_mode(self, mode: str) -> None:
        if mode == "Loiter":
            self._set_mode("LOITER", timeout_s=15)
        elif mode == "AltHold":
            self._set_mode("ALT_HOLD", timeout_s=15)
        else:
            self._set_mode(mode, timeout_s=15)

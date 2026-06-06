from __future__ import annotations

import json
import math
import os
import shutil
import signal
import subprocess
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import pandas as pd
from pymavlink import mavutil


CHANNELS = ["roll", "pitch", "yaw", "throttle"]


@dataclass
class TelemetrySample:
    time_s: float
    x_m: float
    y_m: float
    z_m: float
    alt_m: float
    vx_mps: float
    vy_mps: float
    vz_mps: float
    roll_rad: float
    pitch_rad: float
    yaw_rad: float
    roll_rate_rps: float
    pitch_rate_rps: float
    yaw_rate_rps: float
    mode: str
    autopilot: int | None
    base_mode: int | None
    custom_mode: int | None
    px4_main_mode: int | None
    px4_sub_mode: int | None


class MavlinkVehicleMixin:
    process: subprocess.Popen | None = None
    mav: mavutil.mavfile | None = None
    boot_zero_s: float = 0.0
    t_zero_s: float = 0.0
    t_neutral_s: float = 0.0
    last_mode: str = "UNKNOWN"
    last_autopilot: int | None = None
    last_base_mode: int | None = None
    last_custom_mode: int | None = None
    last_px4_main_mode: int | None = None
    last_px4_sub_mode: int | None = None
    last_attitude: dict[str, float]
    last_position: dict[str, float]
    samples: list[dict[str, Any]]

    def _timing_add(self, key: str, value: int | float) -> None:
        timing = getattr(self, "timing", None)
        if isinstance(timing, dict):
            timing[key] = timing.get(key, 0) + value
            self.timing = timing

    def _connect(self, url: str, timeout_s: float = 60.0):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            try:
                mav = mavutil.mavlink_connection(url, autoreconnect=True, source_system=255)
            except OSError:
                time.sleep(1.0)
                continue
            hb = mav.wait_heartbeat(timeout=2)
            if hb is not None:
                mav.target_system = hb.get_srcSystem()
                mav.target_component = hb.get_srcComponent()
                self.mav = mav
                self._update_heartbeat_state(hb)
                return mav
            try:
                mav.close()
            except Exception:
                pass
        raise TimeoutError(f"Timed out waiting for heartbeat on {url}")

    def _request_message_interval(self, message_id: int, hz: float) -> None:
        if self.mav is None:
            return
        interval_us = int(1_000_000 / hz)
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )

    def _set_param(self, name: str, value: float | int, param_type: int, timeout_s: float = 5.0) -> bool:
        if self.mav is None:
            return False
        encoded = name.encode("utf-8")
        param_value = _encode_param_value(value, param_type)
        for _ in range(3):
            self.mav.mav.param_set_send(
                self.mav.target_system,
                mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
                encoded,
                param_value,
                param_type,
            )
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                msg = self.mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is None:
                    continue
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id = param_id.decode("utf-8", errors="ignore")
                if str(param_id).rstrip("\x00") == name:
                    return True
        return False

    def _wait_ack(self, command: int | None = None, timeout_s: float = 5.0):
        if self.mav is None:
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.5)
            if msg is None:
                continue
            if command is None or int(msg.command) == int(command):
                return msg
        return None

    def _arm(self) -> None:
        assert self.mav is not None
        deadline = time.monotonic() + 60.0
        next_arm_request = 0.0
        request_count = 0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_arm_request:
                self.mav.arducopter_arm()
                request_count += 1
                next_arm_request = now + 2.0
            self.mav.wait_heartbeat(timeout=1)
            if self.mav.motors_armed():
                self._timing_add("arm_request_count", request_count)
                self._timing_add("arm_retry_count", max(0, request_count - 1))
                return
        self._timing_add("arm_request_count", request_count)
        self._timing_add("arm_retry_count", max(0, request_count - 1))
        raise TimeoutError("Timed out waiting for motors to arm")

    def _set_mode(self, mode: str, timeout_s: float = 10.0) -> None:
        assert self.mav is not None
        manual_required_px4_modes = {"POSCTL", "Position", "ALTCTL", "MANUAL", "STABILIZED"}
        deadline = time.monotonic() + timeout_s
        next_mode_request = 0.0
        last_ack: str | None = None
        request_count = 0
        if self.last_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 and mode in manual_required_px4_modes:
            prefeed_deadline = time.monotonic() + 0.7
            while time.monotonic() < prefeed_deadline:
                self._send_manual_normalized(0.0, 0.0, 0.0, 0.0)
                time.sleep(0.02)
        while time.monotonic() < deadline:
            now = time.monotonic()
            needs_manual_input = self.last_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 and mode in manual_required_px4_modes
            if needs_manual_input:
                self._send_manual_normalized(0.0, 0.0, 0.0, 0.0)
            if now >= next_mode_request:
                self._send_mode_command(mode)
                request_count += 1
                next_mode_request = now + 0.2
            msg = self.mav.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=True, timeout=0.02 if needs_manual_input else 0.2)
            if msg is None:
                continue
            if msg.get_type() == "COMMAND_ACK":
                if int(getattr(msg, "command", -1)) == int(mavutil.mavlink.MAV_CMD_DO_SET_MODE):
                    result = int(getattr(msg, "result", -1))
                    progress = int(getattr(msg, "progress", 0))
                    result_param2 = int(getattr(msg, "result_param2", 0))
                    last_ack = f"result={result} progress={progress} result_param2={result_param2}"
                continue
            self.last_mode = mavutil.mode_string_v10(msg)
            self.last_autopilot = int(getattr(msg, "autopilot", 0))
            self.last_base_mode = int(getattr(msg, "base_mode", 0))
            self.last_custom_mode = int(getattr(msg, "custom_mode", 0))
            if self.last_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                self.last_px4_main_mode = (self.last_custom_mode & 0xFF0000) >> 16
                self.last_px4_sub_mode = (self.last_custom_mode & 0xFF000000) >> 24
            if self._mode_matches(self.last_mode, mode):
                self._record_mode_timing(mode, request_count)
                return
        self._record_mode_timing(mode, request_count)
        raise TimeoutError(f"Mode did not switch to {mode}; last mode={self.last_mode}; last_ack={last_ack}")

    def _record_mode_timing(self, mode: str, request_count: int) -> None:
        safe_mode = "".join(ch if ch.isalnum() else "_" for ch in str(mode))
        retry_count = max(0, int(request_count) - 1)
        self._timing_add("mode_switch_request_count_total", int(request_count))
        self._timing_add("mode_switch_retry_count_total", retry_count)
        self._timing_add(f"mode_switch_{safe_mode}_request_count", int(request_count))
        self._timing_add(f"mode_switch_{safe_mode}_retry_count", retry_count)

    def _send_mode_command(self, mode: str) -> None:
        assert self.mav is not None
        if self.last_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4 and isinstance(mode, str) and mode in mavutil.px4_map:
            px4_base_mode, custom_mode, custom_sub_mode = mavutil.px4_map[mode]
            self.mav.mav.command_long_send(
                self.mav.target_system,
                self.mav.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                px4_base_mode,
                custom_mode,
                custom_sub_mode,
                0,
                0,
                0,
                0,
            )
            return
        self.mav.set_mode(mode)

    def _mode_matches(self, observed: str, requested: str) -> bool:
        aliases = {
            "POSCTL": {"POSCTL", "POSITION"},
            "Position": {"POSCTL", "POSITION", "Position"},
            "Hold": {"LOITER", "HOLD", "AUTO.LOITER", "Hold"},
            "Loiter": {"LOITER", "Loiter"},
            "AltHold": {"ALT_HOLD", "ALTHOLD", "ALTCTL", "AltHold"},
        }
        if requested in aliases:
            return observed in aliases[requested]
        return observed == requested

    def _wait_altitude(self, target_alt_m: float, tolerance_m: float = 0.7, timeout_s: float = 45.0) -> None:
        deadline = time.monotonic() + timeout_s
        last_alt = float("nan")
        while time.monotonic() < deadline:
            sample = self._recv_telemetry_sample(blocking=True, timeout=0.5)
            if sample is None:
                continue
            last_alt = sample.alt_m
            if abs(last_alt - target_alt_m) <= tolerance_m:
                return
        raise TimeoutError(f"Timed out waiting for altitude {target_alt_m}m; last={last_alt:.2f}m")

    def _wait_global_position_ready(self, timeout_s: float = 30.0) -> None:
        assert self.mav is not None
        deadline = time.monotonic() + timeout_s
        gps_ok = False
        global_ok = False
        while time.monotonic() < deadline:
            msg = self.mav.recv_match(type=["GPS_RAW_INT", "GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True, timeout=0.5)
            if msg is None:
                continue
            if msg.get_type() == "HEARTBEAT":
                self._update_heartbeat_state(msg)
            elif msg.get_type() == "GPS_RAW_INT":
                gps_ok = int(getattr(msg, "fix_type", 0)) >= 3
            elif msg.get_type() == "GLOBAL_POSITION_INT":
                global_ok = True
            if gps_ok and global_ok:
                return
        return

    def _wait_hover_stable(
        self,
        duration_s: float = 2.0,
        max_xy_speed: float = 0.8,
        max_z_speed: float = 0.5,
        keep_manual_alive: bool = False,
    ) -> None:
        deadline = time.monotonic() + 20.0
        stable_since = None
        while time.monotonic() < deadline:
            if keep_manual_alive:
                self._send_manual_normalized(0.0, 0.0, 0.0, 0.0)
            sample = self._recv_telemetry_sample(blocking=True, timeout=0.5)
            if sample is None:
                continue
            xy_speed = math.hypot(sample.vx_mps, sample.vy_mps)
            stable = xy_speed <= max_xy_speed and abs(sample.vz_mps) <= max_z_speed
            if stable and stable_since is None:
                stable_since = time.monotonic()
            elif not stable:
                stable_since = None
            if stable_since is not None and time.monotonic() - stable_since >= duration_s:
                return
        return

    def _manual_climb_to_altitude(
        self,
        target_alt_m: float,
        speed_factor: float,
        timeout_s: float = 45.0,
        tolerance_m: float = 0.5,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        wall_dt = 1.0 / (50.0 * speed_factor)
        last_alt = float("nan")
        while time.monotonic() < deadline:
            sample = self._recv_telemetry_sample(blocking=True, timeout=0.2)
            if sample is not None:
                last_alt = sample.alt_m
                if abs(last_alt - target_alt_m) <= tolerance_m:
                    for _ in range(30):
                        self._send_manual_normalized(0.0, 0.0, 0.0, 0.0)
                        time.sleep(wall_dt)
                    return
            throttle = 0.45 if (not math.isfinite(last_alt) or last_alt < target_alt_m) else -0.25
            self._send_manual_normalized(0.0, 0.0, 0.0, throttle)
            time.sleep(wall_dt)
        raise TimeoutError(f"Timed out manually adjusting altitude to {target_alt_m}m; last={last_alt:.2f}m")

    def _send_manual_normalized(self, roll: float, pitch: float, yaw: float, throttle: float) -> None:
        assert self.mav is not None
        x_pitch = int(np.clip(pitch, -1.0, 1.0) * 1000)
        y_roll = int(np.clip(roll, -1.0, 1.0) * 1000)
        z_throttle = int(np.clip(0.5 + throttle * 0.5, 0.0, 1.0) * 1000)
        r_yaw = int(np.clip(yaw, -1.0, 1.0) * 1000)
        self.mav.mav.manual_control_send(self.mav.target_system, x_pitch, y_roll, z_throttle, r_yaw, 0)

    def _send_neutral_manual_for(self, duration_s: float, speed_factor: float) -> None:
        wall_dt = 1.0 / (50.0 * speed_factor)
        deadline = time.monotonic() + duration_s / speed_factor
        while time.monotonic() < deadline:
            self._send_manual_normalized(0.0, 0.0, 0.0, 0.0)
            time.sleep(wall_dt)

    def _drain_messages(self, sample_time_s: float, manual_values: dict[str, float]) -> None:
        assert self.mav is not None
        while True:
            msg = self.mav.recv_match(blocking=False)
            if msg is None:
                break
            self._handle_message(msg, sample_time_s, manual_values)

    def _recv_telemetry_sample(self, blocking: bool, timeout: float) -> TelemetrySample | None:
        assert self.mav is not None
        msg = self.mav.recv_match(blocking=blocking, timeout=timeout)
        if msg is None:
            return None
        return self._handle_message(msg, None, None)

    def _handle_message(
        self,
        msg,
        sample_time_s: float | None,
        manual_values: dict[str, float] | None,
    ) -> TelemetrySample | None:
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            self._update_heartbeat_state(msg)
            return None
        if msg_type == "ATTITUDE":
            self.last_attitude = {
                "roll_rad": float(msg.roll),
                "pitch_rad": float(msg.pitch),
                "yaw_rad": float(msg.yaw),
                "roll_rate_rps": float(getattr(msg, "rollspeed", 0.0)),
                "pitch_rate_rps": float(getattr(msg, "pitchspeed", 0.0)),
                "yaw_rate_rps": float(getattr(msg, "yawspeed", 0.0)),
            }
            return None
        if msg_type != "LOCAL_POSITION_NED":
            return None

        boot_s = float(getattr(msg, "time_boot_ms", 0)) / 1000.0
        if self.boot_zero_s == 0.0:
            self.boot_zero_s = boot_s
        time_s = max(0.0, boot_s - self.boot_zero_s)
        attitude = getattr(
            self,
            "last_attitude",
            {
                "roll_rad": 0.0,
                "pitch_rad": 0.0,
                "yaw_rad": 0.0,
                "roll_rate_rps": 0.0,
                "pitch_rate_rps": 0.0,
                "yaw_rate_rps": 0.0,
            },
        )
        sample = TelemetrySample(
            time_s=time_s,
            x_m=float(msg.y),
            y_m=float(msg.x),
            z_m=float(-msg.z),
            alt_m=float(-msg.z),
            vx_mps=float(msg.vy),
            vy_mps=float(msg.vx),
            vz_mps=float(-msg.vz),
            roll_rad=attitude["roll_rad"],
            pitch_rad=attitude["pitch_rad"],
            yaw_rad=attitude["yaw_rad"],
            roll_rate_rps=attitude.get("roll_rate_rps", 0.0),
            pitch_rate_rps=attitude.get("pitch_rate_rps", 0.0),
            yaw_rate_rps=attitude.get("yaw_rate_rps", 0.0),
            mode=self.last_mode,
            autopilot=self.last_autopilot,
            base_mode=self.last_base_mode,
            custom_mode=self.last_custom_mode,
            px4_main_mode=self.last_px4_main_mode,
            px4_sub_mode=self.last_px4_sub_mode,
        )
        if sample_time_s is not None and manual_values is not None:
            row = sample.__dict__.copy()
            row["time_s"] = sample_time_s
            for channel in CHANNELS:
                row[f"manual_{channel}"] = float(manual_values[channel])
            row["t_zero_s"] = self.t_zero_s
            row["t_neutral_s"] = self.t_neutral_s
            self.samples.append(row)
        return sample

    def _execute_sequence(self, input_sequence: pd.DataFrame, scenario, output_dir: Path, speed_factor: float) -> Path:
        assert self.mav is not None
        self.samples = []
        self.t_zero_s = 0.0
        self.t_neutral_s = float(input_sequence["t_s"].max() - self.config.input["neutral_tail_s"])
        transition_t_switch_s = getattr(scenario, "t_switch_s", None)
        transition_enabled = transition_t_switch_s is not None
        if scenario.perturb_mode != scenario.observe_mode:
            self._set_scenario_mode(scenario.perturb_mode)

        start = time.monotonic()
        last_sent_idx = -1
        rows = input_sequence.to_dict("records")
        transition_request_count = 0
        transition_first_request_t_s = None
        transition_observed_t_s = None
        next_transition_request_t_s = float(transition_t_switch_s) if transition_enabled else math.inf
        for idx, row in enumerate(rows):
            row_t_s = float(row["t_s"])
            target_wall = start + float(row["t_s"]) / speed_factor
            sleep_s = target_wall - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            manual = {ch: float(row[ch]) for ch in CHANNELS}

            if transition_enabled and row_t_s >= float(transition_t_switch_s):
                if transition_observed_t_s is None and row_t_s >= next_transition_request_t_s:
                    self._send_scenario_mode_command(scenario.observe_mode)
                    transition_request_count += 1
                    if transition_first_request_t_s is None:
                        transition_first_request_t_s = row_t_s
                    next_transition_request_t_s = row_t_s + 0.2

            self._send_manual_normalized(**manual)
            self._drain_messages(row_t_s, manual)
            if transition_enabled and transition_observed_t_s is None and self._mode_matches(
                self.last_mode, scenario.observe_mode
            ):
                transition_observed_t_s = row_t_s
            last_sent_idx = idx
            if (
                not transition_enabled
                and float(row["t_s"]) >= self.t_neutral_s
                and scenario.perturb_mode != scenario.observe_mode
            ):
                self._set_scenario_mode(scenario.observe_mode)
                scenario = _with_same_modes(scenario)
        for _ in range(20):
            neutral = {ch: 0.0 for ch in CHANNELS}
            self._send_manual_normalized(**neutral)
            self._drain_messages(float(rows[last_sent_idx]["t_s"]), neutral)
            time.sleep(0.01)

        if transition_enabled:
            for row in self.samples:
                row["transition_t_switch_s"] = float(transition_t_switch_s)
                row["transition_first_request_t_s"] = transition_first_request_t_s
                row["transition_observed_t_s"] = transition_observed_t_s
                row["transition_request_count"] = transition_request_count
            timing = getattr(self, "timing", {})
            timing["transition_t_switch_s"] = float(transition_t_switch_s)
            timing["transition_first_request_t_s"] = (
                float(transition_first_request_t_s) if transition_first_request_t_s is not None else math.nan
            )
            timing["transition_observed_t_s"] = (
                float(transition_observed_t_s) if transition_observed_t_s is not None else math.nan
            )
            timing["transition_request_count"] = transition_request_count
            self.timing = timing

        raw_path = output_dir / "mavlink_telemetry.jsonl"
        with raw_path.open("w", encoding="utf-8") as f:
            for row in self.samples:
                f.write(json.dumps(row, sort_keys=True) + "\n")
        return raw_path

    def _parse_telemetry_jsonl(self, raw_log_path: Path) -> pd.DataFrame:
        rows = []
        with Path(raw_log_path).open("r", encoding="utf-8") as f:
            for line in f:
                if line.strip():
                    rows.append(json.loads(line))
        if not rows:
            raise ValueError(f"No telemetry samples in {raw_log_path}")
        df = pd.DataFrame(rows).sort_values("time_s").drop_duplicates("time_s", keep="last")
        df = df.reset_index(drop=True)
        for channel in CHANNELS:
            col = f"manual_{channel}"
            if col not in df:
                df[col] = 0.0
        return df

    def _update_heartbeat_state(self, msg) -> None:
        self.last_mode = mavutil.mode_string_v10(msg)
        self.last_autopilot = int(getattr(msg, "autopilot", 0))
        self.last_base_mode = int(getattr(msg, "base_mode", 0))
        self.last_custom_mode = int(getattr(msg, "custom_mode", 0))
        if self.last_autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
            self.last_px4_main_mode = (self.last_custom_mode & 0xFF0000) >> 16
            self.last_px4_sub_mode = (self.last_custom_mode & 0xFF000000) >> 24
        else:
            self.last_px4_main_mode = None
            self.last_px4_sub_mode = None

    def _terminate_process(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
        self.process = None


def start_process(cmd: list[str], cwd: Path, env: dict[str, str], log_path: Path) -> subprocess.Popen:
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_f = log_path.open("w", encoding="utf-8", errors="replace")
    merged_env = os.environ.copy()
    merged_env.update(env)
    return subprocess.Popen(
        cmd,
        cwd=cwd,
        env=merged_env,
        stdout=log_f,
        stderr=subprocess.STDOUT,
        stdin=subprocess.DEVNULL,
        start_new_session=True,
        text=True,
    )


def kill_process_patterns(patterns: list[str]) -> None:
    for pattern in patterns:
        try:
            subprocess.run(["pkill", "-TERM", "-f", pattern], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(0.2)
            subprocess.run(["pkill", "-KILL", "-f", pattern], check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except FileNotFoundError:
            pass


def copy_latest_matching(search_roots: list[Path], pattern: str, output_path: Path) -> Path | None:
    candidates: list[Path] = []
    for root in search_roots:
        if root.exists():
            candidates.extend(root.rglob(pattern))
    if not candidates:
        return None
    latest = max(candidates, key=lambda p: p.stat().st_mtime)
    shutil.copy2(latest, output_path)
    return output_path


def _with_same_modes(scenario):
    from dataclasses import replace

    return replace(scenario, perturb_mode=scenario.observe_mode)


def _encode_param_value(value: float | int, param_type: int) -> float:
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return struct.unpack(">f", struct.pack(">i", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return struct.unpack(">f", struct.pack(">I", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return struct.unpack(">f", struct.pack(">xxh", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return struct.unpack(">f", struct.pack(">xxH", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return struct.unpack(">f", struct.pack(">xxxb", int(value)))[0]
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return struct.unpack(">f", struct.pack(">xxxB", int(value)))[0]
    return float(value)

"""PX4 control-authority cross-stack probe: MPC_TILTMAX_AIR x offboard attitude.

This is intentionally separate from the ArduPilot/DataFlash harnesses. It starts
PX4 SITL, drives OFFBOARD SET_ATTITUDE_TARGET quaternion setpoints, copies the
PX4 ulog, and classifies the run from ulog topics.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import signal
import struct
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil
from pyulog import ULog


THIS = Path(__file__).resolve()
PLANC_ROOT = THIS.parents[1]
REPO_ROOT = THIS.parents[2]
PX4_ROOT = Path(os.environ.get("PX4_ROOT", "/mnt/nvme/px4_work/PX4-Autopilot"))

RESULTS_DIR = PLANC_ROOT / "results"
LOGS_DIR = PLANC_ROOT / "logs"
ANALYSIS_DIR = PLANC_ROOT / "analysis"
WORK_DIR = PLANC_ROOT / "work" / "px4_ctrlauth_v1"

PX4_SHA = "30e763b6780061d70a14894e3e8b06e6a656f9b8"
PX4_TAG = "v1.15.0"
OFFBOARD_NAV_STATE = 14
TILT_LIMIT_DEG = 45.0

ATTITUDE_QUATERNION_TYPE_MASK = (
    mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE
    | mavutil.mavlink.ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE
)

PX4_PARAM_TYPE_BY_NAME = {
    "COM_RC_IN_MODE": mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    "COM_OF_LOSS_T": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    "MPC_TILTMAX_AIR": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
    "MIS_TAKEOFF_ALT": mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
}


@dataclass
class Px4Process:
    process: subprocess.Popen[str]
    stdout_file: Any
    work_dir: Path
    simulator: str
    start_wall_s: float


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def q_from_euler(roll: float, pitch: float, yaw: float) -> list[float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def quat_tilt_deg(q0: np.ndarray, q1: np.ndarray, q2: np.ndarray, q3: np.ndarray) -> np.ndarray:
    norm = np.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
    valid = norm > 1.0e-6
    w = np.where(valid, q0 / norm, np.nan)
    x = np.where(valid, q1 / norm, np.nan)
    y = np.where(valid, q2 / norm, np.nan)
    z_body_world_z = 1.0 - 2.0 * (x * x + y * y)
    z_body_world_z = np.clip(z_body_world_z, -1.0, 1.0)
    return np.degrees(np.arccos(z_body_world_z))


def q_to_roll_pitch_deg(q: list[float]) -> tuple[float, float]:
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return math.degrees(roll), math.degrees(pitch)


def send_gcs_heartbeat(master: mavutil.mavfile) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_attitude_target(
    master: mavutil.mavfile,
    *,
    roll_deg: float,
    pitch_deg: float,
    yaw_deg: float,
    thrust: float,
) -> None:
    q = q_from_euler(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))
    master.mav.set_attitude_target_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        ATTITUDE_QUATERNION_TYPE_MASK,
        q,
        0.0,
        0.0,
        0.0,
        float(thrust),
    )


def send_px4_mode(master: mavutil.mavfile, mode: str) -> None:
    if mode not in mavutil.px4_map:
        raise ValueError(f"Unknown PX4 mode for pymavlink: {mode}")
    px4_base_mode, custom_mode, custom_sub_mode = mavutil.px4_map[mode]
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
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


def wait_mode(
    master: mavutil.mavfile,
    mode: str,
    *,
    timeout_s: float,
    keepalive: Any | None = None,
) -> str:
    deadline = time.monotonic() + timeout_s
    last_mode = "UNKNOWN"
    while time.monotonic() < deadline:
        if keepalive:
            keepalive()
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.1)
        if msg is None:
            continue
        if msg.get_type() == "HEARTBEAT":
            last_mode = mavutil.mode_string_v10(msg)
            if last_mode == mode:
                return last_mode
    raise TimeoutError(f"Timed out waiting for PX4 mode {mode}; last={last_mode}")


def set_mode(
    master: mavutil.mavfile,
    mode: str,
    *,
    timeout_s: float = 15.0,
    keepalive: Any | None = None,
) -> None:
    deadline = time.monotonic() + timeout_s
    start = time.monotonic()
    next_request = 0.0
    last_ack_result: int | None = None
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            send_px4_mode(master, mode)
            next_request = now + 0.5
        if keepalive:
            keepalive()
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.1)
        if msg is not None:
            if msg.get_type() == "HEARTBEAT" and mavutil.mode_string_v10(msg) == mode:
                return
            if msg.get_type() == "COMMAND_ACK" and int(getattr(msg, "command", -1)) == int(mavutil.mavlink.MAV_CMD_DO_SET_MODE):
                last_ack_result = int(getattr(msg, "result", -1))
                if last_ack_result == mavutil.mavlink.MAV_RESULT_DENIED:
                    raise RuntimeError(f"PX4 denied mode switch to {mode}")
        if mode == "OFFBOARD" and (now - start) >= 2.0 and (
            last_ack_result in {None, mavutil.mavlink.MAV_RESULT_ACCEPTED, mavutil.mavlink.MAV_RESULT_IN_PROGRESS}
        ):
            # Some SITL MAVLink streams do not yield a heartbeat back to this
            # receiver during the setpoint stream, while ulog still records a
            # successful OFFBOARD transition. The oracle checks nav_state.
            return
    wait_mode(master, mode, timeout_s=1.0, keepalive=keepalive)


def encode_param_value(value: float | int, param_type: int) -> float:
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


def decode_param_value(value: float, param_type: int) -> float | int:
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_REAL32:
        return float(value)
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT32:
        return int(struct.unpack(">i", struct.pack(">f", float(value)))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT32:
        return int(struct.unpack(">I", struct.pack(">f", float(value)))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT16:
        return int(struct.unpack(">xxh", struct.pack(">f", float(value)))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT16:
        return int(struct.unpack(">xxH", struct.pack(">f", float(value)))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_INT8:
        return int(struct.unpack(">xxxb", struct.pack(">f", float(value)))[0])
    if param_type == mavutil.mavlink.MAV_PARAM_TYPE_UINT8:
        return int(struct.unpack(">xxxB", struct.pack(">f", float(value)))[0])
    return float(value)


def set_param(master: mavutil.mavfile, name: str, value: float | int, timeout_s: float = 5.0) -> float | int:
    param_type = PX4_PARAM_TYPE_BY_NAME.get(name, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    encoded = encode_param_value(value, param_type)
    for _ in range(3):
        master.mav.param_set_send(
            master.target_system,
            mavutil.mavlink.MAV_COMP_ID_AUTOPILOT1,
            name.encode("utf-8"),
            encoded,
            param_type,
        )
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            msg = master.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is None:
                continue
            param_id = msg.param_id.decode("utf-8", "ignore") if isinstance(msg.param_id, bytes) else str(msg.param_id)
            if param_id.rstrip("\x00") == name:
                return decode_param_value(float(msg.param_value), int(msg.param_type))
    raise TimeoutError(f"Timed out setting parameter {name}")


def request_message_interval(master: mavutil.mavfile, message_id: int, hz: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        message_id,
        int(1_000_000 / hz),
        0,
        0,
        0,
        0,
        0,
    )


def arm(master: mavutil.mavfile, timeout_s: float = 30.0) -> None:
    deadline = time.monotonic() + timeout_s
    next_request = 0.0
    while time.monotonic() < deadline:
        send_gcs_heartbeat(master)
        now = time.monotonic()
        if now >= next_request:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1,
                0,
                0,
                0,
                0,
                0,
                0,
            )
            next_request = now + 1.0
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is not None and (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return
    raise TimeoutError("Timed out arming PX4")


def disarm(master: mavutil.mavfile, timeout_s: float = 8.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        send_gcs_heartbeat(master)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
        hb = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if hb is not None and not (hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return True
    return False


def wait_altitude(master: mavutil.mavfile, target_alt_m: float, timeout_s: float = 90.0, tolerance_m: float = 1.0) -> float:
    deadline = time.monotonic() + timeout_s
    last_alt = float("nan")
    while time.monotonic() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["LOCAL_POSITION_NED", "HEARTBEAT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "LOCAL_POSITION_NED":
            last_alt = -float(msg.z)
            if abs(last_alt - target_alt_m) <= tolerance_m:
                return last_alt
    raise TimeoutError(f"Timed out waiting for altitude {target_alt_m:.1f} m; last={last_alt:.2f}")


def start_px4(run_id: str, simulator: str, speed_factor: float) -> Px4Process:
    run_work = WORK_DIR / run_id
    if run_work.exists():
        shutil.rmtree(run_work)
    run_work.mkdir(parents=True, exist_ok=True)
    stdout_path = run_work / "px4_stdout.log"
    stdout_file = stdout_path.open("w", encoding="utf-8")
    env = os.environ.copy()
    env["HEADLESS"] = "1"
    env["PX4_SIM_SPEED_FACTOR"] = str(speed_factor)
    env.setdefault("PX4_HOME_LAT", "47.397742")
    env.setdefault("PX4_HOME_LON", "8.545594")
    env.setdefault("PX4_HOME_ALT", "488.0")
    cmd = ["make", "px4_sitl", simulator]
    (run_work / "start_command.txt").write_text(" ".join(cmd) + "\n", encoding="utf-8")
    process = subprocess.Popen(
        cmd,
        cwd=PX4_ROOT,
        stdout=stdout_file,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
        env=env,
    )
    return Px4Process(process=process, stdout_file=stdout_file, work_dir=run_work, simulator=simulator, start_wall_s=time.time())


def stop_px4(px4: Px4Process | None) -> None:
    if px4 is None:
        return
    proc = px4.process
    if proc.poll() is None:
        try:
            os.killpg(proc.pid, signal.SIGINT)
            proc.wait(timeout=12)
        except Exception:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
                proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                    proc.wait(timeout=5)
                except Exception:
                    pass
    try:
        px4.stdout_file.close()
    except Exception:
        pass


def connect_mavlink(timeout_s: float = 90.0) -> mavutil.mavfile:
    deadline = time.monotonic() + timeout_s
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            master = mavutil.mavlink_connection("udpin:127.0.0.1:14540", autoreconnect=False, source_system=255)
            hb = master.wait_heartbeat(timeout=2)
            if hb is not None:
                master.target_system = hb.get_srcSystem()
                master.target_component = hb.get_srcComponent()
                return master
            master.close()
        except Exception as exc:
            last_error = exc
            time.sleep(0.5)
    raise TimeoutError(f"Timed out waiting for PX4 MAVLink heartbeat: {last_error}")


def copy_latest_ulog(run_id: str, start_wall_s: float) -> Path | None:
    roots = [
        PX4_ROOT / "build/px4_sitl_default/rootfs/log",
        PX4_ROOT / "build/px4_sitl_default/rootfs/fs/microsd/log",
        PX4_ROOT / "build/px4_sitl_default/tmp/rootfs/fs/microsd/log",
    ]
    candidates: list[Path] = []
    for root in roots:
        if root.exists():
            candidates.extend(p for p in root.rglob("*.ulg") if p.stat().st_mtime >= start_wall_s - 5.0)
    if not candidates:
        return None
    latest = max(candidates, key=lambda p: p.stat().st_mtime)
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    dst = LOGS_DIR / f"{run_id}.ulg"
    shutil.copy2(latest, dst)
    return dst


def topic(ulog: ULog, name: str) -> dict[str, np.ndarray] | None:
    for dataset in ulog.data_list:
        if dataset.name == name and dataset.multi_id == 0:
            return {key: np.asarray(value) for key, value in dataset.data.items()}
    return None


def max_or_none(values: np.ndarray) -> float | None:
    if values.size == 0:
        return None
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.max(finite))


def min_or_none(values: np.ndarray) -> float | None:
    if values.size == 0:
        return None
    finite = values[np.isfinite(values)]
    if finite.size == 0:
        return None
    return float(np.min(finite))


def parse_ulog(ulog_path: Path, *, target_roll_deg: float, run_id: str, expected_offboard_s: float) -> dict[str, Any]:
    ulog = ULog(str(ulog_path), [
        "vehicle_attitude",
        "vehicle_attitude_setpoint",
        "vehicle_local_position",
        "vehicle_status",
        "vehicle_land_detected",
        "failure_detector_status",
    ])
    att = topic(ulog, "vehicle_attitude")
    att_sp = topic(ulog, "vehicle_attitude_setpoint")
    lpos = topic(ulog, "vehicle_local_position")
    status = topic(ulog, "vehicle_status")
    land = topic(ulog, "vehicle_land_detected")
    failure_detector = topic(ulog, "failure_detector_status")

    if att is None or att_sp is None or lpos is None or status is None:
        return {
            "run_id": run_id,
            "label": "inconclusive",
            "reason": "missing_required_ulog_topics",
            "topics_present": [d.name for d in ulog.data_list],
        }

    status_t = status["timestamp"].astype(np.int64)
    nav = status["nav_state"].astype(np.int64)
    offboard_mask = nav == OFFBOARD_NAV_STATE
    first_offboard_us = int(status_t[offboard_mask][0]) if np.any(offboard_mask) else None

    sp_t = att_sp["timestamp"].astype(np.int64)
    if first_offboard_us is not None:
        # Bind the oracle to the intended OFFBOARD command stream. The runner
        # switches to LAND immediately after this interval, and that cleanup
        # mode change must not be counted as offboard loss.
        end_window_us = first_offboard_us + int(max(0.1, expected_offboard_s - 0.2) * 1_000_000)
    else:
        end_window_us = None

    if first_offboard_us is not None and end_window_us is not None:
        sp_window_mask = (sp_t >= first_offboard_us) & (sp_t <= end_window_us)
    else:
        sp_window_mask = np.zeros_like(sp_t, dtype=bool)

    if np.any(sp_window_mask):
        last_sp_us = int(sp_t[sp_window_mask][-1])
    else:
        last_sp_us = None

    def in_window(t: np.ndarray, pad_after_s: float = 0.0) -> np.ndarray:
        if first_offboard_us is None or end_window_us is None:
            return np.zeros_like(t, dtype=bool)
        return (t >= first_offboard_us) & (t <= end_window_us + int(pad_after_s * 1_000_000))

    sp_tilt = quat_tilt_deg(att_sp["q_d[0]"], att_sp["q_d[1]"], att_sp["q_d[2]"], att_sp["q_d[3]"])
    att_t = att["timestamp"].astype(np.int64)
    att_tilt = quat_tilt_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    lpos_t = lpos["timestamp"].astype(np.int64)
    alt = -lpos["z"].astype(float)

    sp_win = sp_window_mask if first_offboard_us is not None else np.zeros_like(sp_t, dtype=bool)
    att_win = in_window(att_t)
    lpos_win = in_window(lpos_t)

    max_setpoint_tilt = max_or_none(sp_tilt[sp_win])
    max_achieved_tilt = max_or_none(att_tilt[att_win])
    target_threshold = min(TILT_LIMIT_DEG + 1.0, max(5.0, abs(float(target_roll_deg)) * 0.5))
    target_mask = sp_win & np.isfinite(sp_tilt) & (sp_tilt > target_threshold)
    first_target_setpoint_us = int(sp_t[target_mask][0]) if np.any(target_mask) else None

    start_alt = None
    min_alt = None
    altitude_loss_m = None
    if first_offboard_us is not None and np.any(lpos_win):
        start_mask = (lpos_t >= first_offboard_us) & (lpos_t <= first_offboard_us + 1_000_000)
        start_vals = alt[start_mask & np.isfinite(alt)]
        if start_vals.size == 0:
            start_vals = alt[lpos_win & np.isfinite(alt)][:20]
        if start_vals.size > 0:
            start_alt = float(np.median(start_vals))
        min_alt = min_or_none(alt[lpos_win])
        if start_alt is not None and min_alt is not None:
            altitude_loss_m = float(start_alt - min_alt)

    ground_contact = False
    landed = False
    if land is not None and first_offboard_us is not None and end_window_us is not None:
        land_t = land["timestamp"].astype(np.int64)
        land_win = (land_t >= first_offboard_us) & (land_t <= end_window_us)
        if np.any(land_win):
            ground_contact = bool(np.any(land.get("ground_contact", np.array([], dtype=bool))[land_win]))
            landed = bool(np.any(land.get("landed", np.array([], dtype=bool))[land_win]))

    unclamped_setpoint = bool(max_setpoint_tilt is not None and max_setpoint_tilt > TILT_LIMIT_DEG + 1.0)
    achieved_exceeds_limit = bool(max_achieved_tilt is not None and max_achieved_tilt > TILT_LIMIT_DEG + 1.0)
    consequence_times: list[int] = []
    if first_offboard_us is not None and end_window_us is not None and start_alt is not None and np.any(lpos_win):
        loss = start_alt - alt
        loss_mask = lpos_win & np.isfinite(loss) & (loss > 15.0)
        if np.any(loss_mask):
            consequence_times.append(int(lpos_t[loss_mask][0]))
    if land is not None and first_offboard_us is not None and end_window_us is not None:
        land_t = land["timestamp"].astype(np.int64)
        land_win = (land_t >= first_offboard_us) & (land_t <= end_window_us)
        contact_mask = np.zeros_like(land_t, dtype=bool)
        if "ground_contact" in land:
            contact_mask |= land["ground_contact"].astype(bool)
        if "landed" in land:
            contact_mask |= land["landed"].astype(bool)
        contact_mask &= land_win
        if np.any(contact_mask):
            consequence_times.append(int(land_t[contact_mask][0]))
    first_consequence_us = min(consequence_times) if consequence_times else None
    hard_consequence = bool(first_consequence_us is not None)

    offboard_dropped = False
    nav_states_in_window: list[int] = []
    failsafe = False
    cleanup_land_after_baseline = False
    first_failure_detector_us = None
    first_failure_detector_flags: list[str] = []
    failure_detector_pre_consequence = False
    if first_offboard_us is not None and end_window_us is not None:
        status_win = (status_t >= first_offboard_us) & (status_t <= end_window_us)
        nav_states_in_window = [int(v) for v in nav[status_win].tolist()]
        preventive_end_us = first_consequence_us if first_consequence_us is not None else end_window_us
        preventive_status_win = (status_t >= first_offboard_us) & (status_t < preventive_end_us)
        if np.any(preventive_status_win):
            offboard_dropped = bool(np.any(nav[preventive_status_win] != OFFBOARD_NAV_STATE))
        if "failsafe" in status:
            failsafe = bool(np.any(status["failsafe"][preventive_status_win]))
        if failure_detector is not None:
            fd_t = failure_detector["timestamp"].astype(np.int64)
            fd_flag_names = [
                "fd_roll",
                "fd_pitch",
                "fd_alt",
                "fd_ext",
                "fd_motor",
                "fd_battery",
                "fd_imbalanced_prop",
            ]
            fd_any = np.zeros_like(fd_t, dtype=bool)
            for name in fd_flag_names:
                if name in failure_detector:
                    fd_any |= failure_detector[name].astype(bool)
            fd_window = (fd_t >= first_offboard_us) & (fd_t <= end_window_us) & fd_any
            if np.any(fd_window):
                first_failure_detector_us = int(fd_t[fd_window][0])
                at_first = fd_t == first_failure_detector_us
                first_failure_detector_flags = [
                    name
                    for name in fd_flag_names
                    if name in failure_detector and bool(np.any(failure_detector[name].astype(bool) & at_first))
                ]
                failure_detector_pre_consequence = first_failure_detector_us < preventive_end_us
        if (
            offboard_dropped
            and not hard_consequence
            and abs(float(target_roll_deg)) <= 1.0
            and altitude_loss_m is not None
            and altitude_loss_m < 2.0
            and set(nav_states_in_window).issubset({OFFBOARD_NAV_STATE, 18})
            and not failsafe
            and first_failure_detector_us is None
        ):
            cleanup_land_after_baseline = True
            offboard_dropped = False

    if first_offboard_us is None:
        label = "inconclusive"
        reason = "offboard_not_entered"
    elif offboard_dropped:
        label = "offboard_dropped"
        reason = "nav_state_left_OFFBOARD_before_oracle_window_end"
    elif target_roll_deg > TILT_LIMIT_DEG + 1.0 and not unclamped_setpoint:
        label = "covered"
        reason = "vehicle_attitude_setpoint_did_not_exceed_tilt_limit"
    elif failsafe:
        label = "contract_violated"
        reason = "vehicle_status_failsafe_true_in_oracle_window"
    elif failure_detector_pre_consequence:
        label = "contract_violated"
        reason = "failure_detector_status_true_before_hard_consequence"
    elif hard_consequence and unclamped_setpoint and achieved_exceeds_limit:
        label = "clean_unsafe"
        reason = "unclamped_offboard_attitude_with_hard_consequence_and_no_tier1_violation"
    else:
        label = "clean_safe"
        reason = "no_hard_consequence_in_oracle_window"

    parsed_csv = LOGS_DIR / f"{run_id}_parsed.csv"
    write_parsed_csv(parsed_csv, att, att_tilt, att_sp, sp_tilt, lpos, first_offboard_us, end_window_us)

    return {
        "run_id": run_id,
        "target_roll_deg": target_roll_deg,
        "label": label,
        "reason": reason,
        "ulog": str(ulog_path),
        "parsed_csv": str(parsed_csv),
        "first_offboard_us": first_offboard_us,
        "first_target_setpoint_us": first_target_setpoint_us,
        "last_attitude_setpoint_us": last_sp_us,
        "end_window_us": end_window_us,
        "max_setpoint_tilt_deg": max_setpoint_tilt,
        "max_achieved_tilt_deg": max_achieved_tilt,
        "setpoint_exceeds_mpc_tiltmax_air": unclamped_setpoint,
        "achieved_exceeds_mpc_tiltmax_air": achieved_exceeds_limit,
        "start_alt_m": start_alt,
        "min_alt_m": min_alt,
        "altitude_loss_m": altitude_loss_m,
        "ground_contact": ground_contact,
        "landed": landed,
        "hard_consequence": hard_consequence,
        "first_consequence_us": first_consequence_us,
        "offboard_dropped": offboard_dropped,
        "cleanup_land_after_baseline": cleanup_land_after_baseline,
        "failsafe": failsafe,
        "first_failure_detector_us": first_failure_detector_us,
        "first_failure_detector_flags": first_failure_detector_flags,
        "failure_detector_pre_consequence": failure_detector_pre_consequence,
        "nav_states_in_window": nav_states_in_window,
    }


def write_parsed_csv(
    path: Path,
    att: dict[str, np.ndarray],
    att_tilt: np.ndarray,
    att_sp: dict[str, np.ndarray],
    sp_tilt: np.ndarray,
    lpos: dict[str, np.ndarray],
    first_offboard_us: int | None,
    end_window_us: int | None,
) -> None:
    import csv

    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = first_offboard_us or int(min(att["timestamp"][0], lpos["timestamp"][0]))
    att_rows = []
    for t, tilt in zip(att["timestamp"], att_tilt):
        if first_offboard_us is not None and end_window_us is not None and not (first_offboard_us <= int(t) <= end_window_us):
            continue
        att_rows.append((int(t), "vehicle_attitude", float(tilt), None))
    sp_rows = []
    for t, tilt in zip(att_sp["timestamp"], sp_tilt):
        if first_offboard_us is not None and end_window_us is not None and not (first_offboard_us <= int(t) <= end_window_us):
            continue
        sp_rows.append((int(t), "vehicle_attitude_setpoint", float(tilt), None))
    alt_rows = []
    for t, z in zip(lpos["timestamp"], lpos["z"]):
        if first_offboard_us is not None and end_window_us is not None and not (first_offboard_us <= int(t) <= end_window_us):
            continue
        alt_rows.append((int(t), "vehicle_local_position", None, -float(z)))
    rows = sorted(att_rows + sp_rows + alt_rows, key=lambda r: (r[0], r[1]))
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["time_s_rel_offboard", "topic", "tilt_deg", "alt_m"])
        for timestamp, topic_name, tilt, alt in rows:
            writer.writerow([(timestamp - t0) / 1_000_000.0, topic_name, tilt, alt])


def run_one(args: argparse.Namespace, target_roll_deg: float, index: int) -> dict[str, Any]:
    run_id = f"px4_ctrlauth_v1_r{int(round(target_roll_deg)):03d}_s{index:02d}"
    px4: Px4Process | None = None
    master: mavutil.mavfile | None = None
    live_events: list[dict[str, Any]] = []
    command_profile: list[dict[str, float]] = []
    requested_params: dict[str, float | int] = {}
    readback_params: dict[str, float | int] = {}

    try:
        px4 = start_px4(run_id, args.simulator, args.speed_factor)
        master = connect_mavlink(timeout_s=args.connect_timeout_s)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 30)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 30)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 2)

        requested_params = {
            "COM_RC_IN_MODE": 1,
            "MPC_TILTMAX_AIR": TILT_LIMIT_DEG,
            "MIS_TAKEOFF_ALT": float(args.takeoff_alt_m),
        }
        if args.com_of_loss_t is not None:
            requested_params["COM_OF_LOSS_T"] = float(args.com_of_loss_t)
        for name, value in requested_params.items():
            readback_params[name] = set_param(master, name, value)

        arm(master, timeout_s=45)
        set_mode(master, "TAKEOFF", timeout_s=15)
        reached_alt = wait_altitude(master, float(args.takeoff_alt_m), timeout_s=args.takeoff_timeout_s)

        thrust = float(args.thrust)
        yaw_deg = 0.0
        dt = 1.0 / float(args.stream_hz)

        prestream_end = time.monotonic() + float(args.prestream_s)
        while time.monotonic() < prestream_end:
            send_gcs_heartbeat(master)
            send_attitude_target(master, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw_deg, thrust=thrust)
            time.sleep(dt)

        def offboard_keepalive() -> None:
            send_gcs_heartbeat(master)
            send_attitude_target(master, roll_deg=0.0, pitch_deg=0.0, yaw_deg=yaw_deg, thrust=thrust)

        set_mode(master, "OFFBOARD", timeout_s=20, keepalive=offboard_keepalive)

        phase_start = time.monotonic()
        phases = [
            ("pre_level", 0.0, float(args.offboard_level_s)),
            ("target", float(target_roll_deg), float(args.target_hold_s)),
            ("post_level", 0.0, float(args.post_level_s)),
        ]

        for phase, roll_deg, duration_s in phases:
            phase_deadline = time.monotonic() + duration_s
            while time.monotonic() < phase_deadline:
                send_gcs_heartbeat(master)
                send_attitude_target(master, roll_deg=roll_deg, pitch_deg=0.0, yaw_deg=yaw_deg, thrust=thrust)
                now = time.monotonic()
                command_profile.append({
                    "t_s": now - phase_start,
                    "phase": phase,
                    "roll_deg": roll_deg,
                    "pitch_deg": 0.0,
                    "yaw_deg": yaw_deg,
                    "thrust": thrust,
                })
                while True:
                    msg = master.recv_match(type=["STATUSTEXT", "HEARTBEAT"], blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == "STATUSTEXT":
                        live_events.append({"t_s": time.monotonic() - phase_start, "text": str(getattr(msg, "text", ""))})
                time.sleep(dt)

        command_profile_path = LOGS_DIR / f"{run_id}_command_profile.json"
        write_json(command_profile_path, {"run_id": run_id, "samples": command_profile})

        cleanup = {"land_requested": False, "disarmed": False}
        try:
            set_mode(master, "LAND", timeout_s=5)
            cleanup["land_requested"] = True
        except Exception as exc:
            cleanup["land_error"] = str(exc)
        cleanup["disarmed"] = disarm(master, timeout_s=8)

        try:
            master.close()
        except Exception:
            pass
        master = None
        stop_px4(px4)
        ulog_path = copy_latest_ulog(run_id, px4.start_wall_s if px4 else time.time())
        if ulog_path is None:
            return {
                "run_id": run_id,
                "target_roll_deg": target_roll_deg,
                "label": "inconclusive",
                "reason": "no_ulog_collected",
                "live_events": live_events[-100:],
                "params_requested": requested_params,
                "param_readbacks": readback_params,
                "cleanup": cleanup,
            }

        expected_offboard_s = float(args.offboard_level_s) + float(args.target_hold_s) + float(args.post_level_s)
        parsed = parse_ulog(
            ulog_path,
            target_roll_deg=target_roll_deg,
            run_id=run_id,
            expected_offboard_s=expected_offboard_s,
        )
        parsed.update({
            "simulator": args.simulator,
            "takeoff_alt_m": float(args.takeoff_alt_m),
            "takeoff_reached_alt_m": reached_alt,
            "stream_hz": float(args.stream_hz),
            "thrust": thrust,
            "command_profile": str(command_profile_path),
            "live_events": live_events[-100:],
            "params_requested": requested_params,
            "param_readbacks": readback_params,
            "cleanup": cleanup,
        })
        oracle_path = LOGS_DIR / f"{run_id}_parsed.oracle.json"
        write_json(oracle_path, parsed)
        parsed["oracle_path"] = str(oracle_path)
        return parsed

    except Exception as exc:
        if master is not None:
            try:
                master.close()
            except Exception:
                pass
        stop_px4(px4)
        return {
            "run_id": run_id,
            "target_roll_deg": target_roll_deg,
            "label": "inconclusive",
            "reason": f"harness_error:{type(exc).__name__}:{exc}",
            "live_events": live_events[-100:],
            "params_requested": requested_params,
            "param_readbacks": readback_params,
        }


def plot_results(runs: list[dict[str, Any]]) -> dict[str, str]:
    ANALYSIS_DIR.mkdir(parents=True, exist_ok=True)
    paths: dict[str, str] = {}
    complete = [r for r in runs if r.get("max_achieved_tilt_deg") is not None]
    if complete:
        xs = [float(r["target_roll_deg"]) for r in complete]
        setpoints = [float(r.get("max_setpoint_tilt_deg") or 0.0) for r in complete]
        achieved = [float(r.get("max_achieved_tilt_deg") or 0.0) for r in complete]
        plt.figure(figsize=(7.5, 4.5))
        plt.plot(xs, setpoints, "o-", label="vehicle_attitude_setpoint tilt")
        plt.plot(xs, achieved, "s-", label="achieved vehicle_attitude tilt")
        plt.axhline(TILT_LIMIT_DEG, color="red", linestyle="--", label="MPC_TILTMAX_AIR 45 deg")
        plt.xlabel("commanded roll setpoint (deg)")
        plt.ylabel("max tilt in offboard window (deg)")
        plt.title("PX4 offboard attitude setpoint bypass candidate")
        plt.grid(True, alpha=0.3)
        plt.legend()
        path = ANALYSIS_DIR / "px4_ctrlauth_v1_tilt_vs_command.png"
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()
        paths["tilt_vs_command"] = str(path)

    if runs:
        label_y = {
            "clean_safe": 0,
            "clean_unsafe": 1,
            "covered": 2,
            "contract_violated": 3,
            "offboard_dropped": 4,
            "inconclusive": 5,
        }
        xs = [float(r["target_roll_deg"]) for r in runs]
        ys = [label_y.get(str(r.get("label")), 5) for r in runs]
        plt.figure(figsize=(7.5, 4.0))
        plt.scatter(xs, ys, s=80)
        plt.yticks(list(label_y.values()), list(label_y.keys()))
        plt.xlabel("commanded roll setpoint (deg)")
        plt.ylabel("run label")
        plt.title("PX4 outcome vs offboard attitude command")
        plt.grid(True, axis="x", alpha=0.3)
        path = ANALYSIS_DIR / "px4_ctrlauth_v1_outcome_vs_roll.png"
        plt.tight_layout()
        plt.savefig(path, dpi=160)
        plt.close()
        paths["outcome_vs_roll"] = str(path)
    return paths


def verdict_from_runs(runs: list[dict[str, Any]]) -> str:
    if any(r.get("label") == "clean_unsafe" for r in runs):
        return "REPLICATES"
    if any(r.get("label") == "covered" for r in runs):
        return "COVERED"
    if runs and all(r.get("label") in {"inconclusive", "offboard_dropped"} for r in runs):
        return "INCONCLUSIVE"
    if runs and any(r.get("label") == "clean_safe" and r.get("setpoint_exceeds_mpc_tiltmax_air") for r in runs):
        return "C-ABSENT"
    return "PARTIAL"


def write_report(result: dict[str, Any]) -> None:
    lines = []
    verdict = result["verdict"]
    lines.append(f"# PX4 control-authority cross-stack v1")
    lines.append("")
    lines.append(f"**VERDICT: {verdict}**")
    lines.append("")
    lines.append(f"PX4 target: `{PX4_TAG}` / `{PX4_SHA}`. SITL simulator used: `{result['config']['simulator']}`.")
    lines.append("")
    lines.append("## Phase A")
    lines.append("")
    lines.append("Static audit verdict: `MPC_TILTMAX_AIR x offboard attitude SET_ATTITUDE_TARGET` is **UNCOVERED**. See `planc/results/px4_coverage_matrix.md` and prereg `planc/results/px4_ctrlauth_v1_prereg.json`.")
    lines.append("")
    lines.append("## Phase B Runs")
    lines.append("")
    lines.append("| roll cmd (deg) | label | max sp tilt | max achieved tilt | alt loss m | hard consequence | fd before consequence | reason |")
    lines.append("|---:|---|---:|---:|---:|---|---|---|")
    for run in result["runs"]:
        def fmt(v: Any) -> str:
            return "NA" if v is None else f"{float(v):.2f}"
        lines.append(
            f"| {float(run['target_roll_deg']):.0f} | {run.get('label')} | "
            f"{fmt(run.get('max_setpoint_tilt_deg'))} | {fmt(run.get('max_achieved_tilt_deg'))} | "
            f"{fmt(run.get('altitude_loss_m'))} | {bool(run.get('hard_consequence'))} | "
            f"{bool(run.get('failure_detector_pre_consequence'))} | {run.get('reason')} |"
        )
    lines.append("")
    lines.append("## Artifacts")
    lines.append("")
    for key, path in result.get("plots", {}).items():
        lines.append(f"- {key}: `{path}`")
    for run in result["runs"]:
        lines.append(f"- {run['run_id']}: ulog `{run.get('ulog')}`, oracle `{run.get('oracle_path')}`")
    lines.append("")
    lines.append("## Interpretation")
    lines.append("")
    if verdict == "REPLICATES":
        lines.append("At least one legal offboard attitude command exceeded `MPC_TILTMAX_AIR`, was accepted without setpoint clipping, produced a hard consequence, and stayed Tier-1 clean before the hard consequence in the ulog oracle window.")
        clean_rolls = [
            f"{float(run['target_roll_deg']):.0f}"
            for run in result["runs"]
            if run.get("label") == "clean_unsafe"
        ]
        if clean_rolls:
            lines.append("")
            lines.append(f"Clean witnesses in this campaign: `{', '.join(clean_rolls)}` deg.")
        if any(run.get("label") == "contract_violated" for run in result["runs"]):
            lines.append("")
            lines.append("Higher roll commands also confirmed the setpoint was not clipped, but are not counted as clean witnesses when `failure_detector_status` asserted before the preregistered hard consequence.")
        if any(run.get("cleanup_land_after_baseline") for run in result["runs"]):
            lines.append("")
            lines.append("The 0 deg baseline row ignores the commanded cleanup LAND transition after the stable OFFBOARD segment; it had no hard consequence and no failure-detector assertion.")
    elif verdict == "C-ABSENT":
        lines.append("The static interface gap is present and offboard attitude exceeded `MPC_TILTMAX_AIR`, but this run set did not produce the preregistered hard consequence. This is not COVERED; it is dynamic consequence absent under the tested points.")
    elif verdict == "COVERED":
        lines.append("At least one dynamic run showed the candidate path did not accept an over-limit attitude setpoint as commanded. Treat this as an important reverse result.")
    elif verdict == "INCONCLUSIVE":
        lines.append("The harness did not produce stable dynamic evidence. Phase A remains a static PARTIAL result.")
    else:
        lines.append("Phase A is complete, but Phase B is not sufficient for a dynamic verdict.")
    (RESULTS_DIR / "px4_ctrlauth_v1_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run PX4 MPC_TILTMAX_AIR x offboard attitude cross-stack probe.")
    parser.add_argument("--simulator", default="jmavsim", choices=["jmavsim", "gz_x500"])
    parser.add_argument("--targets", nargs="+", type=float, default=[0.0, 60.0, 90.0, 120.0, 150.0])
    parser.add_argument("--takeoff-alt-m", type=float, default=50.0)
    parser.add_argument("--thrust", type=float, default=0.5)
    parser.add_argument("--stream-hz", type=float, default=50.0)
    parser.add_argument("--prestream-s", type=float, default=2.0)
    parser.add_argument("--offboard-level-s", type=float, default=2.0)
    parser.add_argument("--target-hold-s", type=float, default=8.0)
    parser.add_argument("--post-level-s", type=float, default=5.0)
    parser.add_argument("--speed-factor", type=float, default=1.0)
    parser.add_argument("--connect-timeout-s", type=float, default=90.0)
    parser.add_argument("--takeoff-timeout-s", type=float, default=100.0)
    parser.add_argument("--com-of-loss-t", type=float, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    WORK_DIR.mkdir(parents=True, exist_ok=True)

    runs: list[dict[str, Any]] = []
    for idx, target in enumerate(args.targets):
        print(f"RUN target_roll={target}", flush=True)
        run = run_one(args, target, idx)
        runs.append(run)
        print(f"DONE {run['run_id']} label={run.get('label')} reason={run.get('reason')}", flush=True)

    plots = plot_results(runs)
    result = {
        "experiment": "px4_ctrlauth_v1",
        "generated_at": utc_now(),
        "px4": {
            "tag": PX4_TAG,
            "sha": PX4_SHA,
            "root": str(PX4_ROOT),
        },
        "config": {
            "simulator": args.simulator,
            "targets": [float(v) for v in args.targets],
            "takeoff_alt_m": float(args.takeoff_alt_m),
            "stream_hz": float(args.stream_hz),
            "thrust": float(args.thrust),
            "target_hold_s": float(args.target_hold_s),
            "post_level_s": float(args.post_level_s),
        },
        "phase_a_static": {
            "coverage_matrix": "planc/results/px4_coverage_matrix.md",
            "candidate": "MPC_TILTMAX_AIR x offboard attitude SET_ATTITUDE_TARGET",
            "decision": "uncovered_by_static_source_audit",
        },
        "runs": runs,
        "plots": plots,
    }
    result["verdict"] = verdict_from_runs(runs)
    write_json(RESULTS_DIR / "px4_ctrlauth_v1_result.json", result)
    write_report(result)
    print(f"VERDICT {result['verdict']}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

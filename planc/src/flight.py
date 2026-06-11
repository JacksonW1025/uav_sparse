from __future__ import annotations

import time
from typing import Any

from pymavlink import mavutil

from injector import send_guided_position_target, witness_target


class FlightError(RuntimeError):
    pass


def send_gcs_heartbeat(master) -> None:
    master.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0,
        0,
        0,
    )


def request_streams(master, rate_hz: int = 10) -> None:
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,
        rate_hz,
        1,
    )
    for msg_id in (
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
        mavutil.mavlink.MAVLINK_MSG_ID_FENCE_STATUS,
        mavutil.mavlink.MAVLINK_MSG_ID_STATUSTEXT,
    ):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1_000_000 / max(rate_hz, 1)),
            0,
            0,
            0,
            0,
            0,
        )


def mode_name(master, heartbeat=None) -> str:
    try:
        if heartbeat is None:
            return str(master.flightmode)
        return str(mavutil.mode_string_v10(heartbeat))
    except Exception:
        return "UNKNOWN"


def wait_heartbeat_mode(master, expected: str, timeout_s: float = 15.0) -> str:
    deadline = time.time() + timeout_s
    last = "UNKNOWN"
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is None:
            continue
        last = mode_name(master, msg)
        if last == expected:
            return last
    raise FlightError(f"Timed out waiting for mode {expected}; last mode {last}")


def set_mode(master, mode: str, timeout_s: float = 15.0) -> None:
    mapping = master.mode_mapping()
    if mode not in mapping:
        raise FlightError(f"Mode {mode} is not in ArduCopter mode mapping: {sorted(mapping)}")
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mapping[mode],
    )
    wait_heartbeat_mode(master, mode, timeout_s=timeout_s)


def wait_position(master, timeout_s: float = 20.0) -> Any:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=0.5)
        if msg is not None and msg.lat != 0 and msg.lon != 0:
            return msg
    raise FlightError("Timed out waiting for GLOBAL_POSITION_INT")


def wait_position_stable(master, min_samples: int = 8, timeout_s: float = 35.0) -> None:
    deadline = time.time() + timeout_s
    samples = 0
    last = None
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "GPS_RAW_INT", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT" and msg.lat != 0 and msg.lon != 0:
            samples += 1
            last = msg
            if samples >= min_samples:
                return
        elif msg.get_type() == "GPS_RAW_INT" and getattr(msg, "fix_type", 0) >= 3:
            samples += 1
            if samples >= min_samples:
                return
    raise FlightError(f"Timed out waiting for stable position estimate; samples={samples}, last={last}")


def relative_alt_m(pos_msg: Any) -> float:
    return float(getattr(pos_msg, "relative_alt", 0.0)) / 1000.0


def _send_arm_command(master) -> None:
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


def arm(master, timeout_s: float = 45.0) -> None:
    deadline = time.time() + timeout_s
    next_arm_send = 0.0
    last_status: list[str] = []
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        if time.time() >= next_arm_send:
            _send_arm_command(master)
            next_arm_send = time.time() + 2.0
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            last_status.append(str(getattr(msg, "text", "")))
        if msg.get_type() == "HEARTBEAT" and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return
    raise FlightError(f"Timed out arming; status={last_status[-5:]}")


def command_takeoff(master, alt_m: float) -> None:
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        alt_m,
    )


def wait_altitude(master, alt_m: float, timeout_s: float = 45.0) -> None:
    deadline = time.time() + timeout_s
    last_alt = None
    while time.time() < deadline:
        pos = wait_position(master, timeout_s=2.0)
        last_alt = relative_alt_m(pos)
        if last_alt >= alt_m * 0.90:
            return
    raise FlightError(f"Timed out reaching altitude {alt_m} m; last relative altitude {last_alt}")


def land_and_disarm(master, timeout_s: float = 45.0) -> None:
    try:
        set_mode(master, "LAND", timeout_s=10.0)
    except Exception:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        )
    deadline = time.time() + timeout_s
    low_since = None
    while time.time() < deadline:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "HEARTBEAT"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "GLOBAL_POSITION_INT":
            alt = relative_alt_m(msg)
            if alt < 0.8:
                low_since = low_since or time.time()
                if time.time() - low_since > 2.0:
                    break
            else:
                low_since = None
        if msg.get_type() == "HEARTBEAT" and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return
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
    end = time.time() + 10.0
    while time.time() < end:
        msg = master.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if msg is not None and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            return


def observe(master, max_wait_s: float, post_action_hold_s: float = 8.0) -> dict[str, Any]:
    start = time.time()
    action_seen_at = None
    modes: list[dict[str, Any]] = []
    statustext: list[str] = []
    while time.time() - start < max_wait_s:
        send_gcs_heartbeat(master)
        msg = master.recv_match(type=["HEARTBEAT", "STATUSTEXT", "FENCE_STATUS"], blocking=True, timeout=0.5)
        if msg is None:
            continue
        if msg.get_type() == "STATUSTEXT":
            statustext.append(str(getattr(msg, "text", "")))
        elif msg.get_type() == "HEARTBEAT":
            mode = mode_name(master, msg)
            if not modes or modes[-1]["mode"] != mode:
                modes.append({"wall_s": time.time() - start, "mode": mode})
            if mode in {"RTL", "LAND", "BRAKE", "SMART_RTL"} and action_seen_at is None:
                action_seen_at = time.time()
        if action_seen_at is not None and time.time() - action_seen_at >= post_action_hold_s:
            break
    return {"modes_seen": modes, "statustext": statustext}


def run_flight(master, config: dict[str, Any], motion: str) -> dict[str, Any]:
    request_streams(master)
    wait_position(master, timeout_s=30.0)
    wait_position_stable(master)
    set_mode(master, "GUIDED")
    arm(master)
    alt_m = float(config["experiment"]["takeoff_alt_m"])
    command_takeoff(master, alt_m)
    wait_altitude(master, alt_m)
    set_mode(master, "GUIDED")

    home = config["experiment"]["home"]
    observation = config["experiment"]["observation_s"]
    if motion == "witness_goto":
        target_lat, target_lon = witness_target(config)
        send_guided_position_target(master, target_lat, target_lon, alt_m)
        observed = observe(master, float(observation["witness"]))
        target = {"lat": target_lat, "lon": target_lon, "rel_alt_m": alt_m}
    elif motion == "hover_center":
        send_guided_position_target(master, float(home["lat"]), float(home["lon"]), alt_m)
        observed = observe(master, float(observation["hover"]), post_action_hold_s=999.0)
        target = {"lat": float(home["lat"]), "lon": float(home["lon"]), "rel_alt_m": alt_m}
    else:
        raise FlightError(f"Unknown motion {motion}")

    land_and_disarm(master)
    return {"motion": motion, "target": target, "observed": observed}

from __future__ import annotations

import math
import time
from typing import Any

from pymavlink import mavutil


EARTH_RADIUS_M = 6378137.0


def destination_point(lat_deg: float, lon_deg: float, bearing_deg: float, distance_m: float) -> tuple[float, float]:
    lat1 = math.radians(lat_deg)
    lon1 = math.radians(lon_deg)
    brng = math.radians(bearing_deg)
    dr = distance_m / EARTH_RADIUS_M
    lat2 = math.asin(math.sin(lat1) * math.cos(dr) + math.cos(lat1) * math.sin(dr) * math.cos(brng))
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(dr) * math.cos(lat1),
        math.cos(dr) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def send_guided_position_target(master, lat_deg: float, lon_deg: float, rel_alt_m: float) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_global_int_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        type_mask,
        int(round(lat_deg * 1.0e7)),
        int(round(lon_deg * 1.0e7)),
        float(rel_alt_m),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def velocity_components_ned(speed_m_s: float, bearing_deg: float) -> tuple[float, float]:
    bearing_rad = math.radians(bearing_deg)
    north_m_s = float(speed_m_s) * math.cos(bearing_rad)
    east_m_s = float(speed_m_s) * math.sin(bearing_rad)
    return north_m_s, east_m_s


def send_guided_velocity_local_ned(
    master,
    north_m_s: float,
    east_m_s: float,
    down_m_s: float = 0.0,
) -> None:
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE
        | mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )
    master.mav.set_position_target_local_ned_send(
        int(time.time() * 1000) & 0xFFFFFFFF,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        float(north_m_s),
        float(east_m_s),
        float(down_m_s),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def witness_target(config: dict[str, Any]) -> tuple[float, float]:
    home = config["experiment"]["home"]
    return destination_point(
        float(home["lat"]),
        float(home["lon"]),
        float(config["experiment"]["target_bearing_deg"]),
        float(config["experiment"]["target_distance_m"]),
    )

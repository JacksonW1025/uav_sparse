#!/usr/bin/env bash
set -euo pipefail

pkill -TERM -f jmavsim || true
pkill -TERM -f px4_sitl || true
pkill -TERM -f PX4_SYS_AUTOSTART || true
pkill -TERM -f 'build/px4_sitl_default/bin/px4' || true
pkill -TERM -f sim_vehicle.py || true
pkill -TERM -f arducopter || true
pkill -TERM -f MAVProxy || true
sleep 0.2
pkill -KILL -f jmavsim || true
pkill -KILL -f px4_sitl || true
pkill -KILL -f PX4_SYS_AUTOSTART || true
pkill -KILL -f 'build/px4_sitl_default/bin/px4' || true
pkill -KILL -f sim_vehicle.py || true
pkill -KILL -f arducopter || true
pkill -KILL -f MAVProxy || true
rm -rf /tmp/px4-* || true

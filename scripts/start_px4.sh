#!/usr/bin/env bash
set -euo pipefail

PX4_ROOT="${PX4_ROOT:-/home/car/PX4-Autopilot}"
export HEADLESS="${HEADLESS:-1}"
export PX4_SIM_SPEED_FACTOR="${PX4_SIM_SPEED_FACTOR:-5}"
cd "$PX4_ROOT"
exec make px4_sitl jmavsim

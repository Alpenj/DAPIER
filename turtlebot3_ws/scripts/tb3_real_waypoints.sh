#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# The shared environment fixes the physical robot to domain 73 and prevents a
# stale shell setting from silently selecting simulation time or localhost DDS.
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

# Help must remain usable while the robot is powered off; it cannot move or
# contact an action server, so there is no reason to run the hardware gate.
if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  exec python3 "$SCRIPT_DIR/tb3_real_waypoints.py" "$@"
fi

# Recheck live odometry, motor torque, battery, and LiDAR immediately before a
# multi-waypoint run.  Planning alone is read-only, but using the same gate for
# both modes keeps the command's result representative of the physical robot.
"$SCRIPT_DIR/tb3_real_ready.sh" --mode nav

exec python3 "$SCRIPT_DIR/tb3_real_waypoints.py" "$@"

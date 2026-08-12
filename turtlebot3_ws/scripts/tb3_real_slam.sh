#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)

# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if timeout 6 ros2 node list 2>/dev/null | awk '$0 == "/cartographer_node" {found=1} END {exit !found}'; then
  echo "ERROR: Cartographer is already running. Do not start a second SLAM process." >&2
  exit 1
fi

"$SCRIPT_DIR/tb3_real_ready.sh" --mode slam

echo
echo "Starting real TurtleBot3 SLAM and RViz."
echo "Keep this terminal open. Press Ctrl-C here only after saving the map."
exec ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=false

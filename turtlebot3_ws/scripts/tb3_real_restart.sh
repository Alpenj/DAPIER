#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if timeout 6 ros2 node list 2>/dev/null | awk '$0 == "/cartographer_node" {found=1} END {exit !found}'; then
  echo "ERROR: stop tb3-slam with Ctrl-C before restarting robot bringup." >&2
  exit 1
fi

echo "Restarting TurtleBot3 bringup on the Jetson."
if ! ssh -o BatchMode=yes -o ConnectTimeout=5 "$TURTLEBOT3_NANO_USER@$TURTLEBOT3_NANO_IP" "sudo systemctl restart turtlebot3-bringup.service"; then
  echo "ERROR: could not restart TurtleBot3 bringup on the Jetson." >&2
  exit 1
fi

echo "Waiting for the previous DDS endpoints to disappear."
exec "$SCRIPT_DIR/tb3_real_check.sh"

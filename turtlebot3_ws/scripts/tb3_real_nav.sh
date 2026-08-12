#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if [[ "$#" -ne 1 ]]; then
  echo "Usage: tb3-nav <map_name|absolute_map_yaml>" >&2
  exit 2
fi

if [[ "$1" = /* ]]; then
  map_yaml="$1"
else
  map_name="${1%.yaml}"
  map_yaml="$HOME/maps/$map_name.yaml"
fi

if [[ ! -s "$map_yaml" ]]; then
  echo "ERROR: map YAML not found: $map_yaml" >&2
  exit 1
fi

if ! python3 "$SCRIPT_DIR/tb3_map_validate.py" "$map_yaml"; then
  echo "ERROR: refusing to start Nav2 with an invalid map." >&2
  exit 1
fi

if timeout 6 ros2 node list 2>/dev/null | awk '$0 == "/cartographer_node" {found=1} END {exit !found}'; then
  echo "ERROR: stop tb3-slam with Ctrl-C before starting navigation." >&2
  exit 1
fi

"$SCRIPT_DIR/tb3_real_ready.sh" --mode nav

echo
echo "Starting Nav2 with: $map_yaml"
echo "In RViz: set the initial pose first, then set a navigation goal."
exec ros2 launch dapier_turtlebot3_real tb3_real_navigation.launch.py map:="$map_yaml" use_sim_time:=False

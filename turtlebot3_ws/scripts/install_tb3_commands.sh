#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
BIN_DIR="$HOME/.local/bin"
mkdir -p "$BIN_DIR"

link_command() {
  local command_name=$1
  local script_name=$2
  ln -sfn "$SCRIPT_DIR/$script_name" "$BIN_DIR/$command_name"
  echo "installed: $BIN_DIR/$command_name -> $SCRIPT_DIR/$script_name"
}

link_command tb3-check tb3_real_check.sh
link_command tb3-ready tb3_real_ready.sh
link_command tb3-wheel-test tb3_real_wheel_test.sh
link_command tb3-restart tb3_real_restart.sh
link_command tb3-teleop tb3_real_teleop.sh
link_command tb3-slam tb3_real_slam.sh
link_command tb3-slam-check tb3_real_slam_check.sh
link_command tb3-map-save tb3_real_map_save.sh
link_command tb3-nav tb3_real_nav.sh
link_command tb3-nav-check tb3_real_nav_check.sh
link_command tb3-nav-watch tb3_real_nav_watch.sh
link_command tb3-waypoints tb3_real_waypoints.sh

echo "Done. Open a new terminal or run: export PATH=\"$BIN_DIR:\$PATH\""

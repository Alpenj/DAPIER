#!/usr/bin/env bash
set -eo pipefail

SCRIPT_PATH=$(readlink -f -- "${BASH_SOURCE[0]}")
SCRIPT_DIR=$(cd -- "$(dirname -- "$SCRIPT_PATH")" && pwd)
# shellcheck source=tb3_real_env.sh
source "$SCRIPT_DIR/tb3_real_env.sh"
set -u

if [[ "${1:-}" != "--wheels-lifted" ]]; then
  read -r -p "Lift both drive wheels, then type LIFTED: " confirmation
  if [[ "$confirmation" != "LIFTED" ]]; then
    echo "ERROR: wheel test cancelled; no motion command was sent." >&2
    exit 1
  fi
fi

"$SCRIPT_DIR/tb3_real_check.sh"
exec python3 "$SCRIPT_DIR/tb3_wheel_test.py" --wheels-lifted

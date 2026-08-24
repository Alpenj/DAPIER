#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REPOSITORY_ROOT="$(cd "${PROJECT_ROOT}/.." && pwd)"
MUJOCO_PYTHON="${REPOSITORY_ROOT}/so101_imitation_learning/.venv/bin/python"
PACKAGE_SOURCE="${PROJECT_ROOT}/src/shoe_sorting_data"

if [[ ! -x "${MUJOCO_PYTHON}" ]]; then
  echo "BLOCKED: existing MuJoCo Python was not found: ${MUJOCO_PYTHON}" >&2
  echo "No package was installed." >&2
  exit 1
fi

export PYTHONPATH="${PACKAGE_SOURCE}${PYTHONPATH:+:${PYTHONPATH}}"
exec systemd-inhibit \
  --what=idle \
  --who=DAPIER-MuJoCo-teleop \
  --why="Keep JDcobot USB control alive during teleop" \
  --mode=block \
  gnome-session-inhibit \
  --inhibit idle:suspend \
  "${MUJOCO_PYTHON}" -m shoe_sorting_data.mujoco_hardware_teleop "$@"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_ROOT="${PROJECT_ROOT}/src/shoe_sorting_data"

if ! command -v ros2 >/dev/null 2>&1; then
  for setup_file in /opt/ros/jazzy/setup.bash /opt/ros/humble/setup.bash; do
    if [[ -f "${setup_file}" ]]; then
      # shellcheck disable=SC1090
      source "${setup_file}"
      break
    fi
  done
fi

for command_name in python3 ros2 colcon; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "ERROR: ${command_name} is not available. Source the installed ROS 2 environment first." >&2
    exit 1
  fi
done

if ! python3 -c 'import setuptools' >/dev/null 2>&1; then
  echo "ERROR: Python setuptools is missing; ament_python packages cannot build in this ROS 2 environment." >&2
  echo "Ask the education-PC administrator to restore the ROS 2 Python build dependencies." >&2
  exit 1
fi

echo "Python: $(python3 --version)"
echo "ROS_DISTRO: ${ROS_DISTRO:-unknown}"
echo "Project: ${PROJECT_ROOT}"

cd "${PACKAGE_ROOT}"
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s test -v
PYTHONDONTWRITEBYTECODE=1 python3 -m compileall -q shoe_sorting_data test

cd "${PROJECT_ROOT}"
colcon build --symlink-install --packages-select shoe_sorting_data

# shellcheck disable=SC1091
set +u
source "${PROJECT_ROOT}/install/setup.bash"
set -u
ros2 run shoe_sorting_data shoe_episode --help >/dev/null

echo "PASS: Phase 0 tests, ROS 2 build, and CLI smoke check completed."

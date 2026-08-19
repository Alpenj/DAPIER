#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
course_python="${HOME}/DAPIER/so101_imitation_learning/.venv/bin/python"

if [[ -x "${script_dir}/.venv/bin/python" ]]; then
  python_bin="${script_dir}/.venv/bin/python"
elif [[ -x "${course_python}" ]]; then
  python_bin="${course_python}"
else
  echo "Python environment not found. Follow README.md setup first." >&2
  exit 2
fi

exec "${python_bin}" "${script_dir}/ros_dd_mujoco.py" "$@"

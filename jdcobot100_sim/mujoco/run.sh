#!/usr/bin/env bash
# jdcobot100 MuJoCo 뷰어 실행 (인자는 robot_move.py로 그대로 전달)
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
course_python="${HOME}/DAPIER/so101_imitation_learning/.venv/bin/python"

if [[ -x "${script_dir}/.venv/bin/python" ]]; then
  python_bin="${script_dir}/.venv/bin/python"
elif [[ -x "${course_python}" ]]; then
  python_bin="${course_python}"
else
  echo "Python 환경을 찾을 수 없습니다. README.md의 설치 절차를 먼저 따르세요." >&2
  exit 2
fi

if [[ ! -f "${script_dir}/jdcobot100.xml" ]]; then
  "${python_bin}" "${script_dir}/urdf_to_mjcf.py"
  "${python_bin}" "${script_dir}/tune_mjcf.py"
fi

exec "${python_bin}" "${script_dir}/robot_move.py" "$@"

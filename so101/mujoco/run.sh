#!/usr/bin/env bash
# SO101 MuJoCo 뷰어 실행. 인자는 run_two_poses.py로 전달된다.
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
course_python="${HOME}/DAPIER/so101_imitation_learning/.venv/bin/python"

if [[ -x "${script_dir}/.venv/bin/python" ]]; then
  py="${script_dir}/.venv/bin/python"
elif [[ -x "${course_python}" ]]; then
  py="${course_python}"
else
  echo "Python 환경을 찾을 수 없습니다. README.md의 설치 절차를 먼저 따르세요." >&2
  exit 2
fi

if [[ ! -f "${script_dir}/so101.xml" ]]; then
  "${py}" "${script_dir}/setup_assets.py"
  "${py}" "${script_dir}/urdf_to_mjcf.py"
  "${py}" "${script_dir}/tune_mjcf.py"
fi

exec "${py}" "${script_dir}/run_two_poses.py" "$@"

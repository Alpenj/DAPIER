#!/usr/bin/env bash
# 실습 5의 전체 흐름을 한 번에 재현한다. README의 모든 수치가 여기서 나온다.
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

export MUJOCO_GL="${MUJOCO_GL:-glfw}"
cd "${script_dir}"

echo "===== 0. upstream 자산 확인 및 연결 ====="
"${py}" setup_assets.py
echo
echo "===== 1. URDF -> MJCF 변환 ====="
"${py}" urdf_to_mjcf.py
echo
echo "===== 2. actuator / default / site 추가 ====="
"${py}" tune_mjcf.py
echo
echo "===== 3. 이름 확인 ====="
"${py}" inspect_names.py
echo
echo "===== 4. 데이터시트에서 파라미터 유도 ====="
"${py}" derive_params.py
echo
echo "===== 5. 유도 결과 검증 ====="
"${py}" verify_derivation.py
echo
echo "===== 6. 게인 비교 ====="
"${py}" gain_sweep.py
echo
echo "===== 7. 자세별 스크린샷 ====="
"${py}" render_poses.py

#!/usr/bin/env bash
# 교재 372030 실습의 단계별 비교 실험을 한 번에 재현한다.
# README.md의 표에 들어간 숫자가 전부 여기서 나온다.
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

cd "${script_dir}"

echo "===== 0. URDF -> MJCF 변환 ====="
"${py}" urdf_to_mjcf.py
echo
echo "===== 1. MJCF 튜닝 ====="
"${py}" tune_mjcf.py
echo
echo "===== A. 변환 직후, actuator 없음 ====="
"${py}" stability_check.py build/jdcobot100_raw.xml --seconds 2
echo
echo "===== B. 교재 6장: actuator만 붙임 (mass=1e-09 그대로) ====="
"${py}" stability_check.py build/jdcobot100_naive_actuator.xml --target 0,0.3,0,0 --seconds 2
echo
echo "===== C. 튜닝 모델 + position actuator 정석 (ctrl = 목표각) ====="
"${py}" stability_check.py jdcobot100.xml --target 0,0.3,0,0 --seconds 3
echo
echo "===== D. 튜닝 모델 + 교재 10장 방식 (ctrl = 파이썬 PD 토크) ====="
"${py}" stability_check.py jdcobot100.xml --target 0,0.3,0,0 --seconds 3 --torque-style
echo
echo "===== E. 중력 하에서 자세 유지 (target = 0) ====="
"${py}" stability_check.py jdcobot100.xml --target 0,0,0,0 --seconds 3
echo
echo "===== F. 자세별 스크린샷 + 추종 오차 ====="
MUJOCO_GL="${MUJOCO_GL:-glfw}" "${py}" render_poses.py

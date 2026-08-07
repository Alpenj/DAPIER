#!/usr/bin/env bash
set -u

repo_dir="${LEROBOT_ROOT:-${HOME}/so101/lerobot}"
uv_bin="${UV_BIN:-${HOME}/.local/bin/uv}"

if [[ ! -d "${repo_dir}" ]]; then
  printf 'ERROR: LeRobot checkout not found: %s\n' "${repo_dir}" >&2
  exit 2
fi
if [[ ! -x "${uv_bin}" ]]; then
  printf 'ERROR: uv executable not found: %s\n' "${uv_bin}" >&2
  exit 3
fi

echo "[OS / groups]"
uname -a
id

echo
echo "[serial ports]"
find /dev -maxdepth 1 \( -name 'ttyACM*' -o -name 'ttyUSB*' \) -print 2>/dev/null

echo
echo "[cameras]"
v4l2-ctl --list-devices 2>&1

echo
echo "[NVIDIA]"
nvidia-smi 2>&1 || true

echo
echo "[LeRobot / PyTorch]"
cd "$repo_dir"
"$uv_bin" run python -c \
  'import lerobot, torch; print("lerobot", lerobot.__version__); print("torch", torch.__version__); print("cuda", torch.cuda.is_available()); print("gpu", torch.cuda.get_device_name(0) if torch.cuda.is_available() else "NOT READY")'

echo
echo "[calibration files]"
find "${HOME}/.cache/huggingface/lerobot/calibration" \
  -maxdepth 4 -type f -name '*.json' -print 2>/dev/null

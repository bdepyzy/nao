#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_INSTALL_DIR="${ROOT}/.uv-python"
VENV="${ROOT}/.venv3"
UV_CACHE_DIR="${UV_CACHE_DIR:-/tmp/uv-cache}"
CUVSLAM_WHEEL_URL="${CUVSLAM_WHEEL_URL:-https://github.com/nvidia-isaac/cuVSLAM/releases/download/v16.0.0/cuvslam-16.0.0%2Bcu13-cp312-abi3-manylinux_2_39_x86_64.whl}"

if ! command -v uv >/dev/null 2>&1; then
  echo "uv is required. Install uv first, then rerun this script." >&2
  exit 1
fi

echo "==> Installing Python 3.12 with uv"
env \
  "UV_CACHE_DIR=${UV_CACHE_DIR}" \
  "UV_PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR}" \
  uv python install 3.12

echo "==> Creating cuVSLAM Python 3.12 venv"
env \
  "UV_CACHE_DIR=${UV_CACHE_DIR}" \
  "UV_PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR}" \
  uv venv --python 3.12 "${VENV}"

echo "==> Installing cuVSLAM and Python 3 dependencies"
env \
  "UV_CACHE_DIR=${UV_CACHE_DIR}" \
  "UV_PYTHON_INSTALL_DIR=${PYTHON_INSTALL_DIR}" \
  uv pip install \
    --python "${VENV}/bin/python" \
    --link-mode=copy \
    "${CUVSLAM_WHEEL_URL}" \
    "numpy" \
    "opencv-python" \
    "pyyaml"

echo "==> Verifying cuVSLAM environment"
"${VENV}/bin/python" - <<'PY'
import cv2
import cuvslam
import numpy
print('python ok')
print('cuvslam', cuvslam.get_version())
print('cv2', cv2.__version__)
print('numpy', numpy.__version__)
PY

cat <<EOF

Done.

Use:
  source .venv3/bin/activate
  python cuvslam_nao_controller.py
EOF

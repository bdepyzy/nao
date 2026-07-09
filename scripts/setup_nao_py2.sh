#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PREFIX="${ROOT}/.venv2"
BUILD_ROOT="${TMPDIR:-/tmp}/nao_py2_build"
OPENSSL_PREFIX="${PREFIX}/openssl-1.0"
PYTHON_VERSION="2.7.18"
OPENSSL_VERSION="1.0.2u"
JOBS="${JOBS:-$(nproc 2>/dev/null || echo 4)}"

PYTHON_TGZ="${BUILD_ROOT}/Python-${PYTHON_VERSION}.tgz"
OPENSSL_TGZ="${BUILD_ROOT}/openssl-${OPENSSL_VERSION}.tar.gz"
GET_PIP="${BUILD_ROOT}/get-pip.py"

mkdir -p "${BUILD_ROOT}" "${PREFIX}"

download() {
  local url="$1"
  local out="$2"
  if [ ! -s "${out}" ]; then
    curl -L "${url}" -o "${out}"
  fi
}

echo "==> Building local OpenSSL ${OPENSSL_VERSION}"
download \
  "https://github.com/openssl/openssl/releases/download/OpenSSL_1_0_2u/openssl-${OPENSSL_VERSION}.tar.gz" \
  "${OPENSSL_TGZ}"

rm -rf "${BUILD_ROOT}/openssl-${OPENSSL_VERSION}"
tar -xf "${OPENSSL_TGZ}" -C "${BUILD_ROOT}"
(
  cd "${BUILD_ROOT}/openssl-${OPENSSL_VERSION}"
  ./config shared \
    "--prefix=${OPENSSL_PREFIX}" \
    "--openssldir=${OPENSSL_PREFIX}/ssl"
  make -j "${JOBS}"
  make install_sw
)

echo "==> Building Python ${PYTHON_VERSION} for NAO qi wheels"
download \
  "https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz" \
  "${PYTHON_TGZ}"

rm -rf "${BUILD_ROOT}/Python-${PYTHON_VERSION}"
tar -xf "${PYTHON_TGZ}" -C "${BUILD_ROOT}"
(
  cd "${BUILD_ROOT}/Python-${PYTHON_VERSION}"
  env \
    "CC=gcc -std=gnu17" \
    "CPPFLAGS=-I${OPENSSL_PREFIX}/include" \
    "LDFLAGS=-L${OPENSSL_PREFIX}/lib -Wl,-rpath,${PREFIX}/lib -Wl,-rpath,${OPENSSL_PREFIX}/lib" \
    ./configure \
      "--prefix=${PREFIX}" \
      --enable-shared \
      --enable-unicode=ucs4
  make -j "${JOBS}"
  make install
)

echo "==> Installing pip for Python 2.7"
download "https://bootstrap.pypa.io/pip/2.7/get-pip.py" "${GET_PIP}"
"${PREFIX}/bin/python2.7" "${GET_PIP}" \
  "pip==20.3.4" \
  "setuptools==44.1.1" \
  "wheel==0.37.1"

echo "==> Installing NAO Python dependencies"
PIP_CACHE_DIR="${PIP_CACHE_DIR:-/tmp/pip-cache}" \
  "${PREFIX}/bin/pip" install \
    "qi==1.6.15" \
    "numpy==1.16.6" \
    "opencv-python==4.2.0.32"

echo "==> Writing activation script"
cat > "${PREFIX}/bin/activate" <<ACTIVATE
# This file must be used with "source .venv2/bin/activate".

deactivate () {
    if [ -n "\${_OLD_VIRTUAL_PATH:-}" ] ; then
        PATH="\${_OLD_VIRTUAL_PATH:-}"
        export PATH
        unset _OLD_VIRTUAL_PATH
    fi

    if [ -n "\${_OLD_VIRTUAL_PYTHONHOME:-}" ] ; then
        PYTHONHOME="\${_OLD_VIRTUAL_PYTHONHOME:-}"
        export PYTHONHOME
        unset _OLD_VIRTUAL_PYTHONHOME
    else
        unset PYTHONHOME
    fi

    if [ -n "\${_OLD_VIRTUAL_LD_LIBRARY_PATH+x}" ] ; then
        LD_LIBRARY_PATH="\${_OLD_VIRTUAL_LD_LIBRARY_PATH:-}"
        export LD_LIBRARY_PATH
        unset _OLD_VIRTUAL_LD_LIBRARY_PATH
    else
        unset LD_LIBRARY_PATH
    fi

    if [ -n "\${_OLD_VIRTUAL_XDG_CONFIG_HOME+x}" ] ; then
        XDG_CONFIG_HOME="\${_OLD_VIRTUAL_XDG_CONFIG_HOME:-}"
        export XDG_CONFIG_HOME
        unset _OLD_VIRTUAL_XDG_CONFIG_HOME
    else
        unset XDG_CONFIG_HOME
    fi

    if [ -n "\${_OLD_VIRTUAL_PS1:-}" ] ; then
        PS1="\${_OLD_VIRTUAL_PS1:-}"
        export PS1
        unset _OLD_VIRTUAL_PS1
    fi

    unset VIRTUAL_ENV
    unset -f deactivate
}

VIRTUAL_ENV="${PREFIX}"
export VIRTUAL_ENV

_OLD_VIRTUAL_PATH="\${PATH:-}"
PATH="\$VIRTUAL_ENV/bin:\$PATH"
export PATH

if [ -n "\${PYTHONHOME:-}" ] ; then
    _OLD_VIRTUAL_PYTHONHOME="\${PYTHONHOME:-}"
    unset PYTHONHOME
fi

if [ -n "\${LD_LIBRARY_PATH+x}" ] ; then
    _OLD_VIRTUAL_LD_LIBRARY_PATH="\${LD_LIBRARY_PATH:-}"
else
    unset _OLD_VIRTUAL_LD_LIBRARY_PATH
fi
LD_LIBRARY_PATH="\$VIRTUAL_ENV/lib:\$VIRTUAL_ENV/openssl-1.0/lib:\$VIRTUAL_ENV/lib/python2.7/site-packages/qi/linux\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}"
export LD_LIBRARY_PATH

if [ -n "\${XDG_CONFIG_HOME+x}" ] ; then
    _OLD_VIRTUAL_XDG_CONFIG_HOME="\${XDG_CONFIG_HOME:-}"
else
    unset _OLD_VIRTUAL_XDG_CONFIG_HOME
fi
XDG_CONFIG_HOME="${ROOT}/.qi-config"
export XDG_CONFIG_HOME

if [ -z "\${VIRTUAL_ENV_DISABLE_PROMPT:-}" ] ; then
    _OLD_VIRTUAL_PS1="\${PS1:-}"
    PS1="(.venv2) \${PS1:-}"
    export PS1
fi
ACTIVATE

mkdir -p "${ROOT}/.qi-config"

echo "==> Verifying Python 2 NAO environment"
zsh -lc "source '${PREFIX}/bin/activate'; python - <<'PY'
import sys
import qi
import cv2
import numpy
assert sys.maxunicode == 1114111
print('python', sys.version.split()[0])
print('qi', qi.__version__)
print('cv2', cv2.__version__)
print('numpy', numpy.__version__)
PY"

cat <<EOF

Done.

Use:
  source .venv2/bin/activate
  python nao_motion_bridge.py

Note: the PyPI package is named qi. There is no installable 'naoqi' package on PyPI.
EOF

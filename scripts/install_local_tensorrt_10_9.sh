#!/usr/bin/env bash
# SPDX-License-Identifier: GPL-3.0-or-later
set -euo pipefail

VERSION="${VERSION:-10.9.0.34-1+cuda12.8}"
SDK_DIR="${SDK_DIR:-${HOME}/Software/TensorRT-10.9.0.34-cuda12.8}"
CACHE_DIR="${CACHE_DIR:-${HOME}/.cache/gaussian_lic_ros2/tensorrt-10.9.0.34-cuda12.8-debs}"

packages=(
  "libnvinfer10=${VERSION}"
  "libnvinfer-lean10=${VERSION}"
  "libnvinfer-plugin10=${VERSION}"
  "libnvinfer-vc-plugin10=${VERSION}"
  "libnvinfer-dispatch10=${VERSION}"
  "libnvonnxparsers10=${VERSION}"
  "libnvinfer-bin=${VERSION}"
  "libnvinfer-headers-dev=${VERSION}"
)

mkdir -p "${CACHE_DIR}" "${SDK_DIR}"
cd "${CACHE_DIR}"

for package in "${packages[@]}"; do
  deb_pattern="${package%%=*}_*.deb"
  if compgen -G "${deb_pattern}" >/dev/null; then
    echo "[trt-local] cached ${package}"
  else
    echo "[trt-local] downloading ${package}"
    apt-get download "${package}"
  fi
done

rm -rf "${SDK_DIR:?}/usr" "${SDK_DIR}/include" "${SDK_DIR}/lib" "${SDK_DIR}/bin"
for deb in ./*.deb; do
  echo "[trt-local] extracting ${deb}"
  dpkg-deb -x "${deb}" "${SDK_DIR}"
done

mkdir -p "${SDK_DIR}/include" "${SDK_DIR}/lib" "${SDK_DIR}/bin"
for header in "${SDK_DIR}"/usr/include/x86_64-linux-gnu/*.h; do
  [[ -e "${header}" ]] || continue
  ln -sfn "${header}" "${SDK_DIR}/include/$(basename "${header}")"
done

for lib in \
  "${SDK_DIR}"/usr/lib/x86_64-linux-gnu/libnvinfer*.so.10 \
  "${SDK_DIR}"/usr/lib/x86_64-linux-gnu/libnvonnxparser*.so.10; do
  [[ -e "${lib}" ]] || continue
  base="$(basename "${lib}" .so.10)"
  ln -sfn "${lib}" "${SDK_DIR}/lib/${base}.so.10"
  ln -sfn "${lib}" "${SDK_DIR}/lib/${base}.so"
done

if [[ -x "${SDK_DIR}/usr/src/tensorrt/bin/trtexec" ]]; then
  ln -sfn "${SDK_DIR}/usr/src/tensorrt/bin/trtexec" "${SDK_DIR}/bin/trtexec"
elif [[ -x "${SDK_DIR}/usr/bin/trtexec" ]]; then
  ln -sfn "${SDK_DIR}/usr/bin/trtexec" "${SDK_DIR}/bin/trtexec"
fi

find "${SDK_DIR}" -maxdepth 4 \( -name trtexec -o -name NvInfer.h -o -name 'libnvinfer.so*' \) -print | sort

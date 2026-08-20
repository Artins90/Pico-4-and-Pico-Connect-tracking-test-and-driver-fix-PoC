#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT}"

source <(scripts/setup-codespaces.sh | grep '^LLVM_MINGW_ROOT=')
export LLVM_MINGW_ROOT

cmake -S . -B build-win64 \
  -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="${ROOT}/cmake/llvm-mingw-x86_64.cmake" \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build-win64 --parallel

echo
echo "Windows executable: ${ROOT}/build-win64/Pico4VRMotionTest.exe"

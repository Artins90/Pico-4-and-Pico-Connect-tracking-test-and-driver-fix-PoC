#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS="$ROOT/.tools"
LLVM_VERSION="20260616"
LLVM_NAME="llvm-mingw-${LLVM_VERSION}-ucrt-ubuntu-22.04-x86_64"
LLVM_URL="https://github.com/mstorsjo/llvm-mingw/releases/download/${LLVM_VERSION}/${LLVM_NAME}.tar.xz"

mkdir -p "$TOOLS"

echo "Installing LLVM-MinGW ${LLVM_VERSION}..."

if [ ! -x "$TOOLS/$LLVM_NAME/bin/x86_64-w64-mingw32-clang++" ]; then
    rm -rf "$TOOLS/$LLVM_NAME"
    rm -f "$TOOLS/$LLVM_NAME.tar.xz"

    curl -fL --retry 3 \
        -o "$TOOLS/$LLVM_NAME.tar.xz" \
        "$LLVM_URL"

    tar -xJf "$TOOLS/$LLVM_NAME.tar.xz" -C "$TOOLS"
    rm -f "$TOOLS/$LLVM_NAME.tar.xz"
fi

echo
echo "LLVM-MinGW installed:"
"$TOOLS/$LLVM_NAME/bin/x86_64-w64-mingw32-clang++" --version

echo
echo "Toolchain:"
echo "  $TOOLS/$LLVM_NAME"
#!/bin/sh
# Windows(MSYS2/MinGW64) 本地工具链构建脚本
# 用途：在 C:\Repos_Win11_C 下用本地 MSYS2 工具链构建 cloud_viewer，
#       避免从 Z: 共享盘读取头文件（hgfs 单文件读取延迟约 110ms，是编译变慢的主因）。
# 用法：C:/Repos_Win11_C/Toolchain/msys64/usr/bin/bash.exe --noprofile --norc build-win-msys2.sh [configure|build|all]

set -u

TOOLCHAIN="${CV_MSYS2_ROOT:-/c/Repos_Win11_C/Toolchain/msys64}"
PROJECT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT/build-msys2"

export PATH="$TOOLCHAIN/mingw64/bin:$TOOLCHAIN/usr/bin:/usr/bin:/bin"
# 临时目录必须可写：工具链目录在只读盘上时，cc1plus 拿不到 /tmp 会静默失败。
export TMP="$PROJECT/.build-tmp"
export TEMP="$TMP"
mkdir -p "$TMP"

cd "$PROJECT" || exit 1

ACTION="${1:-all}"

# 原生 cmake.exe 只认 Windows 路径，把 MSYS2 风格路径换过去。
WIN_TOOLCHAIN="$(cygpath -m "$TOOLCHAIN")"

configure() {
  echo "=== configure (local toolchain @ $WIN_TOOLCHAIN) ==="
  cmake -S . -B build-msys2 -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="$WIN_TOOLCHAIN/mingw64/bin/gcc.exe" \
    -DCMAKE_CXX_COMPILER="$WIN_TOOLCHAIN/mingw64/bin/g++.exe" \
    -DVTK_DIR="$WIN_TOOLCHAIN/mingw64/lib/cmake/vtk" \
    -DCLOUD_VIEWER_BUILD_TESTS=OFF
}

build() {
  echo "=== build cloud_viewer ==="
  cmake --build build-msys2 --target cloud_viewer -j 8
}

case "$ACTION" in
  configure) configure ;;
  build)     build ;;
  all)       configure && build ;;
  *) echo "usage: $0 [configure|build|all]"; exit 2 ;;
esac

rc=$?
echo "BUILD_EXIT=$rc"
exit $rc

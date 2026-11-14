#!/usr/bin/env bash
# Builds g1d_ws on Linux: the robot PC2 (Jetson Orin NX, Ubuntu 20.04, aarch64) or an x86_64
# Ubuntu box. Works offline from a copied tree (scripts/pack_for_robot.ps1 / .sh).
#
#   scripts/build_robot.sh [--deps] [--no-livox] [--no-tests] [--jobs N]
#
#   --deps      apt-get the build / Python dependencies first (needs network)
#   --no-livox  build g1d_lio without Livox-SDK2 (lidar.source: unitree_topic only)
#   --no-tests  skip ctest / pytest after building
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="${BUILD_DIR:-$ROOT/build}"
PREFIX="${INSTALL_PREFIX:-$ROOT/install}"
JOBS="$(nproc)"
DEPS=0
LIVOX=ON
TESTS=1
while [ $# -gt 0 ]; do
  case "$1" in
    --deps) DEPS=1 ;;
    --no-livox) LIVOX=OFF ;;
    --no-tests) TESTS=0 ;;
    --jobs) JOBS="$2"; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
  shift
done

if [ "$DEPS" = 1 ]; then
  sudo apt-get update
  sudo apt-get install -y build-essential cmake git python3-pip python3-numpy python3-yaml
fi

# 1. Submodules (only if this is a git checkout and they are missing).
if [ -d "$ROOT/.git" ] && [ ! -f "$ROOT/third_party/eigen/CMakeLists.txt" ]; then
  git -C "$ROOT" submodule update --init --recursive
fi
for d in eigen yaml-cpp unitree_sdk2 unitree_sdk2_python boost_preprocessor; do
  [ -n "$(ls -A "$ROOT/third_party/$d" 2>/dev/null)" ] || { echo "missing third_party/$d" >&2; exit 1; }
done

# 2. Restore the CycloneDDS SONAME symlinks (they become text files in a Windows checkout).
ARCH="$(uname -m)"
DDS_LIB="$ROOT/third_party/unitree_sdk2/thirdparty/lib/$ARCH"
for lib in libddsc libddscxx; do
  f="$DDS_LIB/$lib.so.0"
  if [ ! -L "$f" ] && [ "$(stat -c %s "$f" 2>/dev/null || echo 0)" -lt 1024 ]; then
    ln -sf "$lib.so" "$f"
    echo "restored symlink $f -> $lib.so"
  fi
done

# 3. Configure + build + install.
ARCH_FLAGS=""
if [ "$ARCH" = "aarch64" ] && grep -qi orin /proc/device-tree/model 2>/dev/null; then
  ARCH_FLAGS="-mcpu=cortex-a78ae"
fi
cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release \
  -DG1D_WITH_UNITREE=ON -DG1D_WITH_LIVOX="$LIVOX" -DG1D_ARCH_FLAGS="$ARCH_FLAGS" \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$BUILD" -j "$JOBS"
cmake --install "$BUILD"
echo "installed to $PREFIX/bin: $(ls "$PREFIX/bin" | tr '\n' ' ')"

# 4. Tests (C++ always; Python if numpy / yaml / pytest are importable).
if [ "$TESTS" = 1 ]; then
  (cd "$BUILD" && ctest --output-on-failure)
  if python3 -c "import numpy, yaml, pytest" 2>/dev/null; then
    if python3 -c "import cyclonedds" 2>/dev/null; then
      (cd "$ROOT/python" && python3 -m pytest -q tests)
    else
      echo "cyclonedds (Python) not installed: running the DDS-free Python tests only"
      (cd "$ROOT/python" && python3 -m pytest -q tests/test_guard_core.py tests/test_calib.py)
    fi
  else
    echo "skip Python tests (python3 -m pip install --user pytest)"
  fi
fi

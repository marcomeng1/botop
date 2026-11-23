#!/usr/bin/env bash
# Builds C++ dependencies that conda-forge does not provide (in a usable version)
# into the active pixi environment. Run via: pixi run deps
#
#   fcl    - rai uses the fcl-0.5 (non-templated) API; conda-forge only has 0.6/0.7, pinned to boost 1.74
#   qhull  - conda-forge ships only reentrant libqhull_r; rai uses the non-reentrant API
#            (static PIC lib only; headers come from the conda qhull package)
#   physx  - not on conda-forge
#
# Each library is built once; a stamp file in $CONDA_PREFIX/share/botop-deps marks it done.
# Usage: build_deps.sh [fcl] [physx]   (default: all)
set -euo pipefail

: "${CONDA_PREFIX:?run this inside the pixi environment (pixi run deps)}"
case "$CONDA_PREFIX" in
  */.pixi/envs/*) ;;
  *) echo "refusing to install into non-pixi prefix: $CONDA_PREFIX" >&2; exit 1 ;;
esac

PREFIX="$CONDA_PREFIX"
SRC="${PIXI_PROJECT_ROOT:-$(pwd)}/.pixi/src"
STAMPS="$PREFIX/share/botop-deps"
JOBS="${JOBS:-$(nproc --ignore 2 2>/dev/null || echo 4)}"
mkdir -p "$SRC" "$STAMPS"

clone() {  # clone <url> <ref> <dir>
  [ -d "$3/.git" ] || git clone --depth 1 --single-branch -b "$2" "$1" "$3"
}

ENV_NAME="$(basename "$PREFIX")"

build_fcl() {
  local d="$SRC/fcl"
  local b="$d/build-$ENV_NAME"
  clone https://github.com/flexible-collision-library/fcl.git fcl-0.5 "$d"
  cmake -S "$d" -B "$b" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_PREFIX_PATH="$PREFIX" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -DFCL_STATIC_LIBRARY=ON -DFCL_BUILD_TESTS=OFF -DFCL_WITH_OCTOMAP=OFF \
    -DCMAKE_CXX_FLAGS="-w -std=c++17"
  cmake --build "$b" -j "$JOBS"
  cmake --install "$b"
}

build_qhull() {
  local d="$SRC/qhull"
  local b="$d/build-$ENV_NAME"
  clone https://github.com/qhull/qhull.git 2020.2 "$d"
  cmake -S "$d" -B "$b" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_C_FLAGS="-w" \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
  cmake --build "$b" --target qhullstatic -j "$JOBS"
  install -m 644 "$b/libqhullstatic.a" "$PREFIX/lib/"
}

build_physx() {
  local d="$SRC/PhysX"
  clone https://github.com/NVIDIA-Omniverse/PhysX.git release/104.1 "$d"
  local px="$d/physx"
  (cd "$px/buildtools/packman" && ./packman update -y)
  rm -rf "$px/compiler/linux-"*
  (cd "$px" && PM_clang_PATH="$PREFIX" ./generate_projects.sh linux)
  local b="$px/compiler/linux-release"
  cmake -S "$px/compiler/public" -B "$b" \
    -DPX_BUILDPVDRUNTIME=OFF -DPX_BUILDSNIPPETS=OFF \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" -DCMAKE_CXX_FLAGS="-w"
  cmake --build "$b" -j "$JOBS"
  cmake --install "$b"
  ln -sfn ../bin/linux.clang/release "$PREFIX/lib/PhysX"
}

libs=("$@")
[ ${#libs[@]} -gt 0 ] || libs=(fcl qhull physx)

for lib in "${libs[@]}"; do
  if [ -f "$STAMPS/$lib" ]; then
    echo "[deps] $lib already installed in $PREFIX"
    continue
  fi
  echo "[deps] building $lib into $PREFIX"
  "build_$lib"
  touch "$STAMPS/$lib"
done
